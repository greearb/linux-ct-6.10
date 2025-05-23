// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Support for Intel AES-NI instructions. This file contains glue
 * code, the real AES implementation is in intel-aes_asm.S.
 *
 * Copyright (C) 2008, Intel Corp.
 *    Author: Huang Ying <ying.huang@intel.com>
 *
 * Added RFC4106 AES-GCM support for 128-bit keys under the AEAD
 * interface for 64-bit kernels.
 *    Authors: Adrian Hoban <adrian.hoban@intel.com>
 *             Gabriele Paoloni <gabriele.paoloni@intel.com>
 *             Tadeusz Struk (tadeusz.struk@intel.com)
 *             Aidan O'Mahony (aidan.o.mahony@intel.com)
 *    Copyright (c) 2010, Intel Corporation.
 */

#include <linux/hardirq.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/err.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/ctr.h>
#include <crypto/b128ops.h>
#include <crypto/gcm.h>
#include <crypto/xts.h>
#include <asm/cpu_device_id.h>
#include <asm/simd.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/simd.h>
#include <crypto/internal/skcipher.h>
#include <linux/jump_label.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/static_call.h>


#define AESNI_ALIGN	16
#define AESNI_ALIGN_ATTR __attribute__ ((__aligned__(AESNI_ALIGN)))
#define AES_BLOCK_MASK	(~(AES_BLOCK_SIZE - 1))
#define AESNI_ALIGN_EXTRA ((AESNI_ALIGN - 1) & ~(CRYPTO_MINALIGN - 1))
#define CRYPTO_AES_CTX_SIZE (sizeof(struct crypto_aes_ctx) + AESNI_ALIGN_EXTRA)
#define XTS_AES_CTX_SIZE (sizeof(struct aesni_xts_ctx) + AESNI_ALIGN_EXTRA)

/* This data is stored at the end of the crypto_tfm struct.
 * It's a type of per "session" data storage location.
 * This needs to be 16 byte aligned.
 */
struct aesni_rfc4106_gcm_ctx {
	u8 hash_subkey[16] AESNI_ALIGN_ATTR;
	struct crypto_aes_ctx aes_key_expanded AESNI_ALIGN_ATTR;
	u8 nonce[4];
};

struct generic_gcmaes_ctx {
	u8 hash_subkey[16] AESNI_ALIGN_ATTR;
	struct crypto_aes_ctx aes_key_expanded AESNI_ALIGN_ATTR;
};

struct aesni_xts_ctx {
	struct crypto_aes_ctx tweak_ctx AESNI_ALIGN_ATTR;
	struct crypto_aes_ctx crypt_ctx AESNI_ALIGN_ATTR;
};

#define GCM_BLOCK_LEN 16

struct gcm_context_data {
	/* init, update and finalize context data */
	u8 aad_hash[GCM_BLOCK_LEN];
	u64 aad_length;
	u64 in_length;
	u8 partial_block_enc_key[GCM_BLOCK_LEN];
	u8 orig_IV[GCM_BLOCK_LEN];
	u8 current_counter[GCM_BLOCK_LEN];
	u64 partial_block_len;
	u64 unused;
	u8 hash_keys[GCM_BLOCK_LEN * 16];
};

static inline void *aes_align_addr(void *addr)
{
	if (crypto_tfm_ctx_alignment() >= AESNI_ALIGN)
		return addr;
	return PTR_ALIGN(addr, AESNI_ALIGN);
}

asmlinkage void aesni_set_key(struct crypto_aes_ctx *ctx, const u8 *in_key,
			      unsigned int key_len);
asmlinkage void aesni_enc(const void *ctx, u8 *out, const u8 *in);
asmlinkage void aesni_dec(const void *ctx, u8 *out, const u8 *in);
asmlinkage void aesni_ecb_enc(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len);
asmlinkage void aesni_ecb_dec(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len);
asmlinkage void aesni_cbc_enc(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);
asmlinkage void aesni_cbc_dec(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);
asmlinkage void aesni_cts_cbc_enc(struct crypto_aes_ctx *ctx, u8 *out,
				  const u8 *in, unsigned int len, u8 *iv);
asmlinkage void aesni_cts_cbc_dec(struct crypto_aes_ctx *ctx, u8 *out,
				  const u8 *in, unsigned int len, u8 *iv);

#define AVX_GEN2_OPTSIZE 640
#define AVX_GEN4_OPTSIZE 4096

asmlinkage void aesni_xts_enc(const struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);

asmlinkage void aesni_xts_dec(const struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);

asmlinkage void aesni_ctr_enc(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);
DEFINE_STATIC_CALL(aesni_ctr_enc_tfm, aesni_ctr_enc);

#ifdef CONFIG_X86_64

/* Scatter / Gather routines, with args similar to above */
asmlinkage void aesni_gcm_init(void *ctx,
			       struct gcm_context_data *gdata,
			       u8 *iv,
			       u8 *hash_subkey, const u8 *aad,
			       unsigned long aad_len);
asmlinkage void aesni_gcm_enc_update(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in, unsigned long plaintext_len);
asmlinkage void aesni_gcm_dec_update(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in,
				     unsigned long ciphertext_len);
asmlinkage void aesni_gcm_finalize(void *ctx,
				   struct gcm_context_data *gdata,
				   u8 *auth_tag, unsigned long auth_tag_len);

asmlinkage void aes_ctr_enc_128_avx_by8(const u8 *in, u8 *iv,
		void *keys, u8 *out, unsigned int num_bytes);
asmlinkage void aes_ctr_enc_192_avx_by8(const u8 *in, u8 *iv,
		void *keys, u8 *out, unsigned int num_bytes);
asmlinkage void aes_ctr_enc_256_avx_by8(const u8 *in, u8 *iv,
		void *keys, u8 *out, unsigned int num_bytes);


asmlinkage void aes_xctr_enc_128_avx_by8(const u8 *in, const u8 *iv,
	const void *keys, u8 *out, unsigned int num_bytes,
	unsigned int byte_ctr);

asmlinkage void aes_xctr_enc_192_avx_by8(const u8 *in, const u8 *iv,
	const void *keys, u8 *out, unsigned int num_bytes,
	unsigned int byte_ctr);

asmlinkage void aes_xctr_enc_256_avx_by8(const u8 *in, const u8 *iv,
	const void *keys, u8 *out, unsigned int num_bytes,
	unsigned int byte_ctr);

/*
 * asmlinkage void aesni_gcm_init_avx_gen2()
 * gcm_data *my_ctx_data, context data
 * u8 *hash_subkey,  the Hash sub key input. Data starts on a 16-byte boundary.
 */
asmlinkage void aesni_gcm_init_avx_gen2(void *my_ctx_data,
					struct gcm_context_data *gdata,
					u8 *iv,
					u8 *hash_subkey,
					const u8 *aad,
					unsigned long aad_len);

asmlinkage void aesni_gcm_enc_update_avx_gen2(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in, unsigned long plaintext_len);
asmlinkage void aesni_gcm_dec_update_avx_gen2(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in,
				     unsigned long ciphertext_len);
asmlinkage void aesni_gcm_finalize_avx_gen2(void *ctx,
				   struct gcm_context_data *gdata,
				   u8 *auth_tag, unsigned long auth_tag_len);

/*
 * asmlinkage void aesni_gcm_init_avx_gen4()
 * gcm_data *my_ctx_data, context data
 * u8 *hash_subkey,  the Hash sub key input. Data starts on a 16-byte boundary.
 */
asmlinkage void aesni_gcm_init_avx_gen4(void *my_ctx_data,
					struct gcm_context_data *gdata,
					u8 *iv,
					u8 *hash_subkey,
					const u8 *aad,
					unsigned long aad_len);

asmlinkage void aesni_gcm_enc_update_avx_gen4(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in, unsigned long plaintext_len);
asmlinkage void aesni_gcm_dec_update_avx_gen4(void *ctx,
				     struct gcm_context_data *gdata, u8 *out,
				     const u8 *in,
				     unsigned long ciphertext_len);
asmlinkage void aesni_gcm_finalize_avx_gen4(void *ctx,
				   struct gcm_context_data *gdata,
				   u8 *auth_tag, unsigned long auth_tag_len);

static __ro_after_init DEFINE_STATIC_KEY_FALSE(gcm_use_avx);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(gcm_use_avx2);

static inline struct
aesni_rfc4106_gcm_ctx *aesni_rfc4106_gcm_ctx_get(struct crypto_aead *tfm)
{
	return aes_align_addr(crypto_aead_ctx(tfm));
}

static inline struct
generic_gcmaes_ctx *generic_gcmaes_ctx_get(struct crypto_aead *tfm)
{
	return aes_align_addr(crypto_aead_ctx(tfm));
}
#endif

static inline struct crypto_aes_ctx *aes_ctx(void *raw_ctx)
{
	return aes_align_addr(raw_ctx);
}

static inline struct aesni_xts_ctx *aes_xts_ctx(struct crypto_skcipher *tfm)
{
	return aes_align_addr(crypto_skcipher_ctx(tfm));
}

static int aes_set_key_common(struct crypto_aes_ctx *ctx,
			      const u8 *in_key, unsigned int key_len)
{
	int err;

	if (!crypto_simd_usable())
		return aes_expandkey(ctx, in_key, key_len);

	err = aes_check_keylen(key_len);
	if (err)
		return err;

	kernel_fpu_begin();
	aesni_set_key(ctx, in_key, key_len);
	kernel_fpu_end();
	return 0;
}

static int aes_set_key(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	return aes_set_key_common(aes_ctx(crypto_tfm_ctx(tfm)), in_key,
				  key_len);
}

static void aesni_encrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_tfm_ctx(tfm));

	if (!crypto_simd_usable()) {
		aes_encrypt(ctx, dst, src);
	} else {
		kernel_fpu_begin();
		aesni_enc(ctx, dst, src);
		kernel_fpu_end();
	}
}

static void aesni_decrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_tfm_ctx(tfm));

	if (!crypto_simd_usable()) {
		aes_decrypt(ctx, dst, src);
	} else {
		kernel_fpu_begin();
		aesni_dec(ctx, dst, src);
		kernel_fpu_end();
	}
}

static int aesni_skcipher_setkey(struct crypto_skcipher *tfm, const u8 *key,
			         unsigned int len)
{
	return aes_set_key_common(aes_ctx(crypto_skcipher_ctx(tfm)), key, len);
}

static int ecb_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes)) {
		kernel_fpu_begin();
		aesni_ecb_enc(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			      nbytes & AES_BLOCK_MASK);
		kernel_fpu_end();
		nbytes &= AES_BLOCK_SIZE - 1;
		err = skcipher_walk_done(&walk, nbytes);
	}

	return err;
}

static int ecb_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes)) {
		kernel_fpu_begin();
		aesni_ecb_dec(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			      nbytes & AES_BLOCK_MASK);
		kernel_fpu_end();
		nbytes &= AES_BLOCK_SIZE - 1;
		err = skcipher_walk_done(&walk, nbytes);
	}

	return err;
}

static int cbc_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes)) {
		kernel_fpu_begin();
		aesni_cbc_enc(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			      nbytes & AES_BLOCK_MASK, walk.iv);
		kernel_fpu_end();
		nbytes &= AES_BLOCK_SIZE - 1;
		err = skcipher_walk_done(&walk, nbytes);
	}

	return err;
}

static int cbc_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes)) {
		kernel_fpu_begin();
		aesni_cbc_dec(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			      nbytes & AES_BLOCK_MASK, walk.iv);
		kernel_fpu_end();
		nbytes &= AES_BLOCK_SIZE - 1;
		err = skcipher_walk_done(&walk, nbytes);
	}

	return err;
}

static int aesni_ccm_setkey(struct crypto_aead *tfm, const u8 *in_key,
			    unsigned int key_len)
{
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_aead_ctx(tfm));

	return aes_set_key_common(ctx, in_key, key_len);
}

static int aesni_ccm_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	if ((authsize & 1) || authsize < 4)
		return -EINVAL;
	return 0;
}

static int ccm_set_msg_len(u8 *block, unsigned int msglen, int csize)
{
	__be32 data;

	memset(block, 0, csize);
	block += csize;

	if (csize >= 4)
		csize = 4;
	else if (msglen > (1 << (8 * csize)))
		return -EOVERFLOW;

	data = cpu_to_be32(msglen);
	memcpy(block - csize, (u8 *)&data + 4 - csize, csize);

	return 0;
}

static int ccm_init_mac(struct aead_request *req, u8 maciv[], u32 msglen)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	__be32 *n = (__be32 *)&maciv[AES_BLOCK_SIZE - 8];
	u32 l = req->iv[0] + 1;

	/* verify that CCM dimension 'L' is set correctly in the IV */
	if (l < 2 || l > 8)
		return -EINVAL;

	/* verify that msglen can in fact be represented in L bytes */
	if (l < 4 && msglen >> (8 * l))
		return -EOVERFLOW;

	/*
	 * Even if the CCM spec allows L values of up to 8, the Linux cryptoapi
	 * uses a u32 type to represent msglen so the top 4 bytes are always 0.
	 */
	n[0] = 0;
	n[1] = cpu_to_be32(msglen);

	memcpy(maciv, req->iv, AES_BLOCK_SIZE - l);

	/*
	 * Meaning of byte 0 according to CCM spec (RFC 3610/NIST 800-38C)
	 * - bits 0..2	: max # of bytes required to represent msglen, minus 1
	 *                (already set by caller)
	 * - bits 3..5	: size of auth tag (1 => 4 bytes, 2 => 6 bytes, etc)
	 * - bit 6	: indicates presence of authenticate-only data
	 */
	maciv[0] |= (crypto_aead_authsize(aead) - 2) << 2;
	if (req->assoclen)
		maciv[0] |= 0x40;

	memset(&req->iv[AES_BLOCK_SIZE - l], 0, l);
	return ccm_set_msg_len(maciv + AES_BLOCK_SIZE - l, msglen, l);
}

static int compute_mac(struct crypto_aes_ctx *ctx, u8 mac[], u8 *data, int n,
		       unsigned int ilen, u8 *idata, bool do_simd)
{
	unsigned int bs = AES_BLOCK_SIZE;
	u8 *odata = mac;
	int datalen, getlen;

	datalen = n;

	/* first time in here, block may be partially filled. */
	getlen = bs - ilen;
	if (datalen >= getlen) {
		memcpy(idata + ilen, data, getlen);

		if (likely(do_simd)) {
			aesni_cbc_enc(ctx, odata, idata, AES_BLOCK_SIZE, odata);
		} else {
			crypto_xor(odata, idata, AES_BLOCK_SIZE);
			aes_encrypt(ctx, odata, odata);
		}

		datalen -= getlen;
		data += getlen;
		ilen = 0;
	}

	/* now encrypt rest of data */
	while (datalen >= bs) {
		if (likely(do_simd)) {
			aesni_cbc_enc(ctx, odata, data, AES_BLOCK_SIZE, odata);
		} else {
			crypto_xor(odata, data, AES_BLOCK_SIZE);
			aes_encrypt(ctx, odata, odata);
		}

		datalen -= bs;
		data += bs;
	}

	/* check and see if there's leftover data that wasn't
	 * enough to fill a block.
	 */
	if (datalen) {
		memcpy(idata + ilen, data, datalen);
		ilen += datalen;
	}
	return ilen;
}

static void ccm_calculate_auth_mac(struct aead_request *req,
				   struct crypto_aes_ctx *ctx, u8 mac[],
				   struct scatterlist *src,
				   bool do_simd)
{
	unsigned int len = req->assoclen;
	struct scatter_walk walk;
	u8 idata[AES_BLOCK_SIZE];
	unsigned int ilen;
	struct {
		__be16 l;
		__be32 h;
	} __packed *ltag = (void *)idata;

	/* prepend the AAD with a length tag */
	if (len < 0xff00) {
		ltag->l = cpu_to_be16(len);
		ilen = 2;
	} else  {
		ltag->l = cpu_to_be16(0xfffe);
		ltag->h = cpu_to_be32(len);
		ilen = 6;
	}

	scatterwalk_start(&walk, src);

	while (len) {
		u8 *src;
		int n;

		n = scatterwalk_clamp(&walk, len);
		if (!n) {
			scatterwalk_start(&walk, sg_next(walk.sg));
			n = scatterwalk_clamp(&walk, len);
		}
		src = scatterwalk_map(&walk);

		ilen = compute_mac(ctx, mac, src, n, ilen, idata, do_simd);
		len -= n;

		scatterwalk_unmap(src);
		scatterwalk_advance(&walk, n);
		scatterwalk_done(&walk, 0, len);
	}

	/* any leftover needs padding and then encrypted */
	if (ilen) {
		crypto_xor(mac, idata, ilen);
		if (likely(do_simd))
			aesni_enc(ctx, mac, mac);
		else
			aes_encrypt(ctx, mac, mac);
	}
}

static int aesni_ccm_encrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_aead_ctx(aead));
	bool const do_simd = crypto_simd_usable();
	u8 __aligned(8) mac[AES_BLOCK_SIZE];
	u8 __aligned(8) buf[AES_BLOCK_SIZE];
	struct skcipher_walk walk;
	u32 l = req->iv[0] + 1;
	int err;

	err = ccm_init_mac(req, mac, req->cryptlen);
	if (err)
		return err;

	if (likely(do_simd)) {
		kernel_fpu_begin();
		aesni_enc(ctx, mac, mac);
	} else {
		aes_encrypt(ctx, mac, mac);
	}

	if (req->assoclen)
		ccm_calculate_auth_mac(req, ctx, mac, req->src, do_simd);

	req->iv[AES_BLOCK_SIZE - 1] = 0x1;
	err = skcipher_walk_aead_encrypt(&walk, req, true);

	while (walk.nbytes >= AES_BLOCK_SIZE) {
		int len = walk.nbytes & AES_BLOCK_MASK;
		int n;

		for (n = 0; n < len; n += AES_BLOCK_SIZE) {
			if (likely(do_simd)) {
				aesni_cbc_enc(ctx, mac, walk.src.virt.addr + n,
					      AES_BLOCK_SIZE, mac);
			} else {
				crypto_xor(mac, walk.src.virt.addr + n,
					   AES_BLOCK_SIZE);
				aes_encrypt(ctx, mac, mac);

				aes_encrypt(ctx, buf, walk.iv);
				crypto_inc(walk.iv, AES_BLOCK_SIZE);
				crypto_xor_cpy(walk.dst.virt.addr + n,
					       walk.src.virt.addr + n,
					       buf, AES_BLOCK_SIZE);
			}
		}
		if (likely(do_simd))
			static_call(aesni_ctr_enc_tfm)(ctx, walk.dst.virt.addr,
						       walk.src.virt.addr, len,
						       walk.iv);

		err = skcipher_walk_done(&walk, walk.nbytes & ~AES_BLOCK_MASK);
	}
	if (walk.nbytes) {
		if (likely(do_simd))
			aesni_enc(ctx, buf, walk.iv);
		else
			aes_encrypt(ctx, buf, walk.iv);

		crypto_xor(mac, walk.src.virt.addr, walk.nbytes);
		crypto_xor_cpy(walk.dst.virt.addr, walk.src.virt.addr,
			       buf, walk.nbytes);

		if (likely(do_simd))
			aesni_enc(ctx, mac, mac);
		else
			aes_encrypt(ctx, mac, mac);

		err = skcipher_walk_done(&walk, 0);
	}

	if (err)
		goto fail;

	memset(walk.iv + AES_BLOCK_SIZE - l, 0, l);

	if (likely(do_simd)) {
		static_call(aesni_ctr_enc_tfm)(ctx, mac, mac, AES_BLOCK_SIZE, walk.iv);
	} else {
		aes_encrypt(ctx, buf, walk.iv);
		crypto_xor(mac, buf, AES_BLOCK_SIZE);
	}

	/* copy authtag to end of dst */
	scatterwalk_map_and_copy(mac, req->dst, req->assoclen + req->cryptlen,
				 crypto_aead_authsize(aead), 1);

fail:
	if (likely(do_simd))
		kernel_fpu_end();
	return err;
}

static int aesni_ccm_decrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_aead_ctx(aead));
	unsigned int authsize = crypto_aead_authsize(aead);
	bool const do_simd = crypto_simd_usable();
	u8 __aligned(8) mac[AES_BLOCK_SIZE];
	u8 __aligned(8) tag[AES_BLOCK_SIZE];
	u8 __aligned(8) buf[AES_BLOCK_SIZE];
	struct skcipher_walk walk;
	u32 l = req->iv[0] + 1;
	int err;

	err = ccm_init_mac(req, mac, req->cryptlen - authsize);
	if (err)
		return err;

	/* copy authtag from end of src */
	scatterwalk_map_and_copy(tag, req->src,
				 req->assoclen + req->cryptlen - authsize,
				 authsize, 0);

	if (likely(do_simd)) {
		kernel_fpu_begin();
		aesni_enc(ctx, mac, mac);
	} else {
		aes_encrypt(ctx, mac, mac);
	}

	if (req->assoclen)
		ccm_calculate_auth_mac(req, ctx, mac, req->src, do_simd);

	req->iv[AES_BLOCK_SIZE - 1] = 0x1;
	err = skcipher_walk_aead_decrypt(&walk, req, true);

	while (walk.nbytes >= AES_BLOCK_SIZE) {
		int len = walk.nbytes & AES_BLOCK_MASK;
		int n;

		if (likely(do_simd))
			static_call(aesni_ctr_enc_tfm)(ctx, walk.dst.virt.addr,
						       walk.src.virt.addr, len,
						       walk.iv);

		for (n = 0; n < len; n += AES_BLOCK_SIZE) {
			if (likely(do_simd)) {
				aesni_cbc_enc(ctx, mac, walk.dst.virt.addr + n,
					      AES_BLOCK_SIZE, mac);
			} else {
				aes_encrypt(ctx, buf, walk.iv);
				crypto_inc(walk.iv, AES_BLOCK_SIZE);
				crypto_xor_cpy(walk.dst.virt.addr + n,
					       walk.src.virt.addr + n,
					       buf, AES_BLOCK_SIZE);

				crypto_xor(mac, walk.dst.virt.addr + n,
					   AES_BLOCK_SIZE);
				aes_encrypt(ctx, mac, mac);
			}
		}

		err = skcipher_walk_done(&walk, walk.nbytes & ~AES_BLOCK_MASK);
	}
	if (walk.nbytes) {
		if (likely(do_simd))
			aesni_enc(ctx, buf, walk.iv);
		else
			aes_encrypt(ctx, buf, walk.iv);

		crypto_xor_cpy(walk.dst.virt.addr, walk.src.virt.addr,
			       buf, walk.nbytes);
		crypto_xor(mac, walk.dst.virt.addr, walk.nbytes);

		if (likely(do_simd))
			aesni_enc(ctx, mac, mac);
		else
			aes_encrypt(ctx, mac, mac);

		err = skcipher_walk_done(&walk, 0);
	}

	if (err)
		goto fail;

	memset(walk.iv + AES_BLOCK_SIZE - l, 0, l);

	if (likely(do_simd)) {
		static_call(aesni_ctr_enc_tfm)(ctx, mac, mac, AES_BLOCK_SIZE, walk.iv);
	} else {
		aes_encrypt(ctx, buf, walk.iv);
		crypto_xor(mac, buf, AES_BLOCK_SIZE);
	}

	/* compare calculated auth tag with the stored one */
	if (crypto_memneq(mac, tag, authsize))
		err = -EBADMSG;

fail:
	if (likely(do_simd))
		kernel_fpu_end();
	return err;
}

static int cts_cbc_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	int cbc_blocks = DIV_ROUND_UP(req->cryptlen, AES_BLOCK_SIZE) - 2;
	struct scatterlist *src = req->src, *dst = req->dst;
	struct scatterlist sg_src[2], sg_dst[2];
	struct skcipher_request subreq;
	struct skcipher_walk walk;
	int err;

	skcipher_request_set_tfm(&subreq, tfm);
	skcipher_request_set_callback(&subreq, skcipher_request_flags(req),
				      NULL, NULL);

	if (req->cryptlen <= AES_BLOCK_SIZE) {
		if (req->cryptlen < AES_BLOCK_SIZE)
			return -EINVAL;
		cbc_blocks = 1;
	}

	if (cbc_blocks > 0) {
		skcipher_request_set_crypt(&subreq, req->src, req->dst,
					   cbc_blocks * AES_BLOCK_SIZE,
					   req->iv);

		err = cbc_encrypt(&subreq);
		if (err)
			return err;

		if (req->cryptlen == AES_BLOCK_SIZE)
			return 0;

		dst = src = scatterwalk_ffwd(sg_src, req->src, subreq.cryptlen);
		if (req->dst != req->src)
			dst = scatterwalk_ffwd(sg_dst, req->dst,
					       subreq.cryptlen);
	}

	/* handle ciphertext stealing */
	skcipher_request_set_crypt(&subreq, src, dst,
				   req->cryptlen - cbc_blocks * AES_BLOCK_SIZE,
				   req->iv);

	err = skcipher_walk_virt(&walk, &subreq, false);
	if (err)
		return err;

	kernel_fpu_begin();
	aesni_cts_cbc_enc(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			  walk.nbytes, walk.iv);
	kernel_fpu_end();

	return skcipher_walk_done(&walk, 0);
}

static int cts_cbc_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	int cbc_blocks = DIV_ROUND_UP(req->cryptlen, AES_BLOCK_SIZE) - 2;
	struct scatterlist *src = req->src, *dst = req->dst;
	struct scatterlist sg_src[2], sg_dst[2];
	struct skcipher_request subreq;
	struct skcipher_walk walk;
	int err;

	skcipher_request_set_tfm(&subreq, tfm);
	skcipher_request_set_callback(&subreq, skcipher_request_flags(req),
				      NULL, NULL);

	if (req->cryptlen <= AES_BLOCK_SIZE) {
		if (req->cryptlen < AES_BLOCK_SIZE)
			return -EINVAL;
		cbc_blocks = 1;
	}

	if (cbc_blocks > 0) {
		skcipher_request_set_crypt(&subreq, req->src, req->dst,
					   cbc_blocks * AES_BLOCK_SIZE,
					   req->iv);

		err = cbc_decrypt(&subreq);
		if (err)
			return err;

		if (req->cryptlen == AES_BLOCK_SIZE)
			return 0;

		dst = src = scatterwalk_ffwd(sg_src, req->src, subreq.cryptlen);
		if (req->dst != req->src)
			dst = scatterwalk_ffwd(sg_dst, req->dst,
					       subreq.cryptlen);
	}

	/* handle ciphertext stealing */
	skcipher_request_set_crypt(&subreq, src, dst,
				   req->cryptlen - cbc_blocks * AES_BLOCK_SIZE,
				   req->iv);

	err = skcipher_walk_virt(&walk, &subreq, false);
	if (err)
		return err;

	kernel_fpu_begin();
	aesni_cts_cbc_dec(ctx, walk.dst.virt.addr, walk.src.virt.addr,
			  walk.nbytes, walk.iv);
	kernel_fpu_end();

	return skcipher_walk_done(&walk, 0);
}

#ifdef CONFIG_X86_64
static void aesni_ctr_enc_avx_tfm(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv)
{
	/*
	 * based on key length, override with the by8 version
	 * of ctr mode encryption/decryption for improved performance
	 * aes_set_key_common() ensures that key length is one of
	 * {128,192,256}
	 */
	if (ctx->key_length == AES_KEYSIZE_128)
		aes_ctr_enc_128_avx_by8(in, iv, (void *)ctx, out, len);
	else if (ctx->key_length == AES_KEYSIZE_192)
		aes_ctr_enc_192_avx_by8(in, iv, (void *)ctx, out, len);
	else
		aes_ctr_enc_256_avx_by8(in, iv, (void *)ctx, out, len);
}

static int ctr_crypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	u8 keystream[AES_BLOCK_SIZE];
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes) > 0) {
		kernel_fpu_begin();
		if (nbytes & AES_BLOCK_MASK)
			static_call(aesni_ctr_enc_tfm)(ctx, walk.dst.virt.addr,
						       walk.src.virt.addr,
						       nbytes & AES_BLOCK_MASK,
						       walk.iv);
		nbytes &= ~AES_BLOCK_MASK;

		if (walk.nbytes == walk.total && nbytes > 0) {
			aesni_enc(ctx, keystream, walk.iv);
			crypto_xor_cpy(walk.dst.virt.addr + walk.nbytes - nbytes,
				       walk.src.virt.addr + walk.nbytes - nbytes,
				       keystream, nbytes);
			crypto_inc(walk.iv, AES_BLOCK_SIZE);
			nbytes = 0;
		}
		kernel_fpu_end();
		err = skcipher_walk_done(&walk, nbytes);
	}
	return err;
}

static void aesni_xctr_enc_avx_tfm(struct crypto_aes_ctx *ctx, u8 *out,
				   const u8 *in, unsigned int len, u8 *iv,
				   unsigned int byte_ctr)
{
	if (ctx->key_length == AES_KEYSIZE_128)
		aes_xctr_enc_128_avx_by8(in, iv, (void *)ctx, out, len,
					 byte_ctr);
	else if (ctx->key_length == AES_KEYSIZE_192)
		aes_xctr_enc_192_avx_by8(in, iv, (void *)ctx, out, len,
					 byte_ctr);
	else
		aes_xctr_enc_256_avx_by8(in, iv, (void *)ctx, out, len,
					 byte_ctr);
}

static int xctr_crypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct crypto_aes_ctx *ctx = aes_ctx(crypto_skcipher_ctx(tfm));
	u8 keystream[AES_BLOCK_SIZE];
	struct skcipher_walk walk;
	unsigned int nbytes;
	unsigned int byte_ctr = 0;
	int err;
	__le32 block[AES_BLOCK_SIZE / sizeof(__le32)];

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes) > 0) {
		kernel_fpu_begin();
		if (nbytes & AES_BLOCK_MASK)
			aesni_xctr_enc_avx_tfm(ctx, walk.dst.virt.addr,
				walk.src.virt.addr, nbytes & AES_BLOCK_MASK,
				walk.iv, byte_ctr);
		nbytes &= ~AES_BLOCK_MASK;
		byte_ctr += walk.nbytes - nbytes;

		if (walk.nbytes == walk.total && nbytes > 0) {
			memcpy(block, walk.iv, AES_BLOCK_SIZE);
			block[0] ^= cpu_to_le32(1 + byte_ctr / AES_BLOCK_SIZE);
			aesni_enc(ctx, keystream, (u8 *)block);
			crypto_xor_cpy(walk.dst.virt.addr + walk.nbytes -
				       nbytes, walk.src.virt.addr + walk.nbytes
				       - nbytes, keystream, nbytes);
			byte_ctr += nbytes;
			nbytes = 0;
		}
		kernel_fpu_end();
		err = skcipher_walk_done(&walk, nbytes);
	}
	return err;
}

static int aes_gcm_derive_hash_subkey(const struct crypto_aes_ctx *aes_key,
				      u8 hash_subkey[AES_BLOCK_SIZE])
{
	static const u8 zeroes[AES_BLOCK_SIZE];

	aes_encrypt(aes_key, hash_subkey, zeroes);
	return 0;
}

static int common_rfc4106_set_key(struct crypto_aead *aead, const u8 *key,
				  unsigned int key_len)
{
	struct aesni_rfc4106_gcm_ctx *ctx = aesni_rfc4106_gcm_ctx_get(aead);

	if (key_len < 4)
		return -EINVAL;

	/*Account for 4 byte nonce at the end.*/
	key_len -= 4;

	memcpy(ctx->nonce, key + key_len, sizeof(ctx->nonce));

	return aes_set_key_common(&ctx->aes_key_expanded, key, key_len) ?:
	       aes_gcm_derive_hash_subkey(&ctx->aes_key_expanded,
					  ctx->hash_subkey);
}

/* This is the Integrity Check Value (aka the authentication tag) length and can
 * be 8, 12 or 16 bytes long. */
static int common_rfc4106_set_authsize(struct crypto_aead *aead,
				       unsigned int authsize)
{
	switch (authsize) {
	case 8:
	case 12:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int generic_gcmaes_set_authsize(struct crypto_aead *tfm,
				       unsigned int authsize)
{
	switch (authsize) {
	case 4:
	case 8:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gcmaes_crypt_by_sg(bool enc, struct aead_request *req,
			      unsigned int assoclen, u8 *hash_subkey,
			      u8 *iv, void *aes_ctx, u8 *auth_tag,
			      unsigned long auth_tag_len)
{
	u8 databuf[sizeof(struct gcm_context_data) + (AESNI_ALIGN - 8)] __aligned(8);
	struct gcm_context_data *data = PTR_ALIGN((void *)databuf, AESNI_ALIGN);
	unsigned long left = req->cryptlen;
	struct scatter_walk assoc_sg_walk;
	struct skcipher_walk walk;
	bool do_avx, do_avx2;
	u8 *assocmem = NULL;
	u8 *assoc;
	int err;

	if (!enc)
		left -= auth_tag_len;

	do_avx = (left >= AVX_GEN2_OPTSIZE);
	do_avx2 = (left >= AVX_GEN4_OPTSIZE);

	/* Linearize assoc, if not already linear */
	if (req->src->length >= assoclen && req->src->length) {
		scatterwalk_start(&assoc_sg_walk, req->src);
		assoc = scatterwalk_map(&assoc_sg_walk);
	} else {
		gfp_t flags = (req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP) ?
			      GFP_KERNEL : GFP_ATOMIC;

		/* assoc can be any length, so must be on heap */
		assocmem = kmalloc(assoclen, flags);
		if (unlikely(!assocmem))
			return -ENOMEM;
		assoc = assocmem;

		scatterwalk_map_and_copy(assoc, req->src, 0, assoclen, 0);
	}

	kernel_fpu_begin();
	if (static_branch_likely(&gcm_use_avx2) && do_avx2)
		aesni_gcm_init_avx_gen4(aes_ctx, data, iv, hash_subkey, assoc,
					assoclen);
	else if (static_branch_likely(&gcm_use_avx) && do_avx)
		aesni_gcm_init_avx_gen2(aes_ctx, data, iv, hash_subkey, assoc,
					assoclen);
	else
		aesni_gcm_init(aes_ctx, data, iv, hash_subkey, assoc, assoclen);
	kernel_fpu_end();

	if (!assocmem)
		scatterwalk_unmap(assoc);
	else
		kfree(assocmem);

	err = enc ? skcipher_walk_aead_encrypt(&walk, req, false)
		  : skcipher_walk_aead_decrypt(&walk, req, false);

	while (walk.nbytes > 0) {
		kernel_fpu_begin();
		if (static_branch_likely(&gcm_use_avx2) && do_avx2) {
			if (enc)
				aesni_gcm_enc_update_avx_gen4(aes_ctx, data,
							      walk.dst.virt.addr,
							      walk.src.virt.addr,
							      walk.nbytes);
			else
				aesni_gcm_dec_update_avx_gen4(aes_ctx, data,
							      walk.dst.virt.addr,
							      walk.src.virt.addr,
							      walk.nbytes);
		} else if (static_branch_likely(&gcm_use_avx) && do_avx) {
			if (enc)
				aesni_gcm_enc_update_avx_gen2(aes_ctx, data,
							      walk.dst.virt.addr,
							      walk.src.virt.addr,
							      walk.nbytes);
			else
				aesni_gcm_dec_update_avx_gen2(aes_ctx, data,
							      walk.dst.virt.addr,
							      walk.src.virt.addr,
							      walk.nbytes);
		} else if (enc) {
			aesni_gcm_enc_update(aes_ctx, data, walk.dst.virt.addr,
					     walk.src.virt.addr, walk.nbytes);
		} else {
			aesni_gcm_dec_update(aes_ctx, data, walk.dst.virt.addr,
					     walk.src.virt.addr, walk.nbytes);
		}
		kernel_fpu_end();

		err = skcipher_walk_done(&walk, 0);
	}

	if (err)
		return err;

	kernel_fpu_begin();
	if (static_branch_likely(&gcm_use_avx2) && do_avx2)
		aesni_gcm_finalize_avx_gen4(aes_ctx, data, auth_tag,
					    auth_tag_len);
	else if (static_branch_likely(&gcm_use_avx) && do_avx)
		aesni_gcm_finalize_avx_gen2(aes_ctx, data, auth_tag,
					    auth_tag_len);
	else
		aesni_gcm_finalize(aes_ctx, data, auth_tag, auth_tag_len);
	kernel_fpu_end();

	return 0;
}

static int gcmaes_encrypt(struct aead_request *req, unsigned int assoclen,
			  u8 *hash_subkey, u8 *iv, void *aes_ctx)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	unsigned long auth_tag_len = crypto_aead_authsize(tfm);
	u8 auth_tag[16];
	int err;

	err = gcmaes_crypt_by_sg(true, req, assoclen, hash_subkey, iv, aes_ctx,
				 auth_tag, auth_tag_len);
	if (err)
		return err;

	scatterwalk_map_and_copy(auth_tag, req->dst,
				 req->assoclen + req->cryptlen,
				 auth_tag_len, 1);
	return 0;
}

static int gcmaes_decrypt(struct aead_request *req, unsigned int assoclen,
			  u8 *hash_subkey, u8 *iv, void *aes_ctx)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	unsigned long auth_tag_len = crypto_aead_authsize(tfm);
	u8 auth_tag_msg[16];
	u8 auth_tag[16];
	int err;

	err = gcmaes_crypt_by_sg(false, req, assoclen, hash_subkey, iv, aes_ctx,
				 auth_tag, auth_tag_len);
	if (err)
		return err;

	/* Copy out original auth_tag */
	scatterwalk_map_and_copy(auth_tag_msg, req->src,
				 req->assoclen + req->cryptlen - auth_tag_len,
				 auth_tag_len, 0);

	/* Compare generated tag with passed in tag. */
	if (crypto_memneq(auth_tag_msg, auth_tag, auth_tag_len)) {
		memzero_explicit(auth_tag, sizeof(auth_tag));
		return -EBADMSG;
	}
	return 0;
}

static int helper_rfc4106_encrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct aesni_rfc4106_gcm_ctx *ctx = aesni_rfc4106_gcm_ctx_get(tfm);
	void *aes_ctx = &(ctx->aes_key_expanded);
	u8 ivbuf[16 + (AESNI_ALIGN - 8)] __aligned(8);
	u8 *iv = PTR_ALIGN(&ivbuf[0], AESNI_ALIGN);
	unsigned int i;
	__be32 counter = cpu_to_be32(1);

	/* Assuming we are supporting rfc4106 64-bit extended */
	/* sequence numbers We need to have the AAD length equal */
	/* to 16 or 20 bytes */
	if (unlikely(req->assoclen != 16 && req->assoclen != 20))
		return -EINVAL;

	/* IV below built */
	for (i = 0; i < 4; i++)
		*(iv+i) = ctx->nonce[i];
	for (i = 0; i < 8; i++)
		*(iv+4+i) = req->iv[i];
	*((__be32 *)(iv+12)) = counter;

	return gcmaes_encrypt(req, req->assoclen - 8, ctx->hash_subkey, iv,
			      aes_ctx);
}

static int helper_rfc4106_decrypt(struct aead_request *req)
{
	__be32 counter = cpu_to_be32(1);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct aesni_rfc4106_gcm_ctx *ctx = aesni_rfc4106_gcm_ctx_get(tfm);
	void *aes_ctx = &(ctx->aes_key_expanded);
	u8 ivbuf[16 + (AESNI_ALIGN - 8)] __aligned(8);
	u8 *iv = PTR_ALIGN(&ivbuf[0], AESNI_ALIGN);
	unsigned int i;

	if (unlikely(req->assoclen != 16 && req->assoclen != 20))
		return -EINVAL;

	/* Assuming we are supporting rfc4106 64-bit extended */
	/* sequence numbers We need to have the AAD length */
	/* equal to 16 or 20 bytes */

	/* IV below built */
	for (i = 0; i < 4; i++)
		*(iv+i) = ctx->nonce[i];
	for (i = 0; i < 8; i++)
		*(iv+4+i) = req->iv[i];
	*((__be32 *)(iv+12)) = counter;

	return gcmaes_decrypt(req, req->assoclen - 8, ctx->hash_subkey, iv,
			      aes_ctx);
}
#endif

static int xts_setkey_aesni(struct crypto_skcipher *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct aesni_xts_ctx *ctx = aes_xts_ctx(tfm);
	int err;

	err = xts_verify_key(tfm, key, keylen);
	if (err)
		return err;

	keylen /= 2;

	/* first half of xts-key is for crypt */
	err = aes_set_key_common(&ctx->crypt_ctx, key, keylen);
	if (err)
		return err;

	/* second half of xts-key is for tweak */
	return aes_set_key_common(&ctx->tweak_ctx, key + keylen, keylen);
}

typedef void (*xts_encrypt_iv_func)(const struct crypto_aes_ctx *tweak_key,
				    u8 iv[AES_BLOCK_SIZE]);
typedef void (*xts_crypt_func)(const struct crypto_aes_ctx *key,
			       const u8 *src, u8 *dst, unsigned int len,
			       u8 tweak[AES_BLOCK_SIZE]);

/* This handles cases where the source and/or destination span pages. */
static noinline int
xts_crypt_slowpath(struct skcipher_request *req, xts_crypt_func crypt_func)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	const struct aesni_xts_ctx *ctx = aes_xts_ctx(tfm);
	int tail = req->cryptlen % AES_BLOCK_SIZE;
	struct scatterlist sg_src[2], sg_dst[2];
	struct skcipher_request subreq;
	struct skcipher_walk walk;
	struct scatterlist *src, *dst;
	int err;

	/*
	 * If the message length isn't divisible by the AES block size, then
	 * separate off the last full block and the partial block.  This ensures
	 * that they are processed in the same call to the assembly function,
	 * which is required for ciphertext stealing.
	 */
	if (tail) {
		skcipher_request_set_tfm(&subreq, tfm);
		skcipher_request_set_callback(&subreq,
					      skcipher_request_flags(req),
					      NULL, NULL);
		skcipher_request_set_crypt(&subreq, req->src, req->dst,
					   req->cryptlen - tail - AES_BLOCK_SIZE,
					   req->iv);
		req = &subreq;
	}

	err = skcipher_walk_virt(&walk, req, false);

	while (walk.nbytes) {
		kernel_fpu_begin();
		(*crypt_func)(&ctx->crypt_ctx,
			      walk.src.virt.addr, walk.dst.virt.addr,
			      walk.nbytes & ~(AES_BLOCK_SIZE - 1), req->iv);
		kernel_fpu_end();
		err = skcipher_walk_done(&walk,
					 walk.nbytes & (AES_BLOCK_SIZE - 1));
	}

	if (err || !tail)
		return err;

	/* Do ciphertext stealing with the last full block and partial block. */

	dst = src = scatterwalk_ffwd(sg_src, req->src, req->cryptlen);
	if (req->dst != req->src)
		dst = scatterwalk_ffwd(sg_dst, req->dst, req->cryptlen);

	skcipher_request_set_crypt(req, src, dst, AES_BLOCK_SIZE + tail,
				   req->iv);

	err = skcipher_walk_virt(&walk, req, false);
	if (err)
		return err;

	kernel_fpu_begin();
	(*crypt_func)(&ctx->crypt_ctx, walk.src.virt.addr, walk.dst.virt.addr,
		      walk.nbytes, req->iv);
	kernel_fpu_end();

	return skcipher_walk_done(&walk, 0);
}

/* __always_inline to avoid indirect call in fastpath */
static __always_inline int
xts_crypt(struct skcipher_request *req, xts_encrypt_iv_func encrypt_iv,
	  xts_crypt_func crypt_func)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	const struct aesni_xts_ctx *ctx = aes_xts_ctx(tfm);
	const unsigned int cryptlen = req->cryptlen;
	struct scatterlist *src = req->src;
	struct scatterlist *dst = req->dst;

	if (unlikely(cryptlen < AES_BLOCK_SIZE))
		return -EINVAL;

	kernel_fpu_begin();
	(*encrypt_iv)(&ctx->tweak_ctx, req->iv);

	/*
	 * In practice, virtually all XTS plaintexts and ciphertexts are either
	 * 512 or 4096 bytes, aligned such that they don't span page boundaries.
	 * To optimize the performance of these cases, and also any other case
	 * where no page boundary is spanned, the below fast-path handles
	 * single-page sources and destinations as efficiently as possible.
	 */
	if (likely(src->length >= cryptlen && dst->length >= cryptlen &&
		   src->offset + cryptlen <= PAGE_SIZE &&
		   dst->offset + cryptlen <= PAGE_SIZE)) {
		struct page *src_page = sg_page(src);
		struct page *dst_page = sg_page(dst);
		void *src_virt = kmap_local_page(src_page) + src->offset;
		void *dst_virt = kmap_local_page(dst_page) + dst->offset;

		(*crypt_func)(&ctx->crypt_ctx, src_virt, dst_virt, cryptlen,
			      req->iv);
		kunmap_local(dst_virt);
		kunmap_local(src_virt);
		kernel_fpu_end();
		return 0;
	}
	kernel_fpu_end();
	return xts_crypt_slowpath(req, crypt_func);
}

static void aesni_xts_encrypt_iv(const struct crypto_aes_ctx *tweak_key,
				 u8 iv[AES_BLOCK_SIZE])
{
	aesni_enc(tweak_key, iv, iv);
}

static void aesni_xts_encrypt(const struct crypto_aes_ctx *key,
			      const u8 *src, u8 *dst, unsigned int len,
			      u8 tweak[AES_BLOCK_SIZE])
{
	aesni_xts_enc(key, dst, src, len, tweak);
}

static void aesni_xts_decrypt(const struct crypto_aes_ctx *key,
			      const u8 *src, u8 *dst, unsigned int len,
			      u8 tweak[AES_BLOCK_SIZE])
{
	aesni_xts_dec(key, dst, src, len, tweak);
}

static int xts_encrypt_aesni(struct skcipher_request *req)
{
	return xts_crypt(req, aesni_xts_encrypt_iv, aesni_xts_encrypt);
}

static int xts_decrypt_aesni(struct skcipher_request *req)
{
	return xts_crypt(req, aesni_xts_encrypt_iv, aesni_xts_decrypt);
}

static struct crypto_alg aesni_cipher_alg = {
	.cra_name		= "aes",
	.cra_driver_name	= "aes-aesni",
	.cra_priority		= 300,
	.cra_flags		= CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		= AES_BLOCK_SIZE,
	.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
	.cra_module		= THIS_MODULE,
	.cra_u	= {
		.cipher	= {
			.cia_min_keysize	= AES_MIN_KEY_SIZE,
			.cia_max_keysize	= AES_MAX_KEY_SIZE,
			.cia_setkey		= aes_set_key,
			.cia_encrypt		= aesni_encrypt,
			.cia_decrypt		= aesni_decrypt
		}
	}
};

static struct skcipher_alg aesni_skciphers[] = {
	{
		.base = {
			.cra_name		= "__ecb(aes)",
			.cra_driver_name	= "__ecb-aes-aesni",
			.cra_priority		= 400,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= aesni_skcipher_setkey,
		.encrypt	= ecb_encrypt,
		.decrypt	= ecb_decrypt,
	}, {
		.base = {
			.cra_name		= "__cbc(aes)",
			.cra_driver_name	= "__cbc-aes-aesni",
			.cra_priority		= 400,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= aesni_skcipher_setkey,
		.encrypt	= cbc_encrypt,
		.decrypt	= cbc_decrypt,
	}, {
		.base = {
			.cra_name		= "__cts(cbc(aes))",
			.cra_driver_name	= "__cts-cbc-aes-aesni",
			.cra_priority		= 400,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.walksize	= 2 * AES_BLOCK_SIZE,
		.setkey		= aesni_skcipher_setkey,
		.encrypt	= cts_cbc_encrypt,
		.decrypt	= cts_cbc_decrypt,
#ifdef CONFIG_X86_64
	}, {
		.base = {
			.cra_name		= "__ctr(aes)",
			.cra_driver_name	= "__ctr-aes-aesni",
			.cra_priority		= 400,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= 1,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.chunksize	= AES_BLOCK_SIZE,
		.setkey		= aesni_skcipher_setkey,
		.encrypt	= ctr_crypt,
		.decrypt	= ctr_crypt,
#endif
	}, {
		.base = {
			.cra_name		= "__xts(aes)",
			.cra_driver_name	= "__xts-aes-aesni",
			.cra_priority		= 401,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= XTS_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= 2 * AES_MIN_KEY_SIZE,
		.max_keysize	= 2 * AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.walksize	= 2 * AES_BLOCK_SIZE,
		.setkey		= xts_setkey_aesni,
		.encrypt	= xts_encrypt_aesni,
		.decrypt	= xts_decrypt_aesni,
	}
};

static
struct simd_skcipher_alg *aesni_simd_skciphers[ARRAY_SIZE(aesni_skciphers)];

#ifdef CONFIG_X86_64
/*
 * XCTR does not have a non-AVX implementation, so it must be enabled
 * conditionally.
 */
static struct skcipher_alg aesni_xctr = {
	.base = {
		.cra_name		= "__xctr(aes)",
		.cra_driver_name	= "__xctr-aes-aesni",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_INTERNAL,
		.cra_blocksize		= 1,
		.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
		.cra_module		= THIS_MODULE,
	},
	.min_keysize	= AES_MIN_KEY_SIZE,
	.max_keysize	= AES_MAX_KEY_SIZE,
	.ivsize		= AES_BLOCK_SIZE,
	.chunksize	= AES_BLOCK_SIZE,
	.setkey		= aesni_skcipher_setkey,
	.encrypt	= xctr_crypt,
	.decrypt	= xctr_crypt,
};

static struct simd_skcipher_alg *aesni_simd_xctr;

asmlinkage void aes_xts_encrypt_iv(const struct crypto_aes_ctx *tweak_key,
				   u8 iv[AES_BLOCK_SIZE]);

#define DEFINE_XTS_ALG(suffix, driver_name, priority)			       \
									       \
asmlinkage void								       \
aes_xts_encrypt_##suffix(const struct crypto_aes_ctx *key, const u8 *src,      \
			 u8 *dst, unsigned int len, u8 tweak[AES_BLOCK_SIZE]); \
asmlinkage void								       \
aes_xts_decrypt_##suffix(const struct crypto_aes_ctx *key, const u8 *src,      \
			 u8 *dst, unsigned int len, u8 tweak[AES_BLOCK_SIZE]); \
									       \
static int xts_encrypt_##suffix(struct skcipher_request *req)		       \
{									       \
	return xts_crypt(req, aes_xts_encrypt_iv, aes_xts_encrypt_##suffix);   \
}									       \
									       \
static int xts_decrypt_##suffix(struct skcipher_request *req)		       \
{									       \
	return xts_crypt(req, aes_xts_encrypt_iv, aes_xts_decrypt_##suffix);   \
}									       \
									       \
static struct skcipher_alg aes_xts_alg_##suffix = {			       \
	.base = {							       \
		.cra_name		= "__xts(aes)",			       \
		.cra_driver_name	= "__" driver_name,		       \
		.cra_priority		= priority,			       \
		.cra_flags		= CRYPTO_ALG_INTERNAL,		       \
		.cra_blocksize		= AES_BLOCK_SIZE,		       \
		.cra_ctxsize		= XTS_AES_CTX_SIZE,		       \
		.cra_module		= THIS_MODULE,			       \
	},								       \
	.min_keysize	= 2 * AES_MIN_KEY_SIZE,				       \
	.max_keysize	= 2 * AES_MAX_KEY_SIZE,				       \
	.ivsize		= AES_BLOCK_SIZE,				       \
	.walksize	= 2 * AES_BLOCK_SIZE,				       \
	.setkey		= xts_setkey_aesni,				       \
	.encrypt	= xts_encrypt_##suffix,				       \
	.decrypt	= xts_decrypt_##suffix,				       \
};									       \
									       \
static struct simd_skcipher_alg *aes_xts_simdalg_##suffix

DEFINE_XTS_ALG(aesni_avx, "xts-aes-aesni-avx", 500);
#if defined(CONFIG_AS_VAES) && defined(CONFIG_AS_VPCLMULQDQ)
DEFINE_XTS_ALG(vaes_avx2, "xts-aes-vaes-avx2", 600);
DEFINE_XTS_ALG(vaes_avx10_256, "xts-aes-vaes-avx10_256", 700);
DEFINE_XTS_ALG(vaes_avx10_512, "xts-aes-vaes-avx10_512", 800);
#endif

/*
 * This is a list of CPU models that are known to suffer from downclocking when
 * zmm registers (512-bit vectors) are used.  On these CPUs, the AES-XTS
 * implementation with zmm registers won't be used by default.  An
 * implementation with ymm registers (256-bit vectors) will be used instead.
 */
static const struct x86_cpu_id zmm_exclusion_list[] = {
	X86_MATCH_VFM(INTEL_SKYLAKE_X,		0),
	X86_MATCH_VFM(INTEL_ICELAKE_X,		0),
	X86_MATCH_VFM(INTEL_ICELAKE_D,		0),
	X86_MATCH_VFM(INTEL_ICELAKE,		0),
	X86_MATCH_VFM(INTEL_ICELAKE_L,		0),
	X86_MATCH_VFM(INTEL_ICELAKE_NNPI,	0),
	X86_MATCH_VFM(INTEL_TIGERLAKE_L,	0),
	X86_MATCH_VFM(INTEL_TIGERLAKE,		0),
	/* Allow Rocket Lake and later, and Sapphire Rapids and later. */
	/* Also allow AMD CPUs (starting with Zen 4, the first with AVX-512). */
	{},
};

static int __init register_xts_algs(void)
{
	int err;

	if (!boot_cpu_has(X86_FEATURE_AVX))
		return 0;
	err = simd_register_skciphers_compat(&aes_xts_alg_aesni_avx, 1,
					     &aes_xts_simdalg_aesni_avx);
	if (err)
		return err;
#if defined(CONFIG_AS_VAES) && defined(CONFIG_AS_VPCLMULQDQ)
	if (!boot_cpu_has(X86_FEATURE_AVX2) ||
	    !boot_cpu_has(X86_FEATURE_VAES) ||
	    !boot_cpu_has(X86_FEATURE_VPCLMULQDQ) ||
	    !boot_cpu_has(X86_FEATURE_PCLMULQDQ) ||
	    !cpu_has_xfeatures(XFEATURE_MASK_SSE | XFEATURE_MASK_YMM, NULL))
		return 0;
	err = simd_register_skciphers_compat(&aes_xts_alg_vaes_avx2, 1,
					     &aes_xts_simdalg_vaes_avx2);
	if (err)
		return err;

	if (!boot_cpu_has(X86_FEATURE_AVX512BW) ||
	    !boot_cpu_has(X86_FEATURE_AVX512VL) ||
	    !boot_cpu_has(X86_FEATURE_BMI2) ||
	    !cpu_has_xfeatures(XFEATURE_MASK_SSE | XFEATURE_MASK_YMM |
			       XFEATURE_MASK_AVX512, NULL))
		return 0;

	err = simd_register_skciphers_compat(&aes_xts_alg_vaes_avx10_256, 1,
					     &aes_xts_simdalg_vaes_avx10_256);
	if (err)
		return err;

	if (x86_match_cpu(zmm_exclusion_list))
		aes_xts_alg_vaes_avx10_512.base.cra_priority = 1;

	err = simd_register_skciphers_compat(&aes_xts_alg_vaes_avx10_512, 1,
					     &aes_xts_simdalg_vaes_avx10_512);
	if (err)
		return err;
#endif /* CONFIG_AS_VAES && CONFIG_AS_VPCLMULQDQ */
	return 0;
}

static void unregister_xts_algs(void)
{
	if (aes_xts_simdalg_aesni_avx)
		simd_unregister_skciphers(&aes_xts_alg_aesni_avx, 1,
					  &aes_xts_simdalg_aesni_avx);
#if defined(CONFIG_AS_VAES) && defined(CONFIG_AS_VPCLMULQDQ)
	if (aes_xts_simdalg_vaes_avx2)
		simd_unregister_skciphers(&aes_xts_alg_vaes_avx2, 1,
					  &aes_xts_simdalg_vaes_avx2);
	if (aes_xts_simdalg_vaes_avx10_256)
		simd_unregister_skciphers(&aes_xts_alg_vaes_avx10_256, 1,
					  &aes_xts_simdalg_vaes_avx10_256);
	if (aes_xts_simdalg_vaes_avx10_512)
		simd_unregister_skciphers(&aes_xts_alg_vaes_avx10_512, 1,
					  &aes_xts_simdalg_vaes_avx10_512);
#endif
}
#else /* CONFIG_X86_64 */
static int __init register_xts_algs(void)
{
	return 0;
}

static void unregister_xts_algs(void)
{
}
#endif /* !CONFIG_X86_64 */

#ifdef CONFIG_X86_64
static int generic_gcmaes_set_key(struct crypto_aead *aead, const u8 *key,
				  unsigned int key_len)
{
	struct generic_gcmaes_ctx *ctx = generic_gcmaes_ctx_get(aead);

	return aes_set_key_common(&ctx->aes_key_expanded, key, key_len) ?:
	       aes_gcm_derive_hash_subkey(&ctx->aes_key_expanded,
					  ctx->hash_subkey);
}

static int generic_gcmaes_encrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct generic_gcmaes_ctx *ctx = generic_gcmaes_ctx_get(tfm);
	void *aes_ctx = &(ctx->aes_key_expanded);
	u8 ivbuf[16 + (AESNI_ALIGN - 8)] __aligned(8);
	u8 *iv = PTR_ALIGN(&ivbuf[0], AESNI_ALIGN);
	__be32 counter = cpu_to_be32(1);

	memcpy(iv, req->iv, 12);
	*((__be32 *)(iv+12)) = counter;

	return gcmaes_encrypt(req, req->assoclen, ctx->hash_subkey, iv,
			      aes_ctx);
}

static int generic_gcmaes_decrypt(struct aead_request *req)
{
	__be32 counter = cpu_to_be32(1);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct generic_gcmaes_ctx *ctx = generic_gcmaes_ctx_get(tfm);
	void *aes_ctx = &(ctx->aes_key_expanded);
	u8 ivbuf[16 + (AESNI_ALIGN - 8)] __aligned(8);
	u8 *iv = PTR_ALIGN(&ivbuf[0], AESNI_ALIGN);

	memcpy(iv, req->iv, 12);
	*((__be32 *)(iv+12)) = counter;

	return gcmaes_decrypt(req, req->assoclen, ctx->hash_subkey, iv,
			      aes_ctx);
}

static struct aead_alg aesni_aeads[] = { {
	.setkey			= common_rfc4106_set_key,
	.setauthsize		= common_rfc4106_set_authsize,
	.encrypt		= helper_rfc4106_encrypt,
	.decrypt		= helper_rfc4106_decrypt,
	.ivsize			= GCM_RFC4106_IV_SIZE,
	.maxauthsize		= 16,
	.base = {
		.cra_name		= "__rfc4106(gcm(aes))",
		.cra_driver_name	= "__rfc4106-gcm-aesni",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_INTERNAL,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct aesni_rfc4106_gcm_ctx),
		.cra_alignmask		= 0,
		.cra_module		= THIS_MODULE,
	},
}, {
	.setkey			= generic_gcmaes_set_key,
	.setauthsize		= generic_gcmaes_set_authsize,
	.encrypt		= generic_gcmaes_encrypt,
	.decrypt		= generic_gcmaes_decrypt,
	.ivsize			= GCM_AES_IV_SIZE,
	.maxauthsize		= 16,
	.base = {
		.cra_name		= "__gcm(aes)",
		.cra_driver_name	= "__generic-gcm-aesni",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_INTERNAL,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct generic_gcmaes_ctx),
		.cra_alignmask		= 0,
		.cra_module		= THIS_MODULE,
	},
} };

#else
static struct aead_alg aesni_aeads[0];
#endif

static struct aead_alg aesni_ccm_aead = {
	.base.cra_name		= "ccm(aes)",
	.base.cra_driver_name	= "ccm-aesni",
	.base.cra_priority	= 400,
	.base.cra_blocksize	= 1,
	.base.cra_ctxsize	= CRYPTO_AES_CTX_SIZE,
	.base.cra_module	= THIS_MODULE,

	.setkey			= aesni_ccm_setkey,
	.setauthsize		= aesni_ccm_setauthsize,
	.encrypt		= aesni_ccm_encrypt,
	.decrypt		= aesni_ccm_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.chunksize		= AES_BLOCK_SIZE,
	.maxauthsize		= AES_BLOCK_SIZE,
};

static struct simd_aead_alg *aesni_simd_aeads[ARRAY_SIZE(aesni_aeads)];

static const struct x86_cpu_id aesni_cpu_id[] = {
	X86_MATCH_FEATURE(X86_FEATURE_AES, NULL),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, aesni_cpu_id);

static int __init aesni_init(void)
{
	int err;

	if (!x86_match_cpu(aesni_cpu_id))
		return -ENODEV;
#ifdef CONFIG_X86_64
	if (boot_cpu_has(X86_FEATURE_AVX2)) {
		pr_info("AVX2 version of gcm_enc/dec engaged.\n");
		static_branch_enable(&gcm_use_avx);
		static_branch_enable(&gcm_use_avx2);
	} else
	if (boot_cpu_has(X86_FEATURE_AVX)) {
		pr_info("AVX version of gcm_enc/dec engaged.\n");
		static_branch_enable(&gcm_use_avx);
	} else {
		pr_info("SSE version of gcm_enc/dec engaged.\n");
	}
	if (boot_cpu_has(X86_FEATURE_AVX)) {
		/* optimize performance of ctr mode encryption transform */
		static_call_update(aesni_ctr_enc_tfm, aesni_ctr_enc_avx_tfm);
		pr_info("AES CTR mode by8 optimization enabled\n");
	}
#endif /* CONFIG_X86_64 */

	err = crypto_register_alg(&aesni_cipher_alg);
	if (err)
		return err;

	err = simd_register_skciphers_compat(aesni_skciphers,
					     ARRAY_SIZE(aesni_skciphers),
					     aesni_simd_skciphers);
	if (err)
		goto unregister_cipher;

	err = simd_register_aeads_compat(aesni_aeads, ARRAY_SIZE(aesni_aeads),
					 aesni_simd_aeads);
	if (err)
		goto unregister_skciphers;

#ifdef CONFIG_X86_64
	if (boot_cpu_has(X86_FEATURE_AVX))
		err = simd_register_skciphers_compat(&aesni_xctr, 1,
						     &aesni_simd_xctr);
	if (err)
		goto unregister_aeads;
#endif /* CONFIG_X86_64 */

	err = register_xts_algs();
	if (err)
		goto unregister_xts;

	if (IS_ENABLED(CONFIG_X86_64)) {
		err = crypto_register_aead(&aesni_ccm_aead);
		if (err)
			goto unregister_xts;
	}

	return 0;

unregister_xts:
	unregister_xts_algs();
#ifdef CONFIG_X86_64
	if (aesni_simd_xctr)
		simd_unregister_skciphers(&aesni_xctr, 1, &aesni_simd_xctr);
unregister_aeads:
#endif /* CONFIG_X86_64 */
	simd_unregister_aeads(aesni_aeads, ARRAY_SIZE(aesni_aeads),
				aesni_simd_aeads);

unregister_skciphers:
	simd_unregister_skciphers(aesni_skciphers, ARRAY_SIZE(aesni_skciphers),
				  aesni_simd_skciphers);
unregister_cipher:
	crypto_unregister_alg(&aesni_cipher_alg);
	return err;
}

static void __exit aesni_exit(void)
{
	if (IS_ENABLED(CONFIG_X86_64))
		crypto_unregister_aead(&aesni_ccm_aead);

	simd_unregister_aeads(aesni_aeads, ARRAY_SIZE(aesni_aeads),
			      aesni_simd_aeads);
	simd_unregister_skciphers(aesni_skciphers, ARRAY_SIZE(aesni_skciphers),
				  aesni_simd_skciphers);
	crypto_unregister_alg(&aesni_cipher_alg);
#ifdef CONFIG_X86_64
	if (boot_cpu_has(X86_FEATURE_AVX))
		simd_unregister_skciphers(&aesni_xctr, 1, &aesni_simd_xctr);
#endif /* CONFIG_X86_64 */
	unregister_xts_algs();
}

late_initcall(aesni_init);
module_exit(aesni_exit);

MODULE_DESCRIPTION("Rijndael (AES) Cipher Algorithm, Intel AES-NI instructions optimized");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CRYPTO("aes");
