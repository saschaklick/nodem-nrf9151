#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * Minimal, self-contained SHA-1 (RFC 3174) providing exactly the one
 * mbedtls symbol (`mbedtls_sha1`) Zephyr's websocket client
 * (subsys/net/lib/websocket/websocket.c) calls directly to compute
 * Sec-WebSocket-Accept during the RFC 6455 handshake - a protocol
 * integrity check, not a security-sensitive operation (the actual security
 * is the underlying TLS connection), so a standalone implementation here is
 * appropriate.
 *
 * This exists because nrf_security's TF-M/PSA-based crypto build (used
 * throughout this project - see prj.conf) doesn't expose that legacy
 * one-shot mbedtls API on the non-secure side at all: preprocessing
 * modules/crypto/mbedtls/library/sha1.c with this build's actual compiler
 * flags shows MBEDTLS_SHA1_C never ends up defined, regardless of which of
 * nrf_security's legacy-crypto Kconfig options are enabled, because the
 * non-secure side's generated mbedtls config header is generated from
 * TF-M's PSA client interface, not this app's own Kconfig selections.
 */

struct sha1_ctx {
	uint32_t state[5];
	uint64_t count; /* bytes processed so far, excluding padding */
	uint8_t buffer[64];
};

static uint32_t rotl32(uint32_t x, int bits)
{
	return (x << bits) | (x >> (32 - bits));
}

static void sha1_transform(uint32_t state[5], const uint8_t block[64])
{
	uint32_t w[80];

	for (int i = 0; i < 16; i++) {
		w[i] = ((uint32_t)block[i * 4] << 24) | ((uint32_t)block[i * 4 + 1] << 16) |
		       ((uint32_t)block[i * 4 + 2] << 8) | (uint32_t)block[i * 4 + 3];
	}
	for (int i = 16; i < 80; i++) {
		w[i] = rotl32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
	}

	uint32_t a = state[0];
	uint32_t b = state[1];
	uint32_t c = state[2];
	uint32_t d = state[3];
	uint32_t e = state[4];

	for (int i = 0; i < 80; i++) {
		uint32_t f, k;

		if (i < 20) {
			f = (b & c) | (~b & d);
			k = 0x5A827999;
		} else if (i < 40) {
			f = b ^ c ^ d;
			k = 0x6ED9EBA1;
		} else if (i < 60) {
			f = (b & c) | (b & d) | (c & d);
			k = 0x8F1BBCDC;
		} else {
			f = b ^ c ^ d;
			k = 0xCA62C1D6;
		}

		uint32_t t = rotl32(a, 5) + f + e + k + w[i];

		e = d;
		d = c;
		c = rotl32(b, 30);
		b = a;
		a = t;
	}

	state[0] += a;
	state[1] += b;
	state[2] += c;
	state[3] += d;
	state[4] += e;
}

static void sha1_init(struct sha1_ctx *ctx)
{
	ctx->state[0] = 0x67452301;
	ctx->state[1] = 0xEFCDAB89;
	ctx->state[2] = 0x98BADCFE;
	ctx->state[3] = 0x10325476;
	ctx->state[4] = 0xC3D2E1F0;
	ctx->count = 0;
}

static void sha1_update(struct sha1_ctx *ctx, const uint8_t *data, size_t len)
{
	size_t offset = (size_t)(ctx->count % 64);

	ctx->count += len;

	while (len > 0) {
		size_t n = 64 - offset;

		if (n > len) {
			n = len;
		}

		memcpy(&ctx->buffer[offset], data, n);
		offset += n;
		data += n;
		len -= n;

		if (offset == 64) {
			sha1_transform(ctx->state, ctx->buffer);
			offset = 0;
		}
	}
}

static void sha1_final(struct sha1_ctx *ctx, uint8_t output[20])
{
	uint64_t bit_count = ctx->count * 8; /* length of the original message, pre-padding */
	uint8_t pad = 0x80;

	sha1_update(ctx, &pad, 1);

	uint8_t zero = 0;

	while (ctx->count % 64 != 56) {
		sha1_update(ctx, &zero, 1);
	}

	/* Appended directly (not via sha1_update()) - the buffer is already
	 * sitting at offset 56 with nothing left to flush until this last
	 * block, and going through sha1_update() here would just recompute
	 * (and, worse, fold into the digest) a `count` that's meant to
	 * describe the original message, not the padding. */
	for (int i = 0; i < 8; i++) {
		ctx->buffer[56 + i] = (uint8_t)(bit_count >> (56 - i * 8));
	}
	sha1_transform(ctx->state, ctx->buffer);

	for (int i = 0; i < 5; i++) {
		output[i * 4] = (uint8_t)(ctx->state[i] >> 24);
		output[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
		output[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 8);
		output[i * 4 + 3] = (uint8_t)(ctx->state[i]);
	}
}

int mbedtls_sha1(const unsigned char *input, size_t ilen, unsigned char output[20])
{
	struct sha1_ctx ctx;

	sha1_init(&ctx);
	sha1_update(&ctx, input, ilen);
	sha1_final(&ctx, output);

	return 0;
}
