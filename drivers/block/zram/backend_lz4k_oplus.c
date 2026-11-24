// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/kernel.h>
#include <linux/slab.h>

#include "backend_lz4k_oplus.h"
#include "../../../../lib/lz4k_oplus/lz4k.h"

static int lz4k_oplus_create(struct zcomp_params *params, struct zcomp_ctx *ctx)
{
	ctx->context = kvmalloc(2 * PAGE_SIZE, GFP_KERNEL);
	return ctx->context ? 0 : -ENOMEM;
}

static void lz4k_oplus_destroy(struct zcomp_ctx *ctx)
{
	kvfree(ctx->context);
}

static int lz4k_oplus_compress(struct zcomp_params *params,
			       struct zcomp_ctx *ctx, struct zcomp_req *req)
{
	int ret = lz4k_compress(ctx->context, req->src, req->dst,
				  req->src_len, req->dst_len);

	if (ret <= 0)
		return -EINVAL;

	req->dst_len = ret;
	return 0;
}

static int lz4k_oplus_decompress(struct zcomp_params *params,
				 struct zcomp_ctx *ctx, struct zcomp_req *req)
{
	int ret = lz4k_decompress(req->src, req->dst, req->src_len, req->dst_len);

	return ret == req->dst_len ? 0 : -EINVAL;
}

static int lz4k_oplus_setup_params(struct zcomp_params *params)
{
	return 0;
}

static void lz4k_oplus_release_params(struct zcomp_params *params)
{
}

const struct zcomp_ops backend_lz4k_oplus = {
	.compress = lz4k_oplus_compress,
	.decompress = lz4k_oplus_decompress,
	.create_ctx = lz4k_oplus_create,
	.destroy_ctx = lz4k_oplus_destroy,
	.setup_params = lz4k_oplus_setup_params,
	.release_params = lz4k_oplus_release_params,
	.name = "lz4k_oplus",
};
