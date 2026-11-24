#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "backend_lz4hc.h"
#include "lz4/lz4_deps.h"
#include "lz4/upstream/lz4.c"
#include "lz4/upstream/lz4hc.c"

struct lz4hc_ctx {
	LZ4_streamHC_t *cstrm;
};

static void lz4hc_release_params(struct zcomp_params *params)
{
}

static int lz4hc_setup_params(struct zcomp_params *params)
{
	if (params->level == ZCOMP_PARAM_NOT_SET)
		params->level = LZ4HC_CLEVEL_DEFAULT;
	else if (params->level < 1 || params->level > 16)
		return -EINVAL;
	params->level = min(params->level, LZ4HC_CLEVEL_MAX);

	if (params->dict_sz && (!params->dict || params->dict_sz > 65536))
		return -EINVAL;
	return 0;
}

static void lz4hc_destroy(struct zcomp_ctx *ctx)
{
	struct lz4hc_ctx *zctx = ctx->context;

	if (!zctx)
		return;

	vfree(zctx->cstrm);
	kfree(zctx);
	ctx->context = NULL;
}

static int lz4hc_create(struct zcomp_params *params, struct zcomp_ctx *ctx)
{
	struct lz4hc_ctx *zctx;
	size_t state_size = params->level >= LZ4HC_CLEVEL_OPT_MIN ?
		LZ4_STREAMHC_EXTSIZE : sizeof(LZ4_streamHC_t);

	zctx = kzalloc(sizeof(*zctx), GFP_KERNEL);
	if (!zctx)
		return -ENOMEM;

	ctx->context = zctx;
	zctx->cstrm = vmalloc(state_size);
	if (!zctx->cstrm)
		goto error;
	if (!LZ4_initStreamHC(zctx->cstrm, state_size))
		goto error;

	return 0;
error:
	lz4hc_destroy(ctx);
	return -ENOMEM;
}

static int lz4hc_compress(struct zcomp_params *params, struct zcomp_ctx *ctx,
			  struct zcomp_req *req)
{
	struct lz4hc_ctx *zctx = ctx->context;
	int ret;

	LZ4_resetStreamHC_fast(zctx->cstrm, params->level);
	if (params->dict_sz) {
		ret = LZ4_loadDictHC(zctx->cstrm, params->dict, params->dict_sz);
		if (ret != params->dict_sz)
			return -EINVAL;
	}
	ret = LZ4_compress_HC_continue(zctx->cstrm, req->src, req->dst,
				       req->src_len, req->dst_len);
	if (!ret)
		return -EINVAL;
	req->dst_len = ret;
	return 0;
}

static int lz4hc_decompress(struct zcomp_params *params, struct zcomp_ctx *ctx,
			    struct zcomp_req *req)
{
	int ret;

	ret = LZ4_decompress_safe_usingDict(req->src, req->dst, req->src_len,
					   req->dst_len, params->dict, params->dict_sz);
	if (ret < 0 || ret != req->dst_len)
		return -EINVAL;
	return 0;
}

const struct zcomp_ops backend_lz4hc = {
	.compress	= lz4hc_compress,
	.decompress	= lz4hc_decompress,
	.create_ctx	= lz4hc_create,
	.destroy_ctx	= lz4hc_destroy,
	.setup_params	= lz4hc_setup_params,
	.release_params	= lz4hc_release_params,
	.name		= "lz4hc",
};
