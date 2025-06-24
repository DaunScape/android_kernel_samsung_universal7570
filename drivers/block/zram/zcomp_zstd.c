/*
 * Copyright (C) 2018 Google
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/zstd.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/cpumask.h>
#include <linux/percpu-defs.h>

#include "zcomp_zstd.h"

struct percpu_zstd_dctx {
	void *dwksp;
	ZSTD_DCtx *dctx;
};

struct zstd_ctx {
	void *cwksp;
	ZSTD_CCtx *cctx;
};

static void* percpu_dctx = NULL;

#define ZRAM_ZSTD_LEVEL		1

static ZSTD_parameters zstd_params(void)
{
	return ZSTD_getParams(ZRAM_ZSTD_LEVEL, PAGE_SIZE, 0);
}

static void *alloc_mem(size_t size)
{
	/*
	 * This function can be called in swapout/fs write path
	 * so we can't use GFP_FS|IO. And it assumes we already
	 * have at least one stream in zram initialization so we
	 * don't do best effort to allocate more stream in here.
	 * A default stream will work well without further multiple
	 * streams. That's why we use NORETRY | NOWARN.
	 */
	void *ret = kzalloc(size, GFP_NOIO | __GFP_NORETRY |
			    __GFP_NOWARN);
	if (!ret)
		ret = __vmalloc(size,
				GFP_NOIO | __GFP_NORETRY | __GFP_NOWARN |
				__GFP_ZERO | __GFP_HIGHMEM,
				PAGE_KERNEL);
	return ret;
}

static int zstd_comp_init(struct zstd_ctx *ctx)
{
	int ret = 0;
	const ZSTD_parameters params = zstd_params();
	const size_t wksp_size = ZSTD_CCtxWorkspaceBound(params.cParams);

	ctx->cwksp = alloc_mem(wksp_size);
	if (!ctx->cwksp) {
		ret = -ENOMEM;
		goto out;
	}

	ctx->cctx = ZSTD_initCCtx(ctx->cwksp, wksp_size);
	if (!ctx->cctx) {
		ret = -EINVAL;
		goto out_free;
	}
out:
	return ret;
out_free:
	vfree(ctx->cwksp);
	goto out;
}

static int zstd_decomp_init(struct percpu_zstd_dctx *ctx)
{
	int ret = 0;
	const size_t wksp_size = ZSTD_DCtxWorkspaceBound();

	ctx->dwksp = alloc_mem(wksp_size);
	if (!ctx->dwksp) {
		ret = -ENOMEM;
		goto out;
	}

	ctx->dctx = ZSTD_initDCtx(ctx->dwksp, wksp_size);
	if (!ctx->dctx) {
		ret = -EINVAL;
		goto out_free;
	}
out:
	return ret;
out_free:
	vfree(ctx->dwksp);
	goto out;
}

static int zcomp_zstd_create_percpu(struct zstd_ctx *ctx)
{
	struct percpu_zstd_dctx *dctx;
	int ret = 0;
	int cpu = 0;

	percpu_dctx = alloc_percpu(struct percpu_zstd_dctx);
	if (!percpu_dctx)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		dctx = per_cpu_ptr(percpu_dctx, cpu);
		ret = zstd_decomp_init(dctx);
		if (ret)
			goto failure;
	}

	return 0;

failure:
	for_each_possible_cpu(cpu) {
		dctx = per_cpu_ptr(percpu_dctx, cpu);
		vfree(dctx->dwksp);
	}

	return ret;
}

static void *zcomp_zstd_create(void)
{
	int ret;

	struct zstd_ctx *ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
	if (ctx == NULL)
		return NULL;

	ret = zstd_comp_init(ctx);
	if (ret != 0)
		goto failed;

	ret = zcomp_zstd_create_percpu(ctx);
	if (ret != 0)
		goto failed;

	return ctx;

failed:
	kvfree(ctx->cwksp);

	return NULL;
}

static void zcomp_zstd_destroy(void *private)
{
	struct zstd_ctx *ctx = private;
	if (ctx) {
		kvfree(ctx->cwksp);
	}

	if (percpu_dctx) {
		int cpu = 0;

		for_each_possible_cpu(cpu) {
			struct percpu_zstd_dctx *ctx;
			ctx = per_cpu_ptr(percpu_dctx, cpu);
			vfree(ctx->dwksp);
		}

		free_percpu(percpu_dctx);
		percpu_dctx = NULL;
	}

	kfree(ctx);
}

static int zcomp_zstd_compress(const unsigned char *src, unsigned char *dst,
			       size_t *dst_len, void *private)
{
	size_t len;
	struct zstd_ctx *ctx = private;
	const ZSTD_parameters params = zstd_params();

	len = ZSTD_compressCCtx(ctx->cctx, dst, 2 * PAGE_SIZE, src, PAGE_SIZE, params);
	if (ZSTD_isError(len)) {
		pr_err("[zram-zstd] Compress error %d\n", ZSTD_getErrorCode(len));
		return ZSTD_isError(len);
	}

	*dst_len = len;
	return 0;
}

static int zcomp_zstd_decompress(const unsigned char *src, size_t src_len,
				 unsigned char *dst)
{
	struct percpu_zstd_dctx *ctx;
	size_t ret;

	//get_cpu();
	ctx = this_cpu_ptr(percpu_dctx);
	ret = ZSTD_decompressDCtx(ctx->dctx, dst, PAGE_SIZE, src, src_len);
	//put_cpu();
	if (ZSTD_isError(ret)) {
		pr_err("[zram-zstd] Decompress error %d\n", ZSTD_getErrorCode(ret));
		return -EINVAL;
	}

	return 0;
}

struct zcomp_backend zcomp_zstd = {
	.compress = zcomp_zstd_compress,
	.decompress = zcomp_zstd_decompress,
	.create = zcomp_zstd_create,
	.destroy = zcomp_zstd_destroy,
	.name = "zstd",
};

