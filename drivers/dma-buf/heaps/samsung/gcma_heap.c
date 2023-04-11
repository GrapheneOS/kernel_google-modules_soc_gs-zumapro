// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF GCMA heap
 *
 */

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/pfn.h>
#include <soc/google/gcma.h>

#include "samsung-dma-heap.h"

struct gcma_heap {
	struct gen_pool *pool;
};

static struct dma_buf *gcma_heap_allocate(struct dma_heap *heap, unsigned long len,
					      unsigned long fd_flags, unsigned long heap_flags)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct gcma_heap *gcma_heap = samsung_dma_heap->priv;
	struct samsung_dma_buffer *buffer;
	struct dma_buf *dmabuf;
	struct page *pages;
	unsigned int alignment = samsung_dma_heap->alignment;
	unsigned long size;
	phys_addr_t paddr;
	unsigned long pfn;
	int protret = 0, ret = -ENOMEM;

	if (dma_heap_flags_video_aligned(samsung_dma_heap->flags))
		len = dma_heap_add_video_padding(len);

	size = ALIGN(len, alignment);

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, size, 1);
	if (IS_ERR(buffer))
		return ERR_PTR(-ENOMEM);

	paddr = gen_pool_alloc(gcma_heap->pool, size);
	if (!paddr) {
		perrfn("failed to allocate from GCMA, size %lu", size);
		goto free_gen;
	}

	pfn = PFN_DOWN(paddr);
	gcma_alloc_range(pfn, pfn + (size >> PAGE_SHIFT) - 1);

	pages = phys_to_page(paddr);
	sg_set_page(buffer->sg_table.sgl, pages, size, 0);

	heap_page_clean(pages, size);
	heap_cache_flush(buffer);

	if (dma_heap_flags_protected(samsung_dma_heap->flags)) {
		buffer->priv = samsung_dma_buffer_protect(buffer, size, 1, paddr);
		if (IS_ERR(buffer->priv)) {
			ret = PTR_ERR(buffer->priv);
			goto free_prot;
		}
	}

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_export;
	}

	return dmabuf;

free_export:
	protret = samsung_dma_buffer_unprotect(buffer);
free_prot:
	gcma_free_range(pfn, pfn + (size >> PAGE_SHIFT) - 1);
	if (!protret)
		gen_pool_free(gcma_heap->pool, paddr, size);
free_gen:
	samsung_dma_buffer_free(buffer);

	return ERR_PTR(ret);
}

static void gcma_heap_release(struct samsung_dma_buffer *buffer)
{
	struct samsung_dma_heap *samsung_dma_heap = buffer->heap;
	struct gcma_heap *gcma_heap = samsung_dma_heap->priv;
	int ret = 0;
	unsigned long pfn;

	if (dma_heap_flags_protected(samsung_dma_heap->flags))
		ret = samsung_dma_buffer_unprotect(buffer);

	pfn = PFN_DOWN(sg_phys(buffer->sg_table.sgl));
	gcma_free_range(pfn, pfn + (buffer->len >> PAGE_SHIFT) - 1);

	if (!ret)
		gen_pool_free(gcma_heap->pool, sg_phys(buffer->sg_table.sgl), buffer->len);

	samsung_dma_buffer_free(buffer);
}

static const struct dma_heap_ops gcma_heap_ops = {
	.allocate = gcma_heap_allocate,
};

static int gcma_heap_probe(struct platform_device *pdev)
{
	struct gcma_heap *gcma_heap;
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	int ret;

	rmem_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!rmem_np)
		return -ENODEV;

	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		perrdev(&pdev->dev, "memory-region handle not found");
		return -ENODEV;
	}

	gcma_heap = devm_kzalloc(&pdev->dev, sizeof(*gcma_heap), GFP_KERNEL);
	if (!gcma_heap)
		return -ENOMEM;

	ret = register_gcma_area(rmem->name, rmem->base, rmem->size);
	if (ret)
		return ret;

	gcma_heap->pool = devm_gen_pool_create(&pdev->dev, PAGE_SHIFT, -1, 0);
	if (!gcma_heap->pool)
		return -ENOMEM;

	ret = gen_pool_add(gcma_heap->pool, rmem->base, rmem->size, -1);
	if (ret)
		return ret;

	ret = samsung_heap_add(&pdev->dev, gcma_heap, gcma_heap_release,
			       &gcma_heap_ops);
	if (ret == -ENODEV)
		return 0;

	return ret;
}

static const struct of_device_id gcma_heap_of_match[] = {
	{ .compatible = "google,dma-heap-gcma", },
	{ },
};
MODULE_DEVICE_TABLE(of, gcma_heap_of_match);

static struct platform_driver gcma_heap_driver = {
	.driver		= {
		.name	= "google,dma-heap-gcma",
		.of_match_table = gcma_heap_of_match,
	},
	.probe		= gcma_heap_probe,
};

int __init gcma_dma_heap_init(void)
{
	return platform_driver_register(&gcma_heap_driver);
}

void gcma_dma_heap_exit(void)
{
	platform_driver_unregister(&gcma_heap_driver);
}
