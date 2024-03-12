// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/module.h>
#include <linux/seq_file.h>
#include <soc/google/meminfo.h>
#include "../../../../dma-buf/heaps/samsung/samsung-dma-heap.h"

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */

static LIST_HEAD(meminfo_list);
static DEFINE_MUTEX(meminfo_lock);

void rvh_meminfo_proc_show(void *data, struct seq_file *m)
{
	struct meminfo *meminfo;

	seq_printf(m, "ION_heap:       %8lu kB\n",
		   dma_heap_inuse_pages() * PAGE_SIZE / 1024);
	seq_printf(m, "ION_heap_pool:  %8lu kB\n",
		   dma_heap_pool_pages() * PAGE_SIZE / 1024);

	mutex_lock(&meminfo_lock);
	list_for_each_entry(meminfo, &meminfo_list, list) {
		seq_printf(m, "%s: %8lu kB\n",
			   meminfo->name, meminfo->size_kb(meminfo->private));
	}
	mutex_unlock(&meminfo_lock);
}

void register_meminfo(struct meminfo *info)
{
	mutex_lock(&meminfo_lock);
	list_add(&info->list, &meminfo_list);
	mutex_unlock(&meminfo_lock);
}
EXPORT_SYMBOL_GPL(register_meminfo);

void unregister_meminfo(struct meminfo *info)
{
	mutex_lock(&meminfo_lock);
	list_del(&info->list);
	mutex_unlock(&meminfo_lock);
}
EXPORT_SYMBOL_GPL(unregister_meminfo);
