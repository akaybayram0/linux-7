// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF carveout heap exporter
 *
 * TI vision-apps (ve genel olarak TIOVX tabanlı yığın) paylaşımlı belleği
 * "dma-heap-carveout" uyumlu reserved-memory bölgeleri üzerinden ister ve
 * /dev/dma_heap/carveout_<bölge-adı> düğümünü açmayı bekler. Mainline'da
 * yalnız system ve cma heap'leri bulunduğu için bu düğüm oluşmaz ve
 * uygulama "Failed to initialize DMA HEAP" ile düşer.
 *
 * Bu sürücü, device tree'de "dma-heap-carveout" olarak işaretlenmiş her
 * reserved-memory bölgesi için bir dma-heap kaydeder. Tahsis, bölgeye ait
 * gen_pool üzerinden yapılır; bellek fiziksel olarak ardışıktır ve bölge
 * sınırları içinde kalır.
 */

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

struct carveout_heap {
	struct dma_heap		*heap;
	struct gen_pool		*pool;
	phys_addr_t		base;
	size_t			size;
};

struct carveout_buffer {
	struct carveout_heap	*heap;
	struct sg_table		sg_table;
	phys_addr_t		paddr;
	size_t			len;
	void			*vaddr;		/* lazy vmap, mutex ile korunur */
	struct mutex		lock;
	int			vmap_cnt;
	struct list_head	attachments;
};

struct carveout_attachment {
	struct device		*dev;
	struct sg_table		table;
	struct list_head	list;
	bool			mapped;
};

static int carveout_attach(struct dma_buf *dmabuf,
			   struct dma_buf_attachment *attachment)
{
	struct carveout_buffer *buf = dmabuf->priv;
	struct carveout_attachment *a;
	int ret;

	a = kzalloc_obj(*a, GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table(&a->table, 1, GFP_KERNEL);
	if (ret) {
		kfree(a);
		return -ENOMEM;
	}
	sg_set_page(a->table.sgl, phys_to_page(buf->paddr), buf->len, 0);

	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);
	a->mapped = false;
	attachment->priv = a;

	mutex_lock(&buf->lock);
	list_add(&a->list, &buf->attachments);
	mutex_unlock(&buf->lock);

	return 0;
}

static void carveout_detach(struct dma_buf *dmabuf,
			    struct dma_buf_attachment *attachment)
{
	struct carveout_buffer *buf = dmabuf->priv;
	struct carveout_attachment *a = attachment->priv;

	mutex_lock(&buf->lock);
	list_del(&a->list);
	mutex_unlock(&buf->lock);

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *carveout_map_dma_buf(struct dma_buf_attachment *attachment,
					     enum dma_data_direction direction)
{
	struct carveout_attachment *a = attachment->priv;
	struct sg_table *table = &a->table;
	int ret;

	ret = dma_map_sgtable(attachment->dev, table, direction, 0);
	if (ret)
		return ERR_PTR(-ENOMEM);

	a->mapped = true;
	return table;
}

static void carveout_unmap_dma_buf(struct dma_buf_attachment *attachment,
				   struct sg_table *table,
				   enum dma_data_direction direction)
{
	struct carveout_attachment *a = attachment->priv;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, 0);
}

static int carveout_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct carveout_buffer *buf = dmabuf->priv;
	unsigned long pfn = PHYS_PFN(buf->paddr);
	unsigned long len = vma->vm_end - vma->vm_start;

	if (len > buf->len || vma->vm_pgoff)
		return -EINVAL;

	/* Aygıtlarla paylaşılan bellek: önbelleklenmemiş eşleme */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start, pfn, len, vma->vm_page_prot);
}

static int carveout_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct carveout_buffer *buf = dmabuf->priv;
	void *vaddr;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt) {
		buf->vmap_cnt++;
		iosys_map_set_vaddr(map, buf->vaddr);
		mutex_unlock(&buf->lock);
		return 0;
	}

	vaddr = memremap(buf->paddr, buf->len, MEMREMAP_WC);
	if (!vaddr) {
		mutex_unlock(&buf->lock);
		return -ENOMEM;
	}

	buf->vaddr = vaddr;
	buf->vmap_cnt++;
	iosys_map_set_vaddr(map, vaddr);
	mutex_unlock(&buf->lock);

	return 0;
}

static void carveout_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct carveout_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (!--buf->vmap_cnt) {
		memunmap(buf->vaddr);
		buf->vaddr = NULL;
	}
	mutex_unlock(&buf->lock);
	iosys_map_clear(map);
}

static void carveout_dma_buf_release(struct dma_buf *dmabuf)
{
	struct carveout_buffer *buf = dmabuf->priv;

	if (buf->vmap_cnt > 0)
		memunmap(buf->vaddr);

	gen_pool_free(buf->heap->pool, (unsigned long)buf->paddr, buf->len);
	sg_free_table(&buf->sg_table);
	kfree(buf);
}

static const struct dma_buf_ops carveout_buf_ops = {
	.attach		= carveout_attach,
	.detach		= carveout_detach,
	.map_dma_buf	= carveout_map_dma_buf,
	.unmap_dma_buf	= carveout_unmap_dma_buf,
	.release	= carveout_dma_buf_release,
	.mmap		= carveout_mmap,
	.vmap		= carveout_vmap,
	.vunmap		= carveout_vunmap,
};

static struct dma_buf *carveout_allocate(struct dma_heap *heap,
					 unsigned long len,
					 u32 fd_flags,
					 u64 heap_flags)
{
	struct carveout_heap *ch = dma_heap_get_drvdata(heap);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct carveout_buffer *buf;
	struct dma_buf *dmabuf;
	unsigned long paddr;
	int ret;

	buf = kzalloc_obj(*buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	mutex_init(&buf->lock);
	INIT_LIST_HEAD(&buf->attachments);
	buf->heap = ch;
	buf->len = PAGE_ALIGN(len);

	paddr = gen_pool_alloc(ch->pool, buf->len);
	if (!paddr) {
		ret = -ENOMEM;
		goto err_free_buf;
	}
	buf->paddr = (phys_addr_t)paddr;

	ret = sg_alloc_table(&buf->sg_table, 1, GFP_KERNEL);
	if (ret)
		goto err_free_pool;
	sg_set_page(buf->sg_table.sgl, phys_to_page(buf->paddr), buf->len, 0);

	exp_info.ops = &carveout_buf_ops;
	exp_info.size = buf->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_free_table;
	}

	return dmabuf;

err_free_table:
	sg_free_table(&buf->sg_table);
err_free_pool:
	gen_pool_free(ch->pool, paddr, buf->len);
err_free_buf:
	kfree(buf);
	return ERR_PTR(ret);
}

static const struct dma_heap_ops carveout_heap_ops = {
	.allocate = carveout_allocate,
};

static int __init carveout_heap_add(struct reserved_mem *rmem)
{
	struct dma_heap_export_info exp_info;
	struct carveout_heap *ch;
	char *name, *at;
	int ret;

	ch = kzalloc_obj(*ch, GFP_KERNEL);
	if (!ch)
		return -ENOMEM;

	ch->base = rmem->base;
	ch->size = rmem->size;

	ch->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!ch->pool) {
		ret = -ENOMEM;
		goto err_free_ch;
	}

	ret = gen_pool_add(ch->pool, rmem->base, rmem->size, -1);
	if (ret)
		goto err_destroy_pool;

	/*
	 * Kullanıcı alanı /dev/dma_heap/carveout_<bölge-adı> bekler ve adreste
	 * unit-address ("@8a0000000") BULUNMAZ. reserved_mem->name düğümün tam
	 * adıdır, bu yüzden '@' işaretinden itibarasını kırpıyoruz.
	 */
	name = kasprintf(GFP_KERNEL, "carveout_%s", rmem->name);
	if (!name) {
		ret = -ENOMEM;
		goto err_destroy_pool;
	}
	at = strchr(name, '@');
	if (at)
		*at = '\0';

	exp_info.name = name;
	exp_info.ops = &carveout_heap_ops;
	exp_info.priv = ch;

	ch->heap = dma_heap_add(&exp_info);
	if (IS_ERR(ch->heap)) {
		ret = PTR_ERR(ch->heap);
		goto err_free_name;
	}

	pr_info("carveout_heap: %s kaydedildi (%pa, %zu bayt)\n",
		name, &ch->base, ch->size);

	kfree(name);
	return 0;

err_free_name:
	kfree(name);
err_destroy_pool:
	gen_pool_destroy(ch->pool);
err_free_ch:
	kfree(ch);
	return ret;
}

static int __init carveout_heap_init(void)
{
	struct device_node *np;
	struct reserved_mem *rmem;
	int count = 0;

	for_each_compatible_node(np, NULL, "dma-heap-carveout") {
		rmem = of_reserved_mem_lookup(np);
		if (!rmem) {
			pr_warn("carveout_heap: %pOF için reserved_mem bulunamadı\n", np);
			continue;
		}
		if (!carveout_heap_add(rmem))
			count++;
	}

	if (!count)
		pr_debug("carveout_heap: uygun bölge yok\n");

	return 0;
}

module_init(carveout_heap_init);
MODULE_DESCRIPTION("DMA-BUF Carveout Heap");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_IMPORT_NS("DMA_BUF_HEAP");
MODULE_LICENSE("GPL");
