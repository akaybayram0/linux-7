// SPDX-License-Identifier: GPL-2.0-only
/*
 * Character device interface driver for Remoteproc framework.
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/uaccess.h>
#include <uapi/linux/remoteproc_cdev.h>

#include "remoteproc_internal.h"

#define NUM_RPROC_DEVICES	64
static dev_t rproc_major;

/*
 * Translate a dma-buf fd into the physical address of its backing memory.
 *
 * The address is taken from the CPU pages of the scatterlist, not from
 * sg_dma_address(). The DMA address is the wrong answer here: the remote
 * processor is not the device the buffer gets mapped for, and on a device
 * with a narrow DMA mask the DMA API would bounce the buffer through the
 * swiotlb and report the address of the bounce copy.
 *
 * Mapping is therefore performed against a device whose mask covers all of
 * memory, so that no bouncing occurs and the pages are left untouched.
 *
 * A physically contiguous buffer is described by exactly one segment; anything
 * else cannot be expressed as a single address and is rejected.
 */
static int rproc_get_dma_buf_phys(struct rproc *rproc, void __user *argp)
{
	struct device *dev = rproc->dev.parent;
	struct rproc_dma_buf_phys req;
	struct dma_buf_attachment *attach;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	u64 saved_mask;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (req.pad)
		return -EINVAL;

	dmabuf = dma_buf_get(req.fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	attach = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto put_dmabuf;
	}

	saved_mask = dma_get_mask(dev);
	dma_set_mask(dev, DMA_BIT_MASK(64));

	sgt = dma_buf_map_attachment_unlocked(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto restore_mask;
	}

	if (sgt->orig_nents != 1) {
		dev_err(&rproc->dev, "dma-buf is not physically contiguous\n");
		ret = -EINVAL;
		goto unmap;
	}

	req.phys = page_to_phys(sg_page(sgt->sgl)) + sgt->sgl->offset;
	ret = copy_to_user(argp, &req, sizeof(req)) ? -EFAULT : 0;

unmap:
	dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
restore_mask:
	dma_set_mask(dev, saved_mask);
	dma_buf_detach(dmabuf, attach);
put_dmabuf:
	dma_buf_put(dmabuf);

	return ret;
}

static ssize_t rproc_cdev_write(struct file *filp, const char __user *buf, size_t len, loff_t *pos)
{
	struct rproc *rproc = container_of(filp->f_inode->i_cdev, struct rproc, cdev);
	int ret = 0;
	char cmd[10];

	if (!len || len > sizeof(cmd))
		return -EINVAL;

	ret = copy_from_user(cmd, buf, len);
	if (ret)
		return -EFAULT;

	if (!strncmp(cmd, "start", len)) {
		ret = rproc_boot(rproc);
	} else if (!strncmp(cmd, "stop", len)) {
		ret = rproc_shutdown(rproc);
	} else if (!strncmp(cmd, "detach", len)) {
		ret = rproc_detach(rproc);
	} else {
		dev_err(&rproc->dev, "Unrecognized option\n");
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static long rproc_device_ioctl(struct file *filp, unsigned int ioctl, unsigned long arg)
{
	struct rproc *rproc = container_of(filp->f_inode->i_cdev, struct rproc, cdev);
	void __user *argp = (void __user *)arg;
	s32 param;

	switch (ioctl) {
	case RPROC_SET_SHUTDOWN_ON_RELEASE:
		if (copy_from_user(&param, argp, sizeof(s32)))
			return -EFAULT;

		rproc->cdev_put_on_release = !!param;
		break;
	case RPROC_GET_SHUTDOWN_ON_RELEASE:
		param = (s32)rproc->cdev_put_on_release;
		if (copy_to_user(argp, &param, sizeof(s32)))
			return -EFAULT;

		break;
	case RPROC_GET_DMA_BUF_PHYS:
		return rproc_get_dma_buf_phys(rproc, argp);
	default:
		dev_err(&rproc->dev, "Unsupported ioctl\n");
		return -EINVAL;
	}

	return 0;
}

static int rproc_cdev_release(struct inode *inode, struct file *filp)
{
	struct rproc *rproc = container_of(inode->i_cdev, struct rproc, cdev);
	int ret = 0;

	if (!rproc->cdev_put_on_release)
		return 0;

	if (rproc->state == RPROC_RUNNING)
		rproc_shutdown(rproc);
	else if (rproc->state == RPROC_ATTACHED)
		ret = rproc_detach(rproc);

	return ret;
}

static const struct file_operations rproc_fops = {
	.write = rproc_cdev_write,
	.unlocked_ioctl = rproc_device_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.release = rproc_cdev_release,
};

int rproc_char_device_add(struct rproc *rproc)
{
	int ret;

	cdev_init(&rproc->cdev, &rproc_fops);
	rproc->cdev.owner = THIS_MODULE;

	rproc->dev.devt = MKDEV(MAJOR(rproc_major), rproc->index);
	cdev_set_parent(&rproc->cdev, &rproc->dev.kobj);
	ret = cdev_add(&rproc->cdev, rproc->dev.devt, 1);
	if (ret < 0)
		dev_err(&rproc->dev, "Failed to add char dev for %s\n", rproc->name);

	return ret;
}

void rproc_char_device_remove(struct rproc *rproc)
{
	cdev_del(&rproc->cdev);
}

void __init rproc_init_cdev(void)
{
	int ret;

	ret = alloc_chrdev_region(&rproc_major, 0, NUM_RPROC_DEVICES, "remoteproc");
	if (ret < 0)
		pr_err("Failed to alloc rproc_cdev region, err %d\n", ret);
}
