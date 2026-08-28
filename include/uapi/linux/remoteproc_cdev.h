/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * IOCTLs for Remoteproc's character device interface.
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _UAPI_REMOTEPROC_CDEV_H_
#define _UAPI_REMOTEPROC_CDEV_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define RPROC_MAGIC	0xB7

/*
 * The RPROC_SET_SHUTDOWN_ON_RELEASE ioctl allows to enable/disable the shutdown of a remote
 * processor automatically when the controlling userpsace closes the char device interface.
 *
 * input parameter: integer
 *   0		: disable automatic shutdown
 *   other	: enable automatic shutdown
 */
#define RPROC_SET_SHUTDOWN_ON_RELEASE _IOW(RPROC_MAGIC, 1, __s32)

/*
 * The RPROC_GET_SHUTDOWN_ON_RELEASE ioctl gets information about whether the automatic shutdown of
 * a remote processor is enabled or disabled when the controlling userspace closes the char device
 * interface.
 *
 * output parameter: integer
 *   0		: automatic shutdown disable
 *   other	: automatic shutdown enable
 */
#define RPROC_GET_SHUTDOWN_ON_RELEASE _IOR(RPROC_MAGIC, 2, __s32)

/**
 * struct rproc_dma_buf_phys - dma-buf physical address translation
 * @fd:		[in]  dma-buf file descriptor to translate
 * @pad:	[in]  must be zero
 * @phys:	[out] physical address of the buffer
 */
struct rproc_dma_buf_phys {
	__u32 fd;
	__u32 pad;
	__u64 phys;
};

/*
 * The RPROC_GET_DMA_BUF_PHYS ioctl translates a dma-buf file descriptor into
 * the physical address of its backing memory.
 *
 * Some DSPs, such as the TI C7x, run without an MMU and are programmed with
 * raw physical addresses. Userspace allocates buffers from a dma-buf heap and
 * needs to tell the remote processor where they live before triggering work.
 *
 * The buffer must be physically contiguous, which is guaranteed for carveout
 * and CMA heaps but not in general, so a scattered buffer is rejected.
 */
#define RPROC_GET_DMA_BUF_PHYS _IOWR(RPROC_MAGIC, 0, struct rproc_dma_buf_phys)

#endif
