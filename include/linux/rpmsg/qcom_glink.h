/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_RPMSG_QCOM_GLINK_H
#define _LINUX_RPMSG_QCOM_GLINK_H

#include <linux/device.h>

/* KPORT_VENDOR_GLINK_COMPAT: vendor remoteproc/glink users still reference the
 * pre-6.18 glink edge type and the explicit "start" / early-SSR hooks.  Upstream
 * now starts the edge from qcom_glink_native_probe() and has no early-SSR path,
 * so the vendor call sites only need a no-op shim. */
struct qcom_glink;

struct qcom_glink_smem;

/* Vendor extension: memory-sharing DA translation, built out of tree as
 * qcom_glink_memshare.ko (upstream dropped it together with zero-copy intents). */
struct qcom_glink_mem_entry;

struct qcom_glink_mem_entry *
qcom_glink_mem_entry_init(struct device *dev, void *va, dma_addr_t dma,
			  size_t len, u32 da);
void qcom_glink_mem_entry_free(struct qcom_glink_mem_entry *mem);
void *qcom_glink_prepare_da_for_cpu(u64 da, size_t len);

#if IS_ENABLED(CONFIG_RPMSG_QCOM_GLINK)
void qcom_glink_ssr_notify(const char *ssr_name);
#else
static inline void qcom_glink_ssr_notify(const char *ssr_name) {}
#endif

#if IS_ENABLED(CONFIG_RPMSG_QCOM_GLINK_SMEM)

struct qcom_glink_smem *qcom_glink_smem_register(struct device *parent,
						 struct device_node *node);
void qcom_glink_smem_unregister(struct qcom_glink_smem *glink);
static inline int qcom_glink_smem_start(struct qcom_glink *glink) { return 0; }
static inline void qcom_glink_early_ssr_notify(void *data) { }

#else

static inline struct qcom_glink_smem *
qcom_glink_smem_register(struct device *parent,
			 struct device_node *node)
{
	return NULL;
}

static inline void qcom_glink_smem_unregister(struct qcom_glink_smem *glink) {}
#endif

#endif
