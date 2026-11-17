/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016-2017, Linaro Ltd
 */

#ifndef __QCOM_GLINK_NATIVE_H__
#define __QCOM_GLINK_NATIVE_H__

#include <linux/types.h>

struct device;
struct dev_pm_ops;
struct rpmsg_device;
struct rpmsg_endpoint;

#define GLINK_FEATURE_INTENT_REUSE	BIT(0)
#define GLINK_FEATURE_MIGRATION		BIT(1)
#define GLINK_FEATURE_TRACER_PKT	BIT(2)

struct qcom_glink_pipe {
	size_t length;

	size_t (*avail)(struct qcom_glink_pipe *glink_pipe);

	void (*peek)(struct qcom_glink_pipe *glink_pipe, void *data,
		     unsigned int offset, size_t count);
	void (*advance)(struct qcom_glink_pipe *glink_pipe, size_t count);

	void (*write)(struct qcom_glink_pipe *glink_pipe,
		      const void *hdr, size_t hlen,
		      const void *data, size_t dlen);
	void (*kick)(struct qcom_glink_pipe *glink_pipe);
};

struct qcom_glink;

struct qcom_glink *qcom_glink_native_probe(struct device *dev,
					   unsigned long features,
					   struct qcom_glink_pipe *rx,
					   struct qcom_glink_pipe *tx,
					   bool intentless);
void qcom_glink_native_remove(struct qcom_glink *glink);
void qcom_glink_native_rx(struct qcom_glink *glink);

extern const struct dev_pm_ops glink_native_pm_ops;
int qcom_glink_native_start(struct qcom_glink *glink);
void qcom_glink_native_unregister(struct qcom_glink *glink);

int qcom_glink_get_signals(struct rpmsg_endpoint *ept);
int qcom_glink_set_signals(struct rpmsg_endpoint *ept, u32 set, u32 clear);
int qcom_glink_register_signals_cb(struct rpmsg_endpoint *ept,
	int (*signals_cb)(struct rpmsg_device *dev, void *priv, u32 old, u32 new));
bool qcom_glink_rx_done_supported(struct rpmsg_endpoint *ept);
int qcom_glink_rx_done(struct rpmsg_endpoint *ept, void *data);

#endif
