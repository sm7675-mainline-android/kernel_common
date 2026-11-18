/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __UCSI_GLINK_H__
#define __UCSI_GLINK_H__

#include <linux/errno.h>
#include <linux/usb/typec.h>

struct ucsi_glink_constat_info {
	enum typec_accessory acc;
};

struct notifier_block;

#if IS_ENABLED(CONFIG_UCSI_PMIC_GLINK)
int register_ucsi_glink_notifier(struct notifier_block *notifier);
int unregister_ucsi_glink_notifier(struct notifier_block *notifier);
#else
static inline int register_ucsi_glink_notifier(struct notifier_block *notifier)
{
	return -ENODEV;
}

static inline int unregister_ucsi_glink_notifier(struct notifier_block *notifier)
{
	return -ENODEV;
}
#endif

#endif
