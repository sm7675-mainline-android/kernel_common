/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SELINUX_H
#define _LINUX_SELINUX_H

#include <linux/errno.h>
#include <linux/kconfig.h>

#if IS_ENABLED(CONFIG_SECURITY_SELINUX)
int selinux_get_enforcing(void);
#else
static inline int selinux_get_enforcing(void)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_SELINUX_H */
