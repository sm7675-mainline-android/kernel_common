# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the linux kernel.
#

obj-$(CONFIG_QCOM_SCM)		+= qcom-scm.o
# qtee_shmbridge is linked into qcom-scm.ko, exactly like the vendor tree
# (crd/drivers/firmware/Makefile: qcom-scm-objs-$(CONFIG_QTEE_SHM_BRIDGE) +=
# qtee_shmbridge.o).  qseecom_dlkm, msm_drm, msm_kgsl, crypto-qti and others
# import qtee_shmbridge_*, so it has to ship inside a module they already
# depend on.  It must NOT be a module of its own: it calls
# qcom_scm_enable_shm_bridge(), which lives in the same object.
qcom-scm-objs += qcom_scm.o qcom_scm-smc.o qcom_scm-legacy.o qtee_shmbridge.o
# qcom_tzmem stays a SEPARATE module.  It must not be linked in here: it is a
# module of its own in the vendor_boot modules.load, and merging it would
# duplicate every qcom_tzmem_* export.  There is no link cycle either way as
# long as CONFIG_QCOM_TZMEM_MODE_GENERIC=y -- with MODE_SHMBRIDGE=y qcom_tzmem
# calls qcom_scm_shm_bridge_*() while qcom_scm calls qcom_tzmem_alloc(), which
# depmod reports as "Cycle detected: qcom_tzmem -> qcom_scm -> qcom_tzmem" and
# then neither module loads.  Keep GENERIC.
CFLAGS_qcom_scm-smc.o := -I$(src)
obj-$(CONFIG_QCOM_TZMEM)	+= qcom_tzmem.o
obj-$(CONFIG_QCOM_QSEECOM)	+= qcom_qseecom.o
obj-$(CONFIG_QCOM_QSEECOM_UEFISECAPP) += qcom_qseecom_uefisecapp.o
