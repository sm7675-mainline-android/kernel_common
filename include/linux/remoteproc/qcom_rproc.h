#ifndef __QCOM_RPROC_H__
#define __QCOM_RPROC_H__

struct notifier_block;

/**
 * enum qcom_ssr_notify_type - Startup/Shutdown events related to a remoteproc
 * processor.
 *
 * @QCOM_SSR_BEFORE_POWERUP:	Remoteproc about to start (prepare stage)
 * @QCOM_SSR_AFTER_POWERUP:	Remoteproc is running (start stage)
 * @QCOM_SSR_BEFORE_SHUTDOWN:	Remoteproc crashed or shutting down (stop stage)
 * @QCOM_SSR_AFTER_SHUTDOWN:	Remoteproc is down (unprepare stage)
 */
enum qcom_ssr_notify_type {
	QCOM_SSR_BEFORE_POWERUP,
	QCOM_SSR_AFTER_POWERUP,
	QCOM_SSR_BEFORE_SHUTDOWN,
	QCOM_SSR_AFTER_SHUTDOWN,
};

struct qcom_ssr_notify_data {
	const char *name;
	bool crashed;
};

#if IS_ENABLED(CONFIG_QCOM_RPROC_COMMON)

void *qcom_register_ssr_notifier(const char *name, struct notifier_block *nb);
int qcom_unregister_ssr_notifier(void *notify, struct notifier_block *nb);
/* KPORT: vendor glink subdev uses the early-SSR chain (implemented in the
 * vendor rproc_qcom_common module). */
void *qcom_register_early_ssr_notifier(const char *name, struct notifier_block *nb);
int qcom_unregister_early_ssr_notifier(void *notify, struct notifier_block *nb);

#else

static inline void *qcom_register_ssr_notifier(const char *name,
					       struct notifier_block *nb)
{
	return NULL;
}

static inline int qcom_unregister_ssr_notifier(void *notify,
					       struct notifier_block *nb)
{
	return 0;
}

static inline void *qcom_register_early_ssr_notifier(const char *name,
						struct notifier_block *nb)
{
	return NULL;
}

static inline int qcom_unregister_early_ssr_notifier(void *notify,
						struct notifier_block *nb)
{
	return 0;
}

#endif


/* Allow the vendor ADSP loader to select DTB firmware at runtime. */
struct rproc;
int qcom_rproc_set_dtb_firmware(struct rproc *rproc, const char *dtb_fw_name);

#endif
