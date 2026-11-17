#ifndef _LINUX_SPLASHLOG_H
#define _LINUX_SPLASHLOG_H

#include <linux/init.h>
#include <linux/types.h>

struct kernel_siginfo;

enum splashlog_module_phase {
	SL_MODULE_END,
	SL_MODULE_FILE,
	SL_MODULE_INIT,
	SL_MODULE_INIT_DONE,
	SL_MODULE_ASYNC,
	SL_MODULE_FINISH,
};

#ifdef CONFIG_SPLASHLOG
void __init splashlog_early_init(void);
void splashlog_before_init(void);
bool splashlog_modules_enabled(void);
void splashlog_modtrace(const char *name);
void splashlog_modfail(const char *name, int err);
void splashlog_modattempt(const char *path, int err);
void splashlog_modphase(const char *name, enum splashlog_module_phase phase);
bool splashlog_buffer_in_use(phys_addr_t base, size_t size);
void splashlog_display_takeover(void);
void splashlog_user_fault(const struct kernel_siginfo *info, bool handled);
void splashlog_warn(const char *file, int line, void *caller);
#else
static inline void splashlog_early_init(void) { }
static inline void splashlog_before_init(void) { }
static inline bool splashlog_modules_enabled(void) { return false; }
static inline void splashlog_modtrace(const char *name) { }
static inline void splashlog_modfail(const char *name, int err) { }
static inline void splashlog_modattempt(const char *path, int err) { }
static inline void splashlog_modphase(const char *name,
				    enum splashlog_module_phase phase) { }
static inline bool splashlog_buffer_in_use(phys_addr_t base, size_t size)
{
	return false;
}
static inline void splashlog_display_takeover(void) { }
static inline void splashlog_user_fault(const struct kernel_siginfo *info,
				       bool handled) { }
static inline void splashlog_warn(const char *file, int line, void *caller) { }
#endif

#endif
