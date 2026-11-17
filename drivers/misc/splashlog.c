// SPDX-License-Identifier: GPL-2.0-only
/*
 * Early framebuffer console and persistent boot log for SM7675.
 *
 * Pixel writes use the bootloader's reserved splash buffer. Native DRM must
 * call splashlog_display_takeover() before reusing scanout; recording continues
 * after rendering and its timers stop. Panic may render one final snapshot.
 *
 * The black box has a persistent_ram-compatible header and a private trailer.
 * Its physical region must be reserved, linearly mapped and disjoint from the
 * framebuffer and ramoops zones. Payload is cleaned to PoC before publishing
 * the write cursor. Previous-boot replay uses separate storage so console
 * replay cannot overwrite the evidence being displayed.
 *
 * spl.bootdump selects cumulative diagnostics:
 *   0: console, compact status and one-line kernel fault; no task sampling.
 *   1: selected evidence, module history and bounded boot-progress summaries.
 *   2: full kernel fault context and periodic service/blocked task snapshots.
 * Block-device dumps and heartbeat logging require their own explicit options.
 * See Documentation/admin-guide/splashlog.rst for parameter and storage details.
 */

#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kdebug.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/kthread.h>
#include <linux/memblock.h>
#include <linux/minmax.h>
#include <linux/nmi.h>
#include <linux/panic.h>
#include <linux/panic_notifier.h>
#include <linux/pid.h>
#include <linux/reboot.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#include <linux/sched/stat.h>
#include <linux/sched/task_stack.h>
#include <linux/freezer.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/splashlog.h>
#include <linux/stacktrace.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/ptrace.h>

#define SL_STAGES	10

/* Partial redraws are bounded; the slower tick completes older screen rows. */
#define SL_TICK_MS	250

#define SL_RENDER_J	8

#define SL_KICK_ROWS	16
#define SL_LINES	1024

#define SL_LINELEN	200
#define SL_MAXROWS	128
#define SL_BAR_H	12

#define SL_MARGIN	12

#define SL_SEP		0xff00c8d0u

/* Header signature matches persistent_ram with a zero signature seed. */
#define SL_BB_MAGIC	0x43474244u	/* "DBGC" */

#define SL_BB_OFF	0x2800000
#define SL_BB_SIZE	0x100000
#define SL_BB_REPLAY	131072

/* The trailer is private to splashlog and excluded from the text ring. */
#define SL_CANARY_OFF	(SL_BB_SIZE - 32)
#define SL_CANARY_MAGIC	0x534c434eu	/* "SLCN" */

#define SL_SPARE_OFF	0x2900000u
#define SL_SPARE_MAGIC	0x534c5350u	/* "SLSP" */

#define SL_FG		0xffffffffu
#define SL_BG		0xff001028u
#define SL_BAR_BG	0xff101010u
#define SL_HB_ON	0xffffffffu
#define SL_HB_OFF	0xff505050u
#define SL_RED		0xffff0000u
#define SL_YEL		0xffffff00u

#define SL_RP_BG	0xff300000u
#define SL_RP_BANNER	0xffffff00u
#define SL_RP_BANNER_FG	0xff000000u
#define SL_RP_ROWS	512

#define SL_RP_SCALE	1

static phys_addr_t sl_phys = 0xe3940000;
static size_t sl_phys_size = 0x2b00000;
static u8 *sl_fb;
static size_t sl_size, sl_frame_bytes;
static unsigned int sl_w = 1240, sl_h = 2772, sl_stride;
static unsigned int sl_scale_x2 = 3, sl_cw, sl_ch, sl_cols, sl_rows;
static unsigned int sl_frames = 1;
static bool sl_ready, sl_dead, sl_force, sl_dirty, sl_fb_frozen;
static bool sl_native_display;

static bool sl_bb_frozen;

static bool sl_halted;
static unsigned int sl_stage = 1;

static const char *sl_fault_why = "unknown";
static bool sl_watchdog_fault;
static unsigned int sl_fault_phase;
static unsigned long sl_last_console = INITIAL_JIFFIES;
static unsigned long sl_last_kbeat = INITIAL_JIFFIES;
static atomic_t sl_console_count = ATOMIC_INIT(0);
static u64 sl_printk_seq, sl_delivery_seq;
static char sl_first_die[384];
static unsigned long sl_first_die_stack[32];
static unsigned int sl_first_die_depth;

static phys_addr_t sl_bb_phys;
static u8 *sl_bb;
static size_t sl_bb_room;
static bool sl_bb_ok;
static bool sl_bb_disabled;
static unsigned int sl_bb_bytes;

static unsigned int sl_boots;
static unsigned int sl_prev_panic;
static unsigned int sl_prev_sig;
static int sl_prev_start, sl_prev_size;
static bool sl_have_old;

static unsigned int sl_spare_boots;

static char sl_rp[SL_RP_ROWS][SL_LINELEN];
static unsigned int sl_rp_rows;
static char sl_rp_cur[SL_LINELEN];
static unsigned int sl_rp_curlen;
static unsigned int sl_rp_y;

static unsigned int sl_rp_cw, sl_rp_ch, sl_rp_cols;

static bool sl_rp_full;

static bool sl_rp_hold;
static bool sl_rp_drawn;
static unsigned int sl_replay_at = 3;

static char sl_ring[SL_LINES][SL_LINELEN];
static unsigned int sl_rlen[SL_LINES];
static unsigned int sl_head, sl_used;

static char sl_shown[SL_MAXROWS][SL_LINELEN];

static char sl_prow[SL_MAXROWS][SL_LINELEN];

static struct timer_list sl_timer;
static bool sl_timer_on;
static bool sl_flip;

/* Console writes only arm a bounded redraw; they do not render directly. */
static struct timer_list sl_rtimer;
static bool sl_rtimer_ok;
static bool sl_rtimer_armed;

static bool sl_enabled, sl_quiet, sl_mods;

static unsigned int sl_halt;

/* Serializes text rings, persistent writes and framebuffer updates. */
static DEFINE_RAW_SPINLOCK(sl_lock);

static void sl_feed_locked(const char *s, unsigned int count, bool to_bb);
static void sl_render(void);
static void sl_render_locked(bool partial);
static void sl_bar_locked(void);
static void sl_rp_render_locked(bool full);
static void sl_rp_show(void);
static void sl_halt_forever(void);
static void sl_puts(const char *s, bool to_bb);

static void sl_evidence_dump(void);
static int sl_bootdump;
static void sl_dump_modring(void);
static void sl_status_locked(void);
static void sl_diag_locked(void);
static unsigned int sl_chunks(unsigned int len, unsigned int cols);
static void sl_wrap_into(char *dst, const char *src, unsigned int len,
			 unsigned int c, unsigned int cols);

static int __init splashlog_setup(char *str)
{
	if (!str) {
		sl_enabled = true;
		return 0;
	}
	return kstrtobool(str, &sl_enabled);
}
early_param("spl.enable", splashlog_setup);

static unsigned int sl_fb_mode = 1;

static int __init splashlog_fb_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!kstrtouint(str, 0, &v) && v <= 3)
		sl_fb_mode = v;
	return 0;
}
early_param("spl.fb", splashlog_fb_setup);

static bool sl_beat_on;

static int __init splashlog_beat_setup(char *str)
{
	sl_beat_on = true;
	return 0;
}
early_param("spl.beat", splashlog_beat_setup);

static int __init splashlog_bootdump_setup(char *str)
{
	unsigned int v;

	if (!str || kstrtouint(str, 0, &v) || v > 2)
		v = 0;
	sl_bootdump = v;
	return 0;
}
early_param("spl.bootdump", splashlog_bootdump_setup);

static int __init splashlog_quiet_setup(char *str)
{
	sl_quiet = true;
	return 0;
}
early_param("splashlog.quiet", splashlog_quiet_setup);

static int __init splashlog_mods_setup(char *str)
{
	sl_mods = true;
	return 0;
}
early_param("splashlog.mods", splashlog_mods_setup);

static int __init splashlog_halt_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!kstrtouint(str, 0, &v) && v <= 2)
		sl_halt = v;
	return 0;
}
early_param("spl.halt", splashlog_halt_setup);

static int __init splashlog_scale_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1.5"))
		sl_scale_x2 = 3;
	else if (!kstrtouint(str, 0, &v) && v >= 1 && v <= 3)
		sl_scale_x2 = v * 2;
	return 0;
}
early_param("splashlog.scale", splashlog_scale_setup);

static int __init splashlog_phys_setup(char *str)
{
	unsigned long long v;

	if (!str)
		return -EINVAL;

	if (!kstrtoull(str, 0, &v))
		sl_phys = v;
	return 0;
}
early_param("splashlog.phys", splashlog_phys_setup);

static int __init splashlog_bb_setup(char *str)
{
	unsigned long long v;

	if (!str)
		return -EINVAL;

	if (!strcmp(str, "off") || !strcmp(str, "0")) {
		sl_bb_disabled = true;
		return 0;
	}
	if (!kstrtoull(str, 0, &v))
		sl_bb_phys = v;
	return 0;
}
early_param("spl.bb", splashlog_bb_setup);

static int __init splashlog_width_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!kstrtouint(str, 0, &v) && v >= 320 && v <= 4096)
		sl_w = v;
	return 0;
}
early_param("splashlog.width", splashlog_width_setup);

static int __init splashlog_height_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!kstrtouint(str, 0, &v) && v >= 320 && v <= 8192)
		sl_h = v;
	return 0;
}
early_param("splashlog.height", splashlog_height_setup);

static int __init splashlog_replay_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!strcmp(str, "off") || !strcmp(str, "0")) {
		sl_replay_at = 0;
		return 0;
	}
	if (!kstrtouint(str, 0, &v) && v <= 3)
		sl_replay_at = v;
	return 0;
}
early_param("splashlog.replay", splashlog_replay_setup);

static unsigned int sl_stall_secs;
static unsigned long sl_last_msg;
static struct timer_list sl_stall_timer;
static bool sl_stall_ok;

static int __init splashlog_stall_setup(char *str)
{
	unsigned int v;

	if (!str)
		return 0;
	if (!kstrtouint(str, 0, &v) && v <= 600)
		sl_stall_secs = v;
	return 0;
}
early_param("splashlog.stall", splashlog_stall_setup);

static unsigned int sl_deadline_ms;
static bool sl_deadline_panic;

/* One timer per CPU allows another CPU to detect a locally blocked tick. */
static DEFINE_PER_CPU(struct timer_list, sl_dl_timer);

static int sl_dl_early_cpu = -1;

/* Only one CPU may dump state and start a diagnostic restart. */
static atomic_t sl_dl_fired = ATOMIC_INIT(0);

static int __init splashlog_deadline_setup(char *str)
{
	unsigned long seconds;
	unsigned int fraction = 0, digits = 0;
	char integer[24];
	const char *cursor;
	size_t length;

	if (!str)
		return -EINVAL;
	length = strcspn(str, ".");
	if (!length || length >= sizeof(integer))
		return -EINVAL;
	memcpy(integer, str, length);
	integer[length] = 0;
	if (kstrtoul(integer, 0, &seconds))
		return -EINVAL;
	cursor = str + length;
	if (*cursor == '.') {
		cursor++;
		while (digits < 3 && *cursor >= '0' && *cursor <= '9') {
			fraction = fraction * 10 + (*cursor++ - '0');
			digits++;
		}
		if (!digits)
			return -EINVAL;
		while (digits++ < 3)
			fraction *= 10;
	}
	if (*cursor)
		return -EINVAL;
	seconds = min(seconds, 600UL);
	sl_deadline_ms = seconds * 1000 + fraction;
	return 0;
}

early_param("spl.deadline", splashlog_deadline_setup);

static int __init splashlog_deadline_panic_setup(char *str)
{
	sl_deadline_panic = !str || strcmp(str, "0");
	return 0;
}
early_param("spl.deadline_panic", splashlog_deadline_panic_setup);

static bool sl_oops_continue;

static int __init splashlog_oops_setup(char *str)
{
	sl_oops_continue = !str || !strcmp(str, "continue");
	return 0;
}
early_param("splashlog.oops", splashlog_oops_setup);

static bool sl_reboot_on_oops;

static int __init splashlog_reboot_on_oops_setup(char *str)
{
	unsigned int v = 1;

	if (str && !kstrtouint(str, 0, &v))
		sl_reboot_on_oops = !!v;
	else
		sl_reboot_on_oops = true;
	return 0;
}
early_param("spl.reboot_on_oops", splashlog_reboot_on_oops_setup);

static const char *sl_drop;
static unsigned int sl_drop_cnt;
static bool sl_in_trace;

static char sl_drop_buf[128];

static int __init splashlog_drop_setup(char *str)
{
	if (!str || !*str)
		return 0;
	strscpy(sl_drop_buf, str, sizeof(sl_drop_buf));
	sl_drop = sl_drop_buf;
	pr_info("splashlog: drop filter = '%s'\n", sl_drop_buf);
	return 0;
}
early_param("splashlog.drop", splashlog_drop_setup);

static int __init splashlog_spl_drop_setup(char *str)
{
	return splashlog_drop_setup(str);
}
early_param("spl.drop", splashlog_spl_drop_setup);

static const char sl_drop_default[] =
	"init: Sending SIGTERM;init: Sending SIGKILL;"
	"init: Shutdown service;"
	"init: Stopping services;init: Remaining userspace processes except init;"
	"init: pid:;libprocessgroup: Removed cgroup;"
	"init: sync() before umount;init: sync() after umount;"
	"init: sync() after unmount;init: Ready to umount apexes;"
	"init: waitid failed: No child processes;"
	"init: powerctl_shutdown_time_ms;"
	"libprocessgroup: Unable to remove cgroup;"
	"Unmounting /system/apex;Unmounting /apex;"
	"init: Umounting;init: Umounted;init: Unmounting;init: Unmounted;"
	"received SIGTERM;received SIGKILL;"
	"unmounting filesystem;Unmounting filesystem;"
	"umounting filesystem;Umounting filesystem;"
	"SELinux: Skipping restorecon on directory;"
	"Thread Pool max thread count is 0;"
	"EXT4-fs (dm-,mounted filesystem;"
	"Reboot ending. Jumping to kernel;"
	"OPL;leds;"
	"avc: denied;"
	"arm-smmu;arm_smmu;"
	"Adding to iommu group;wakeup function;";

static bool sl_blkdump;

static int __init splashlog_blkdump_setup(char *str)
{
	sl_blkdump = str && (*str == '1' || *str == 'y' || *str == 'Y');
	return 0;
}
early_param("spl.blk", splashlog_blkdump_setup);

static bool sl_pattern_match(const char *pat, const char *buf, unsigned int n)
{
	char tok[96];
	const char *g;

	for (g = pat; *g; ) {
		bool group_ok = true, any_tok = false;

		while (*g && *g != ';' && *g != '+') {
			int k = 0;

			while (*g && *g != ',' && *g != ';' && *g != '+' && k < (int)sizeof(tok) - 1)
				tok[k++] = *g++;
			tok[k] = 0;
			if (*g == ',')
				g++;
			if (!k)
				continue;
			any_tok = true;
			if (!strnstr(buf, tok, n)) {
				group_ok = false;

				while (*g && *g != ';' && *g != '+')
					g++;
				break;
			}
		}
		if (*g == ';' || *g == '+')
			g++;
		if (group_ok && any_tok)
			return true;
	}

	return false;
}

static bool sl_chunk_matches(const char *s, unsigned int count)
{
	char buf[512];
	unsigned int n = min_t(unsigned int, count, sizeof(buf) - 1);

	memcpy(buf, s, n);
	buf[n] = 0;

	if (strnstr(buf, "GPU SMMU", n))
		return false;
	if (sl_pattern_match(sl_drop_default, buf, n))
		return true;
	if (sl_drop && sl_pattern_match(sl_drop, buf, n))
		return true;

	return false;
}

/* Keep fatal exception stacks even when routine stack dumps are disabled. */
static bool sl_drop_panic_trace(const char *message, unsigned int count)
{
	if (sl_bootdump || sl_first_die[0])
		return false;
	if (strnstr(message, "Call trace:", count)) {
		sl_in_trace = true;
		return true;
	}
	if (sl_in_trace && strnstr(message, "+0x", count))
		return true;
	sl_in_trace = false;
	return false;
}

#ifdef CONFIG_BLOCK

static int sl_dump_one_blk(struct device *dev, void *data)
{
	struct block_device *part;
	struct gendisk *disk;
	unsigned long idx;

	if (!dev->type || !dev->type->name || strcmp(dev->type->name, "disk"))
		return 0;

	disk = dev_to_disk(dev);
	pr_emerg("splashlog: blk %-12s major=%d minors=%d size=%lluKiB prot=%d\n",
		 disk->disk_name, disk->major, disk->minors,
		 (unsigned long long)get_capacity(disk) >> 1,
		 disk->part0->bd_holder ? 1 : 0);

	xa_for_each(&disk->part_tbl, idx, part) {
		if (!bdev_is_partition(part))
			continue;

		pr_emerg("splashlog: blk   %s%u partno=%lu size=%lluKiB\n",
			 disk->disk_name, (unsigned int)idx, idx,
			 (unsigned long long)bdev_nr_sectors(part) >> 1);
	}

	return 0;
}

static void sl_dump_blkdevs(void)
{
	pr_emerg("splashlog: ---- block devices at fault time ----\n");
	class_for_each_device(&block_class, NULL, NULL, sl_dump_one_blk);
	pr_emerg("splashlog: ---- end of block devices ----\n");
}
#else
static void sl_dump_blkdevs(void) { }
#endif

struct sl_user_fault {
	char executable[80];
	char line[384];
	int signal, code, syscall;
	bool handled;
	unsigned int count;
};

static DEFINE_RAW_SPINLOCK(sl_service_lock);
static char sl_service_snapshot[8192] = "BOOTWAIT not sampled\n";

static void sl_worker_sample(void)
{
	static struct {
		pid_t pid;
		u64 runtime;
	} previous[256];
	struct task_struct *workers[8], *group, *task;
	u64 deltas[8], delta, runtime;
	unsigned int count = 0, index, slot;
	char comm[TASK_COMM_LEN];

	rcu_read_lock();
	for_each_process_thread(group, task) {
		if (!(task->flags & PF_WQ_WORKER))
			continue;
		slot = (unsigned int)task->pid % ARRAY_SIZE(previous);
		runtime = READ_ONCE(task->se.sum_exec_runtime);
		delta = previous[slot].pid == task->pid &&
			runtime >= previous[slot].runtime ? runtime - previous[slot].runtime : 0;
		previous[slot].pid = task->pid;
		previous[slot].runtime = runtime;
		if (delta < NSEC_PER_SEC || count >= ARRAY_SIZE(workers))
			continue;
		get_task_struct(task);
		workers[count] = task;
		deltas[count++] = delta;
	}
	rcu_read_unlock();
	for (index = 0; index < count; index++) {
		task = workers[index];
		get_task_comm(comm, task);
		pr_info("splashlog: BOOTWAIT busy-worker tid=%d comm=%s state=%c cpu=%u runtime_delta=%llums\n",
			task->pid, comm, task_state_to_char(task), task_cpu(task),
			div_u64(deltas[index], NSEC_PER_MSEC));
		print_worker_info(KERN_INFO, task);
		put_task_struct(task);
	}
}

static void sl_service_sample(struct work_struct *work)
{
	struct task_struct *candidates[64], *threads[8], *blocked[8], *group, *task;
	unsigned int count = 0, index, thread_count, thread_index, shown = 0;
	unsigned int blocked_count = 0;
	unsigned long flags;
	char executable[80], comm[TASK_COMM_LEN];
	char *cursor, *end;
	static char snapshot[sizeof(sl_service_snapshot)];
	int length;

	if (!sl_enabled || sl_bootdump < 2 || sl_dead || sl_halted)
		return;
	sl_worker_sample();
	length = scnprintf(snapshot, sizeof(snapshot), "BOOTWAIT sample=%lus\n",
			   (jiffies - INITIAL_JIFFIES) / HZ);
	rcu_read_lock();
	for_each_process(group) {
		char parent_comm[TASK_COMM_LEN];

		if (group->flags & PF_KTHREAD)
			continue;
		get_task_comm(comm, group);
		get_task_comm(parent_comm, rcu_dereference(group->real_parent));
		if (strcmp(comm, "system_server") && strcmp(comm, "surfaceflinger") &&
		    strcmp(comm, "ndroid.systemui") && strncmp(comm, "boot-", 5) &&
		    strncmp(parent_comm, "boot-", 5) && strcmp(comm, "dmesg") &&
		    strcmp(comm, "logcat") && strcmp(comm, "audioserver") &&
		    strncmp(comm, "android.hardwar", 14) &&
		    strncmp(comm, "vendor.qti.hard", 14))
			continue;
		get_task_struct(group);
		candidates[count++] = group;
		if (count == ARRAY_SIZE(candidates))
			break;
	}
	rcu_read_unlock();
	for (index = 0; index < count; index++) {
		struct file *exe;
		bool framework;

		group = candidates[index];
		exe = get_task_exe_file(group);
		if (!exe)
			goto next;
		spin_lock(&exe->f_path.dentry->d_lock);
		strscpy(executable, exe->f_path.dentry->d_name.name, sizeof(executable));
		spin_unlock(&exe->f_path.dentry->d_lock);
		fput(exe);
		framework = !strcmp(executable, "app_process64");
		if (!strstr(executable, "sensors-service") && !framework &&
		    strcmp(executable, "busybox") && strcmp(executable, "toybox") &&
		    strcmp(executable, "sh") && strcmp(executable, "logcat") &&
		    strcmp(executable, "dmesg") && strcmp(executable, "timeout") &&
		    strcmp(executable, "dd") &&
		    strcmp(executable, "debuggerd") &&
		    strcmp(executable, "audioserver") &&
		    strcmp(executable, "surfaceflinger") && !strstr(executable, "composer"))
			goto next;
		thread_count = 1;
		get_task_struct(group);
		threads[0] = group;
		if (framework) {
			rcu_read_lock();
			for_each_thread(group, task) {
				if (task == group)
					continue;
				get_task_comm(comm, task);
				if (strcmp(comm, "android.ui") && strcmp(comm, "android.display") &&
				    strcmp(comm, "android.fg") && strcmp(comm, "RenderThread"))
					continue;
				get_task_struct(task);
				threads[thread_count++] = task;
				if (thread_count == ARRAY_SIZE(threads))
					break;
			}
			rcu_read_unlock();
		}
		for (thread_index = 0; thread_index < thread_count; thread_index++) {
			struct mm_struct *mm;
			struct vm_area_struct *vma;
			struct pt_regs *regs;
			unsigned long pc = 0, offset = 0, argument = 0, wait;
			char image[64] = "?";
			int syscall = -1;
			void *stack;

			task = threads[thread_index];
			if (shown >= 24)
				goto next_thread;
			get_task_comm(comm, task);
			stack = try_get_task_stack(task);
			if (stack) {
				regs = task_pt_regs(task);
				pc = READ_ONCE(regs->pc);
				syscall = READ_ONCE(regs->syscallno);
				argument = READ_ONCE(regs->orig_x0);
				put_task_stack(task);
			}
			wait = get_wchan(task);
			mm = get_task_mm(task);
			if (mm) {
				if (mmap_read_trylock(mm)) {
					vma = vma_lookup(mm, pc);
					if (vma && vma->vm_file) {
						struct dentry *dentry = vma->vm_file->f_path.dentry;

						spin_lock(&dentry->d_lock);
						strscpy(image, dentry->d_name.name, sizeof(image));
						spin_unlock(&dentry->d_lock);
						offset = pc - vma->vm_start + (vma->vm_pgoff << PAGE_SHIFT);
					}
					mmap_read_unlock(mm);
				}
				mmput(mm);
			}
			length += scnprintf(snapshot + length, sizeof(snapshot) - length,
				"BOOTWAIT %.47s %d/%d %s %c exec=%u ut=%llu sys=%d arg0=%lx wait=%ps pc=%lx %.63s+%lx frozen=%u/%u\n",
				executable, group->pid, task->pid, comm,
				task_state_to_char(task), task->in_execve,
				(unsigned long long)READ_ONCE(task->utime), syscall, argument, (void *)wait,
				pc, image, offset, freezing(task), cgroup_freezing(task));
			if (READ_ONCE(task->__state) & TASK_UNINTERRUPTIBLE) {
				unsigned long entries[8];
				unsigned int frames, frame;
				struct file *file = NULL;

				if (syscall == 64 || syscall == 66 || syscall == 67 ||
				    syscall == 68 || syscall == 70 || syscall == 82 || syscall == 83)
					file = fget_task(task, argument);
				if (file) {
					char pathbuf[256], *path;

					path = d_path(&file->f_path, pathbuf, sizeof(pathbuf));
					pr_info("splashlog: BOOTWAIT file pid=%d sys=%d fd=%lu file=%s ino=%lu dev=%u:%u err=%d pages=%lu dirty=%u writeback=%u\n",
						task->pid, syscall, argument, IS_ERR(path) ? "?" : path,
						file_inode(file)->i_ino,
						MAJOR(file_inode(file)->i_sb->s_dev),
						MINOR(file_inode(file)->i_sb->s_dev),
						READ_ONCE(file->f_mapping->wb_err),
						READ_ONCE(file->f_mapping->nrpages),
						mapping_tagged(file->f_mapping, PAGECACHE_TAG_DIRTY),
						mapping_tagged(file->f_mapping, PAGECACHE_TAG_WRITEBACK));
					fput(file);
				}

				frames = stack_trace_save_tsk(task, entries,
							     ARRAY_SIZE(entries), 0);
				for (frame = 0; frame < frames; frame++)
					pr_info("splashlog: BOOTWAIT blocked %d %pS\n",
							task->pid, (void *)entries[frame]);
			}
			shown++;
next_thread:
			put_task_struct(task);
		}
next:
		put_task_struct(group);
	}
	rcu_read_lock();
	for_each_process_thread(group, task) {
		unsigned int state = READ_ONCE(task->__state);

		if (!(state & TASK_UNINTERRUPTIBLE) || (state & TASK_NOLOAD))
			continue;
		if (blocked_count >= ARRAY_SIZE(blocked))
			goto blocked_collected;
		get_task_struct(task);
		blocked[blocked_count++] = task;
	}
blocked_collected:
	rcu_read_unlock();
	for (index = 0; index < blocked_count; index++) {
		unsigned long entries[8];
		unsigned int frames, frame;

		task = blocked[index];
		get_task_comm(comm, task);
		pr_info("splashlog: BOOTWAIT IO tid=%d comm=%s wait=%ps\n",
			task->pid, comm, (void *)get_wchan(task));
		print_worker_info(KERN_INFO, task);
		frames = stack_trace_save_tsk(task, entries, ARRAY_SIZE(entries), 0);
		for (frame = 0; frame < frames; frame++)
			pr_info("splashlog: BOOTWAIT IO %d %pS\n",
				task->pid, (void *)entries[frame]);
		put_task_struct(task);
	}
	if (!shown)
		scnprintf(snapshot + length, sizeof(snapshot) - length, "BOOTWAIT no target tasks\n");
	raw_spin_lock_irqsave(&sl_service_lock, flags);
	strscpy(sl_service_snapshot, snapshot, sizeof(sl_service_snapshot));
	raw_spin_unlock_irqrestore(&sl_service_lock, flags);
	for (cursor = snapshot; *cursor; cursor = end + 1) {
		end = strchr(cursor, '\n');
		if (!end) {
			pr_info("splashlog: %s\n", cursor);
			break;
		}
		pr_info("splashlog: %.*s\n", (int)(end - cursor), cursor);
	}
}
static DECLARE_WORK(sl_service_work, sl_service_sample);

static void sl_dump_service_snapshot(void)
{
	static char snapshot[sizeof(sl_service_snapshot)];
	unsigned long flags;

	if (!raw_spin_trylock_irqsave(&sl_service_lock, flags))
		return;
	strscpy(snapshot, sl_service_snapshot, sizeof(snapshot));
	raw_spin_unlock_irqrestore(&sl_service_lock, flags);
	sl_puts(snapshot, true);
}

static DEFINE_RAW_SPINLOCK(sl_user_fault_lock);
static struct sl_user_fault sl_user_faults[32];
static unsigned int sl_user_fault_next;

void splashlog_user_fault(const struct kernel_siginfo *info, bool handled)
{
	struct sl_user_fault *entry = NULL;
	struct pt_regs *regs = current_pt_regs();
	struct file *exe;
	struct vm_area_struct *vma;
	unsigned long flags, offset = 0, address = 0;
	unsigned int index;
	bool fresh = false;
	char executable[80] = "?", image[80] = "?", line[384];
	int syscall = -1;

	if (!sl_enabled || !sl_bootdump || sl_dead || !current->mm)
		return;
	exe = get_task_exe_file(current);
	if (exe) {
		spin_lock(&exe->f_path.dentry->d_lock);
		strscpy(executable, exe->f_path.dentry->d_name.name, sizeof(executable));
		spin_unlock(&exe->f_path.dentry->d_lock);
		fput(exe);
	}
	if (info->si_signo == SIGSYS && info->si_code == SYS_SECCOMP)
		syscall = info->si_syscall;
	if (info->si_code > 0 &&
	    (info->si_signo == SIGSEGV || info->si_signo == SIGBUS ||
	     info->si_signo == SIGILL || info->si_signo == SIGFPE ||
	     info->si_signo == SIGTRAP))
		address = (unsigned long)info->si_addr;
	if (mmap_read_trylock(current->mm)) {
		vma = vma_lookup(current->mm, instruction_pointer(regs));
		if (vma && vma->vm_file) {
			struct dentry *dentry = vma->vm_file->f_path.dentry;

			spin_lock(&dentry->d_lock);
			strscpy(image, dentry->d_name.name, sizeof(image));
			spin_unlock(&dentry->d_lock);
			offset = instruction_pointer(regs) - vma->vm_start +
				 (vma->vm_pgoff << PAGE_SHIFT);
		}
		mmap_read_unlock(current->mm);
	}
	scnprintf(line, sizeof(line),
		  "USERFAULT t=%lus exe=%.79s pid=%d/%d sig=%d code=%d syscall=%d arch=%x addr=%lx pc=%lx %.79s+%lx stage=%s\n",
		  (jiffies - INITIAL_JIFFIES) / HZ, executable,
		  current->tgid, current->pid, info->si_signo, info->si_code,
		  syscall, syscall >= 0 ? info->si_arch : 0,
		  address, instruction_pointer(regs), image, offset,
		  handled ? "handler" : "fatal");
	raw_spin_lock_irqsave(&sl_user_fault_lock, flags);
	for (index = 0; index < ARRAY_SIZE(sl_user_faults); index++) {
		entry = &sl_user_faults[index];
		if (entry->count && !strcmp(entry->executable, executable) &&
		    entry->signal == info->si_signo && entry->code == info->si_code &&
		    entry->syscall == syscall && entry->handled == handled)
			break;
	}
	if (index == ARRAY_SIZE(sl_user_faults)) {
		entry = &sl_user_faults[sl_user_fault_next++ % ARRAY_SIZE(sl_user_faults)];
		strscpy(entry->executable, executable, sizeof(entry->executable));
		entry->signal = info->si_signo;
		entry->code = info->si_code;
		entry->syscall = syscall;
		entry->handled = handled;
		entry->count = 0;
		fresh = true;
	}
	entry->count++;
	if (fresh)
		strscpy(entry->line, line, sizeof(entry->line));
	raw_spin_unlock_irqrestore(&sl_user_fault_lock, flags);
	if (fresh)
		pr_err("splashlog: %s", line);
}

static DEFINE_RAW_SPINLOCK(sl_display_lock);
static char sl_display_events[6][256];
static unsigned int sl_display_next;

static void sl_capture_display(const char *message, unsigned int count)
{
	unsigned long flags;
	char line[256];
	size_t length;

	scnprintf(line, sizeof(line), "%.*s", (int)min_t(unsigned int, count, sizeof(line) - 1), message);
	if (!strstr(line, "DISPLAY POWER") && !strstr(line, "[BACKLIGHT]") &&
	    !strstr(line, "DISPLAY DIAG post-enable end"))
		return;
	length = strlen(line);
	if (length && line[length - 1] == '\n')
		line[length - 1] = 0;
	raw_spin_lock_irqsave(&sl_display_lock, flags);
	strscpy(sl_display_events[sl_display_next++ % ARRAY_SIZE(sl_display_events)],
		line, sizeof(line));
	raw_spin_unlock_irqrestore(&sl_display_lock, flags);
}

static void sl_dump_display_events(void)
{
	static char events[ARRAY_SIZE(sl_display_events)][256];
	unsigned long flags;
	unsigned int next, count, index;

	if (!raw_spin_trylock_irqsave(&sl_display_lock, flags))
		return;
	memcpy(events, sl_display_events, sizeof(events));
	next = sl_display_next;
	raw_spin_unlock_irqrestore(&sl_display_lock, flags);
	count = min_t(unsigned int, next, ARRAY_SIZE(events));
	for (index = next - count; index < next; index++) {
		sl_puts("splashlog: display-tail ", true);
		sl_puts(events[index % ARRAY_SIZE(events)], true);
		sl_puts("\n", true);
	}
}

static void sl_dump_user_faults(void)
{
	static struct sl_user_fault snapshot[ARRAY_SIZE(sl_user_faults)];
	unsigned long flags;
	unsigned int index, pass, shown = 0;
	char prefix[48];

	if (!raw_spin_trylock_irqsave(&sl_user_fault_lock, flags))
		return;
	memcpy(snapshot, sl_user_faults, sizeof(snapshot));
	raw_spin_unlock_irqrestore(&sl_user_fault_lock, flags);
	for (pass = 0; pass < 3 && shown < 8; pass++) {
		for (index = 0; index < ARRAY_SIZE(snapshot) && shown < 8; index++) {
			const char *exe = snapshot[index].executable;
			bool critical = !strcmp(exe, "surfaceflinger") ||
				!strcmp(exe, "zygote64") || !strcmp(exe, "zygote") ||
				!strcmp(exe, "servicemanager") || !strcmp(exe, "system_server") ||
				!strcmp(exe, "vold") || !strcmp(exe, "keystore2");
			unsigned int priority = critical ? 0 : snapshot[index].handled ? 1 : 2;

			if (!snapshot[index].count || priority != pass)
				continue;
			scnprintf(prefix, sizeof(prefix), "splashlog: crash x%u ", snapshot[index].count);
			sl_puts(prefix, true);
			sl_puts(snapshot[index].line, true);
			shown++;
		}
	}
	sl_dump_display_events();
}

static void sl_dump_procs(void)
{

	static char out[900];
	char comm[48];
	struct task_struct *pt;
	int len, n, total = 0, kth = 0;

	len = scnprintf(out, sizeof(out), "splashlog: key procs: ");
	n = 0;
	kth = 0;
	rcu_read_lock();
	for_each_process(pt) {
		if (pt->flags & PF_KTHREAD) {
			kth++;
			continue;
		}
		total++;
		if (n >= 16)
			continue;
		len += scnprintf(out + len, sizeof(out) - len, "%s(%d) ",
				 pt->comm, pt->pid);
		n++;
	}
	rcu_read_unlock();
	scnprintf(comm, sizeof(comm), "[user=%d%s kthread=%d]\n", total,
		  total > n ? ", first 16" : "", kth);
	if (len < (int)sizeof(out) - 24)
		len += scnprintf(out + len, sizeof(out) - len, "%s", comm);
	else
		out[len - 2] = '\n', out[len - 1] = 0;
	sl_puts(out, true);
}

static void sl_dump_tasklist(void);
static void sl_dump_blocked_stacks(void);

static void sl_dump_init_threads(void)
{
	struct task_struct *group, *task;
	unsigned int shown = 0;
	char line[256], comm[TASK_COMM_LEN];

	rcu_read_lock();
	for_each_process_thread(group, task) {
		struct pt_regs *regs;
		unsigned long entries[6];
		unsigned int count = 0, index;
		void *stack;

		if (group->pid != 1)
			continue;
		if (shown++ >= 16)
			break;
		get_task_comm(comm, task);
		stack = try_get_task_stack(task);
		regs = stack ? task_pt_regs(task) : NULL;
		scnprintf(line, sizeof(line),
			  "INITTHR pid=%d tid=%d comm=%s state=%c stack=%u",
			  group->pid, task->pid, comm, task_state_to_char(task),
			  stack != NULL);
		if (regs) {
			if (regs->syscallno != NO_SYSCALL) {
				scnprintf(line + strlen(line), sizeof(line) - strlen(line),
					  " pc=%llx lr=%llx syscall=%d x0=%llx"
					  " x1=%llx x2=%llx x3=%llx\n",
					  regs->pc, regs->regs[30], regs->syscallno,
					  regs->regs[0], regs->regs[1], regs->regs[2],
					  regs->regs[3]);
			} else {
				scnprintf(line + strlen(line), sizeof(line) - strlen(line),
					  " pc=%llx lr=%llx syscall=%d\n",
					  regs->pc, regs->regs[30], regs->syscallno);
			}
		} else {
			scnprintf(line + strlen(line), sizeof(line) - strlen(line),
				  "\n");
		}
		sl_puts(line, true);
		printk_deferred(KERN_EMERG "%s", line);
		if (stack && !task_is_running(task))
			count = stack_trace_save_tsk(task, entries,
						     ARRAY_SIZE(entries), 0);
		put_task_stack(task);
		for (index = 0; index < count; index++) {
			scnprintf(line, sizeof(line), "  %pS\n",
				  (void *)entries[index]);
			sl_puts(line, true);
			printk_deferred(KERN_EMERG "%s", line);
		}
	}
	rcu_read_unlock();
	if (!shown) {
		sl_puts("INITTHR none\n", true);
		printk_deferred(KERN_EMERG "INITTHR none\n");
	}
}

static void sl_fault_hold_fn(struct work_struct *w)
{
	char buf[160];

	WRITE_ONCE(sl_fault_phase, 2);
	sl_bb_frozen = false;
	scnprintf(buf, sizeof(buf),
		      "splashlog: FAULT worker entered why=%s uptime=%lus\n",
		      sl_fault_why, (jiffies - INITIAL_JIFFIES) / HZ);
	sl_puts(buf, true);
	if (READ_ONCE(sl_watchdog_fault)) {
		if (sl_bootdump >= 1) {
			sl_evidence_dump();
			sl_dump_modring();
			sl_dump_procs();
		}
		if (sl_bootdump >= 2) {
			sl_dump_blocked_stacks();

			sl_dump_init_threads();
		}
		if (sl_deadline_panic) {
			panic("splashlog diagnostic deadline %ums",
			      sl_deadline_ms);
		}
		if (sl_bootdump >= 2)
			sl_dump_tasklist();
		if (sl_bootdump >= 1)
			sl_dump_user_faults();
		goto restart;
	}
	if (sl_bootdump >= 2)
		sl_dump_blocked_stacks();

	sl_bb_frozen = false;
	if (sl_blkdump)
		sl_dump_blkdevs();

	if (sl_bootdump >= 1) {
		sl_evidence_dump();
		sl_dump_procs();
		sl_dump_modring();
	}

restart:
	WRITE_ONCE(sl_fault_phase, 3);
	scnprintf(buf, sizeof(buf),
		      "splashlog: ==== FAULT ====\n"
		      "splashlog: why=%s stage=%u uptime=%lus\n",
		      sl_fault_why, sl_stage,
		      (jiffies - INITIAL_JIFFIES) / HZ);
	sl_puts(buf, true);
	if (READ_ONCE(sl_watchdog_fault)) {
		WRITE_ONCE(sl_fault_phase, 4);
		sl_bb_frozen = true;
		emergency_restart();
		for (;;)
			cpu_relax();
	}

	pr_emerg("splashlog: ==== FAULT ====\n");

	pr_emerg("splashlog: why=%s stage=%u uptime=%lus\n",
		 sl_fault_why, sl_stage,
		 (jiffies - INITIAL_JIFFIES) / HZ);
	pr_emerg("splashlog: tainted=%s\n", print_tainted());
	pr_emerg("splashlog: => warm restart (reason written, box frozen)\n");
	sl_bb_frozen = true;
	emergency_restart();
	for (;;)
		cpu_relax();
}
static DECLARE_WORK(sl_fault_work, sl_fault_hold_fn);

static int __init splashlog_replay_at_setup(char *str)
{
	unsigned int v;

	if (!str)
		return -EINVAL;

	if (!kstrtouint(str, 0, &v) && v <= 3)
		sl_replay_at = v;
	return 0;
}
early_param("splashlog.replay_at", splashlog_replay_at_setup);

struct sl_bb_hdr {
	u32 sig;
	s32 start;
	s32 size;
	u8 data[];
};

struct sl_bb_canary {
	u32 magic;
	u32 boots;
	u32 panic;
	u32 rsvd;
	u32 hdr_sig;
	s32 hdr_start;
	s32 hdr_size;
	u32 rsvd2;
};

static void sl_bb_clean(void *p, size_t len)
{
	dcache_clean_poc((unsigned long)p, (unsigned long)p + len);
}

static struct sl_bb_canary *sl_canary(void)
{
	return (struct sl_bb_canary *)(sl_bb + SL_CANARY_OFF);
}

static bool sl_canary_window_ok(const struct sl_bb_canary *c)
{
	if (c->hdr_sig != SL_BB_MAGIC)
		return false;
	if (c->hdr_size <= 0 || c->hdr_size > (s32)sl_bb_room)
		return false;
	if (c->hdr_start < 0 || c->hdr_start > c->hdr_size)
		return false;
	return true;
}

static bool sl_bb_looks_text(s32 start, s32 size)
{
	struct sl_bb_hdr *h = (struct sl_bb_hdr *)sl_bb;
	unsigned int off, n, i, good = 0;

	n = min_t(unsigned int, (unsigned int)size, 1024);
	if (!n || n > sl_bb_room)
		return false;
	off = ((unsigned int)start + sl_bb_room - n) % sl_bb_room;
	for (i = 0; i < n; i++) {
		u8 c = h->data[(off + i) % sl_bb_room];

		if (c == '\n' || c == '\r' || c == '\t' ||
		    (c >= 0x20 && c < 0x7f))
			good++;
	}
	return good * 2 >= n;
}

static void sl_canary_mark(bool panic)
{
	struct sl_bb_canary *c;
	struct sl_bb_hdr *h;

	if (!sl_bb_ok)
		return;

	c = sl_canary();
	h = (struct sl_bb_hdr *)sl_bb;
	c->magic = SL_CANARY_MAGIC;
	c->boots = sl_boots;
	c->panic = panic ? 1 : 0;
	c->rsvd = 0;

	c->hdr_sig = panic ? h->sig : 0;
	c->hdr_start = panic ? h->start : 0;
	c->hdr_size = panic ? h->size : 0;
	c->rsvd2 = 0;
	sl_bb_clean(c, sizeof(*c));
}

struct sl_spare {
	u32 magic;
	u32 boots;
};

static struct sl_spare *sl_spare(void)
{
	return (struct sl_spare *)(sl_fb + SL_SPARE_OFF);
}

static void sl_spare_probe(void)
{
	struct sl_spare *s;
	phys_addr_t pa = sl_phys + SL_SPARE_OFF;

	if (!sl_ready)
		return;

	if (!memblock_is_reserved(pa) ||
	    !memblock_is_reserved(pa + sizeof(struct sl_spare) - 1))
		return;

	s = sl_spare();
	if (s->magic == SL_SPARE_MAGIC && s->boots && s->boots < 0xffff)
		sl_spare_boots = s->boots + 1;
	else
		sl_spare_boots = 1;

	s->magic = SL_SPARE_MAGIC;
	s->boots = sl_spare_boots;
	sl_bb_clean(s, sizeof(*s));
}

static bool sl_bb_setup(void)
{
	struct sl_bb_hdr *h;
	struct sl_bb_canary *c;

	sl_bb = (u8 *)__va(sl_bb_phys);
	if (!sl_bb)
		return false;

	h = (struct sl_bb_hdr *)sl_bb;
	sl_bb_room = SL_CANARY_OFF - sizeof(*h);

	c = sl_canary();
	if (c->magic == SL_CANARY_MAGIC && c->boots && c->boots < 0xffff) {
		sl_boots = c->boots + 1;
		sl_prev_panic = c->panic;
		sl_prev_sig = c->hdr_sig;
		sl_prev_start = c->hdr_start;
		sl_prev_size = c->hdr_size;
	} else {
		sl_boots = 1;
		sl_prev_panic = 0;
		sl_prev_sig = 0;
		sl_prev_start = 0;
		sl_prev_size = 0;
	}

	if (h->sig == SL_BB_MAGIC &&
	    h->size > 0 && h->size <= (s32)sl_bb_room &&
	    h->start >= 0 && h->start <= h->size) {
		sl_bb_ok = true;
		sl_canary_mark(false);
		return true;
	}

	if (sl_prev_panic && sl_canary_window_ok(c) &&
	    sl_bb_looks_text(c->hdr_start, c->hdr_size)) {
		h->sig = SL_BB_MAGIC;
		h->start = c->hdr_start;
		h->size = c->hdr_size;
		sl_bb_clean(h, sizeof(*h));
		sl_bb_ok = true;
		sl_canary_mark(false);
		return true;
	}

	sl_bb_ok = true;
	sl_canary_mark(false);

	h->sig = 0;
	h->start = 0;
	h->size = 0;
	sl_bb_clean(h, sizeof(*h));
	h->sig = SL_BB_MAGIC;
	sl_bb_clean(&h->sig, sizeof(h->sig));
	return false;
}

static void sl_bb_reset(void)
{
	struct sl_bb_hdr *h = (struct sl_bb_hdr *)sl_bb;

	if (!sl_bb_ok)
		return;
	h->start = 0;
	h->size = 0;
	sl_bb_clean(h, sizeof(*h));
}

/* Caller holds sl_lock; clean payload before publishing size and cursor. */
static void sl_bb_write_locked(const char *s, unsigned int count)
{
	struct sl_bb_hdr *h = (struct sl_bb_hdr *)sl_bb;
	unsigned int start, n, rem;
	size_t size;

	if (!sl_bb_ok || !count || sl_bb_frozen)
		return;

	if (count > sl_bb_room) {
		s += count - sl_bb_room;
		count = sl_bb_room;
	}

	size = h->size;
	if (size < sl_bb_room) {
		size += count;
		if (size > sl_bb_room)
			size = sl_bb_room;
	}

	start = h->start;
	n = start + count;
	while (n >= sl_bb_room)
		n -= sl_bb_room;

	rem = sl_bb_room - start;
	if (rem < count) {
		memcpy(h->data + start, s, rem);
		memcpy(h->data, s + rem, count - rem);
		sl_bb_clean(h->data + start, rem);
		sl_bb_clean(h->data, count - rem);
	} else {
		memcpy(h->data + start, s, count);
		sl_bb_clean(h->data + start, count);
	}

	h->size = size;
	sl_bb_clean(&h->size, sizeof(h->size));
	h->start = n;
	sl_bb_clean(&h->start, sizeof(h->start));

	if (h->sig != SL_BB_MAGIC) {
		h->sig = SL_BB_MAGIC;
		sl_bb_clean(&h->sig, sizeof(h->sig));
	}
}

static void sl_rp_line(const char *s, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		char c = s[i];

		if (c == '\r')
			continue;
		if (c == '\n') {

			if (sl_rp_rows >= SL_RP_ROWS) {
				memmove(sl_rp, sl_rp + 1,
					sizeof(sl_rp[0]) * (SL_RP_ROWS - 1));
				sl_rp_rows = SL_RP_ROWS - 1;
			}
			sl_rp_cur[sl_rp_curlen] = '\0';
			memcpy(sl_rp[sl_rp_rows], sl_rp_cur, sl_rp_curlen + 1);
			sl_rp_rows++;
			sl_rp_curlen = 0;
			continue;
		}
		if (c < 0x20 || c >= 0x7f)
			continue;
		if (sl_rp_curlen < SL_LINELEN - 1)
			sl_rp_cur[sl_rp_curlen++] = c;
	}
}

static void sl_bb_replay(void)
{
	struct sl_bb_hdr *h = (struct sl_bb_hdr *)sl_bb;
	unsigned int len, total, off, tail;
	const char *sep;

	if (!sl_bb_ok || h->sig != SL_BB_MAGIC || h->size <= 0)
		return;

	sl_rp_rows = 0;
	sl_rp_curlen = 0;

	total = h->size;
	if (total > sl_bb_room)
		total = sl_bb_room;
	len = total > SL_BB_REPLAY ? SL_BB_REPLAY : total;
	sl_bb_bytes = len;

	sep = "########## PREVIOUS BOOT, tail before blackout ##########";
	sl_rp_line(sep, strlen(sep));
	sl_rp_line("\n", 1);

	off = (h->start + sl_bb_room - len) % sl_bb_room;
	tail = sl_bb_room - off;
	if (tail >= len) {
		sl_rp_line((const char *)h->data + off, len);
	} else {
		sl_rp_line((const char *)h->data + off, tail);
		sl_rp_line((const char *)h->data, len - tail);
	}

	if (sl_rp_curlen)
		sl_rp_line("\n", 1);
	sep = "############ end of black box ############";
	sl_rp_line(sep, strlen(sep));
	sl_rp_line("\n", 1);
}

static void sl_rp_show(void)
{
	unsigned long flags;

	if (!sl_ready)
		return;

	if (!sl_rp_rows && !sl_rp_full)
		return;

	raw_spin_lock_irqsave(&sl_lock, flags);
	if (sl_rp_full) {

		sl_rp_drawn = true;
		sl_force = true;
		sl_render_locked(false);
	} else {
		sl_rp_render_locked(true);
	}
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_halt_forever(void)
{
	unsigned long flags;

	sl_halted = true;

	pr_emerg("splashlog: HALTED after panic -- long-press power to restart\n");

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_rp_full = true;
	sl_rp_hold = true;
	sl_rp_drawn = true;
	sl_force = true;
	sl_render_locked(false);
	/* Keep the replay frame and its backing evidence stable while PID 1 sleeps. */
	sl_bb_frozen = true;
	sl_fb_frozen = true;
	raw_spin_unlock_irqrestore(&sl_lock, flags);

	for (;;) {
		touch_softlockup_watchdog();
		schedule_timeout_interruptible(HZ);
	}
}

static bool sl_buffer_overlaps(phys_addr_t base, size_t size,
			       phys_addr_t owned_base, size_t owned_size)
{
	if (!size || !owned_size)
		return false;
	if (base < owned_base)
		return owned_base - base < size;
	return base - owned_base < owned_size;
}

bool splashlog_buffer_in_use(phys_addr_t base, size_t size)
{
	if (!sl_enabled)
		return false;
	return (sl_ready && sl_buffer_overlaps(base, size, sl_phys, sl_size)) ||
	       (sl_bb_ok && sl_buffer_overlaps(base, size, sl_bb_phys, SL_BB_SIZE));
}
EXPORT_SYMBOL_GPL(splashlog_buffer_in_use);

void splashlog_display_takeover(void)
{
	unsigned long flags;

	if (!sl_enabled || READ_ONCE(sl_native_display))
		return;

	raw_spin_lock_irqsave(&sl_lock, flags);
	if (sl_native_display || sl_dead || sl_first_die[0]) {
		raw_spin_unlock_irqrestore(&sl_lock, flags);
		return;
	}
	sl_native_display = true;
	sl_fb_frozen = true;
	raw_spin_unlock_irqrestore(&sl_lock, flags);

	if (sl_timer_on)
		timer_shutdown_sync(&sl_timer);
	if (sl_rtimer_ok)
		timer_shutdown_sync(&sl_rtimer);
	pr_info("splashlog: native display owns scanout; framebuffer rendering stopped, blackbox recording continues\n");
}
EXPORT_SYMBOL_GPL(splashlog_display_takeover);

static void sl_fault_display(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sl_lock, flags);
	if (sl_native_display && sl_fb_mode != 3) {
		sl_fb_frozen = false;
		sl_force = true;
		sl_dirty = true;
	}
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_rect(unsigned int x, unsigned int y, unsigned int w,
		    unsigned int h, u32 c)
{
	unsigned int i, j;

	if (!sl_fb || sl_fb_frozen || x >= sl_w || y >= sl_h)
		return;
	if (x + w > sl_w)
		w = sl_w - x;
	if (y + h > sl_h)
		h = sl_h - y;
	for (j = 0; j < h; j++) {
		u32 *p = (u32 *)(sl_fb + (size_t)(y + j) * sl_stride) + x;

		for (i = 0; i < w; i++)
			p[i] = c;
	}
}

static void sl_glyph_s(unsigned int px, unsigned int py, unsigned char ch,
		       u32 fg, u32 bg, unsigned int scale_x2)
{
	const u8 *g = (const u8 *)font_vga_8x16.data + (unsigned int)ch * 16;
	unsigned int r, c, dy, dx;

	if (sl_fb_frozen)
		return;

	for (r = 0; r < 16; r++) {
		u8 bits = g[r];
		unsigned int top = r * scale_x2 / 2;
		unsigned int bottom = (r + 1) * scale_x2 / 2;

		for (c = 0; c < 8; c++) {
			u32 col = (bits & (0x80 >> c)) ? fg : bg;
			unsigned int left = c * scale_x2 / 2;
			unsigned int right = (c + 1) * scale_x2 / 2;

			for (dy = top; dy < bottom; dy++)
				for (dx = left; dx < right; dx++)
					*(u32 *)(sl_fb +
						 (size_t)(py + dy) *
							sl_stride +
						 (size_t)(px + dx) * 4)
						= col;
		}
	}
}

static void sl_glyph(unsigned int px, unsigned int py, unsigned char ch,
		     u32 fg, u32 bg)
{
	sl_glyph_s(px, py, ch, fg, bg, sl_scale_x2);
}

static void sl_clean(unsigned int y0, unsigned int y1)
{
	unsigned int f;
	size_t off, len;

	if (!sl_fb || sl_fb_frozen || y1 <= y0)
		return;
	if (y1 > sl_h)
		y1 = sl_h;
	off = (size_t)y0 * sl_stride;
	len = (size_t)(y1 - y0) * sl_stride;

	for (f = 1; f < sl_frames; f++)
		memcpy(sl_fb + (size_t)f * sl_frame_bytes + off, sl_fb + off, len);

	for (f = 0; f < sl_frames; f++)
		dcache_clean_poc((unsigned long)(sl_fb + (size_t)f * sl_frame_bytes + off),
				 (unsigned long)(sl_fb + (size_t)f * sl_frame_bytes + off + len));
}

static void sl_bar_locked(void)
{
	sl_rect(0, 0, sl_w, SL_BAR_H, SL_BAR_BG);
	sl_rect(0, 0, sl_w * sl_stage / SL_STAGES, SL_BAR_H, SL_FG);
	if (!sl_quiet && sl_w > 64)
		sl_rect(sl_w - 40, 0, 28, SL_BAR_H,
			sl_flip ? SL_HB_ON : SL_HB_OFF);
	sl_clean(0, SL_BAR_H);
}

static void sl_status_locked(void)
{
	unsigned int y = sl_h - sl_ch, j, len;
	char row[SL_LINELEN];

	sl_rect(0, y, sl_w, sl_ch, SL_RP_BANNER);

	if (!sl_bootdump) {
		len = scnprintf(row, sizeof(row), " up=%lus log=%d %s",
				(jiffies - INITIAL_JIFFIES) / HZ,
				atomic_read(&sl_console_count),
				sl_flip ? "[ALIVE]" : "[     ]");
	} else {
		len = scnprintf(row, sizeof(row),
			" up=%lus log=%d rb=%llu cs=%llu age=%lus kage=%lus fault=%u drop=%u %s",
			(jiffies - INITIAL_JIFFIES) / HZ,
			atomic_read(&sl_console_count),
			READ_ONCE(sl_printk_seq), READ_ONCE(sl_delivery_seq),
			(jiffies - READ_ONCE(sl_last_console)) / HZ,
			(jiffies - READ_ONCE(sl_last_kbeat)) / HZ,
			READ_ONCE(sl_fault_phase), sl_drop_cnt,
			sl_flip ? "[ALIVE]" : "[     ]");
	}
	if (len > sl_cols)
		len = sl_cols;
	memset(row + len, ' ', sl_cols - len);
	for (j = 0; j < sl_cols; j++)
		sl_glyph(j * sl_cw, y, (unsigned char)row[j],
			 SL_RP_BANNER_FG, SL_RP_BANNER);

	sl_clean(y, sl_h);
}

static void sl_diag_locked(void)
{
	unsigned int y = sl_h - 2 * sl_ch, j, len;
	char row[SL_LINELEN];

	sl_rect(0, y, sl_w, sl_ch, SL_BAR_BG);

	len = scnprintf(row, sizeof(row),
			" boot=%u spare=%u panic=%u old=%u sig=%08x start=%x size=%x",
			sl_boots, sl_spare_boots, sl_prev_panic,
			sl_have_old ? 1 : 0, sl_prev_sig,
			sl_prev_start, sl_prev_size);
	if (len > sl_cols)
		len = sl_cols;
	memset(row + len, ' ', sl_cols - len);
	for (j = 0; j < sl_cols; j++)
		sl_glyph(j * sl_cw, y, (unsigned char)row[j], SL_FG, SL_BAR_BG);

	sl_clean(y, y + sl_ch);
}

static void sl_rp_render_locked(bool full)
{
	unsigned int avail, maxrows, shown, total, skip, v, py;
	unsigned int i, j, len, c, chunks, top, bottom;
	char row[SL_LINELEN];
	const char *banner = sl_rp_hold ?
		"*** HALTED after panic -- long-press power to restart ***" :
		"*** PREVIOUS BOOT -- splashlog black box ***";

	bottom = sl_h - (sl_bootdump ? 2 : 1) * sl_ch;

	top = sl_rp_full ? SL_BAR_H + SL_MARGIN : sl_rp_y;
	if (bottom <= top + 2 * sl_rp_ch)
		return;
	top &= ~1u;

	avail = (bottom - top - SL_MARGIN) / sl_rp_ch;
	maxrows = avail > 2 ? avail - 2 : 0;

	if (!full)
		return;

	sl_rect(0, top, sl_w, bottom - top, SL_RP_BG);

	memset(row, ' ', sl_rp_cols);
	len = min_t(size_t, strlen(banner), sl_rp_cols);
	memcpy(row, banner, len);
	for (j = 0; j < sl_rp_cols; j++)
		sl_glyph_s(j * sl_rp_cw, top, (unsigned char)row[j],
			   SL_RP_BANNER_FG, SL_RP_BANNER, SL_RP_SCALE * 2);

	total = 0;
	for (i = 0; i < sl_rp_rows; i++) {
		len = min(strlen(sl_rp[i]), (size_t)(SL_LINELEN - 1));
		total += sl_chunks(len, sl_rp_cols);
	}

	skip = total > maxrows ? total - maxrows : 0;
	shown = total - skip;

	v = 0;
	for (i = 0; i < sl_rp_rows; i++) {
		len = min(strlen(sl_rp[i]), (size_t)(SL_LINELEN - 1));
		chunks = sl_chunks(len, sl_rp_cols);
		for (c = 0; c < chunks; c++, v++) {
			if (v < skip)
				continue;
			py = top + (v - skip + 1) * sl_rp_ch;
			memset(row, ' ', sl_rp_cols);
			sl_wrap_into(row, sl_rp[i], len, c, sl_rp_cols);
			for (j = 0; j < sl_rp_cols; j++)
				sl_glyph_s(j * sl_rp_cw, py, (unsigned char)row[j],
					   SL_FG, SL_RP_BG, SL_RP_SCALE * 2);
		}
	}

	len = scnprintf(row, sizeof(row),
			"=== end of previous boot (shown %u of %u) ===",
			shown, total);
	if (len > sl_rp_cols)
		len = sl_rp_cols;
	memset(row + len, ' ', sl_rp_cols - len);
	for (j = 0; j < sl_rp_cols; j++)
		sl_glyph_s(j * sl_rp_cw, top + (maxrows + 1) * sl_rp_ch,
			   (unsigned char)row[j], SL_RP_BANNER_FG, SL_RP_BANNER,
			   SL_RP_SCALE * 2);

	sl_clean(top, bottom);
	sl_rp_drawn = true;
}

static unsigned int sl_chunks(unsigned int len, unsigned int cols)
{
	return len ? (len + cols - 1) / cols : 1;
}

static void sl_wrap_into(char *dst, const char *src, unsigned int len,
			 unsigned int c, unsigned int cols)
{
	unsigned int start = c * cols, clen;

	clen = (len > start) ? min(len - start, cols) : 0;
	if (clen)
		memcpy(dst, src + start, clen);
}

static void sl_build_rows(unsigned int rows)
{
	unsigned int i, k, li = sl_head, n = 0, len, chunks, c;

	for (i = 0; i < rows; i++)
		memset(sl_prow[i], ' ', sl_cols);

	for (k = 0; k < sl_used && n < rows; k++) {
		len = min(sl_rlen[li], (unsigned int)(SL_LINELEN - 1));
		chunks = sl_chunks(len, sl_cols);
		for (c = chunks; c-- > 0 && n < rows; n++)
			sl_wrap_into(sl_prow[rows - 1 - n], sl_ring[li], len, c,
				     sl_cols);
		li = (li + SL_LINES - 1) % SL_LINES;
	}
}

static void sl_render_locked(bool partial)
{
	unsigned int i, j, lo, hi, first = 0, tail, x0 = SL_MARGIN;
	unsigned int y0 = SL_BAR_H + SL_MARGIN;
	unsigned int rows = sl_rows;
	bool full = sl_force;

	if (!sl_ready || sl_fb_frozen)
		return;

	if (full)
		partial = false;

	if (sl_rp_drawn) {
		if (sl_rp_full)
			rows = 0;
		else if (sl_rp_y > y0)
			rows = min(rows, (sl_rp_y - y0) / sl_ch);
	}

	if (full) {

		unsigned int clr = (sl_rp_drawn && !sl_rp_full) ? sl_rp_y : sl_h;

		sl_rect(0, 0, sl_w, clr, SL_BG);
		memset(sl_shown, 0, sizeof(sl_shown));
		sl_force = false;
	}

	sl_bar_locked();
	sl_build_rows(rows);

	tail = rows > SL_KICK_ROWS ? rows - SL_KICK_ROWS : 0;
	if (partial)
		first = tail;

	lo = rows;
	hi = 0;
	for (i = first; i < rows; i++) {
		if (!full && !memcmp(sl_prow[i], sl_shown[i], sl_cols))
			continue;
		for (j = 0; j < sl_cols; j++)
			sl_glyph(x0 + j * sl_cw, y0 + i * sl_ch,
				 (unsigned char)sl_prow[i][j], SL_FG, SL_BG);
		memcpy(sl_shown[i], sl_prow[i], sl_cols);
		if (i < lo)
			lo = i;
		hi = i + 1;
	}
	if (hi > lo)
		sl_clean(y0 + lo * sl_ch, y0 + hi * sl_ch);

	if (tail) {
		unsigned int sy = y0 + (tail - 1) * sl_ch;

		sl_rect(0, sy, sl_w, sl_ch, SL_SEP);
		sl_clean(sy, sy + sl_ch);
	}

	if (sl_rp_drawn)
		sl_rp_render_locked(full);

	if (sl_bootdump >= 1)
		sl_diag_locked();
	sl_status_locked();

	sl_dirty = first > 0;
}

static void sl_render(void)
{
	unsigned long flags;

	if (!sl_ready)
		return;
	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_render_locked(false);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

/* Timers are unavailable during the setup_arch() early console stage. */
static void sl_kick(void)
{
	if (!sl_rtimer_ok || sl_rtimer_armed || sl_dead || sl_fb_frozen)
		return;
	sl_rtimer_armed = true;
	mod_timer(&sl_rtimer, jiffies + SL_RENDER_J);
}

static void sl_render_tick(struct timer_list *t)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_rtimer_armed = false;
	if (!sl_dead && sl_dirty)
		sl_render_locked(true);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_border(u32 c)
{
	unsigned long flags;

	if (!sl_ready)
		return;
	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_rect(0, 0, sl_w, 24, c);
	sl_rect(0, sl_h - 24, sl_w, 24, c);
	sl_clean(0, 24);
	sl_clean(sl_h - 24, sl_h);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

/* A tab at the line limit must terminate even before its next tab stop. */
static void sl_feed_locked(const char *s, unsigned int count, bool to_bb)
{
	const char *orig = s;
	unsigned int n = count;

	sl_last_msg = jiffies;

	while (count--) {
		char c = *s++;
		unsigned int *len = &sl_rlen[sl_head];

		if (c == '\n') {
			sl_head = (sl_head + 1) % SL_LINES;
			sl_rlen[sl_head] = 0;
			if (sl_used < SL_LINES)
				sl_used++;
		} else if (c == '\r') {
			*len = 0;
		} else if (c == '\b') {
			if (*len)
				(*len)--;
		} else if (c == '\t') {
			do {
				if (*len < SL_LINELEN - 1)
					sl_ring[sl_head][(*len)++] = ' ';
			} while ((*len & 7) && *len < SL_LINELEN - 1);
		} else if (c >= 0x20 && c < 0x7f) {
			if (*len < SL_LINELEN - 1)
				sl_ring[sl_head][(*len)++] = c;
		}
	}

	if (to_bb)
		sl_bb_write_locked(orig, n);
	sl_dirty = true;
	sl_kick();
}

static void sl_puts(const char *s, bool to_bb)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_feed_locked(s, strlen(s), to_bb);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

/* ---------------- console ---------------- */

#define SL_EV_SIZE	8192
static char sl_ev[SL_EV_SIZE];
static size_t sl_ev_len;
static char sl_ev_last[256];
static unsigned int sl_ev_last_len;
static DEFINE_RAW_SPINLOCK(sl_evidence_lock);

/* Extend the evidence keys for a focused investigation, excluding known floods. */
static const char * const sl_ev_keys[] = {
	"init: ", "Service '", "received signal", "crash", "timed out",
	"failed to", "probe failed", "deferred probe", "Unknown symbol",
	"disagrees about version", "no symbol version", "I/O error",
	"fscrypt", "f2fs", "erofs", "keymint", "keystore", "vold",
};

static const char * const sl_ev_skip[] = {
	"not defined in policy", "policy capability", "avc: denied",
	"SELinux: Skipping restorecon on directory",
};

static void sl_evidence_feed(const char *message, unsigned int count)
{
	unsigned int length = min_t(unsigned int, count, sizeof(sl_ev_last) - 1);
	unsigned int index;
	unsigned long flags;
	size_t drop;

	for (index = 0; index < ARRAY_SIZE(sl_ev_skip); index++)
		if (strnstr(message, sl_ev_skip[index], length))
			return;
	for (index = 0; index < ARRAY_SIZE(sl_ev_keys); index++)
		if (strnstr(message, sl_ev_keys[index], length))
			break;
	if (index == ARRAY_SIZE(sl_ev_keys))
		return;
	raw_spin_lock_irqsave(&sl_evidence_lock, flags);
	if (length == sl_ev_last_len && !memcmp(message, sl_ev_last, length))
		goto unlock;
	if (sl_ev_len + length >= sizeof(sl_ev)) {
		drop = sl_ev_len + length - sizeof(sl_ev) + 1;
		while (drop < sl_ev_len && sl_ev[drop - 1] != '\n')
			drop++;
		memmove(sl_ev, sl_ev + drop, sl_ev_len - drop);
		sl_ev_len -= drop;
	}
	memcpy(sl_ev + sl_ev_len, message, length);
	sl_ev_len += length;
	sl_ev[sl_ev_len] = 0;
	memcpy(sl_ev_last, message, length);
	sl_ev_last_len = length;
unlock:
	raw_spin_unlock_irqrestore(&sl_evidence_lock, flags);
}

static void sl_evidence_dump(void)
{
	static char snapshot[4097];
	unsigned long flags;
	size_t start, length;

	if (!sl_bootdump || !raw_spin_trylock_irqsave(&sl_evidence_lock, flags))
		return;
	start = sl_ev_len > sizeof(snapshot) - 1 ? sl_ev_len - sizeof(snapshot) + 1 : 0;
	if (start) {
		while (start < sl_ev_len && sl_ev[start - 1] != '\n')
			start++;
	}
	length = sl_ev_len - start;
	memcpy(snapshot, sl_ev + start, length);
	snapshot[length] = 0;
	raw_spin_unlock_irqrestore(&sl_evidence_lock, flags);
	if (!length)
		return;
	sl_puts("splashlog: ==== EARLY EVIDENCE (tail) ====\n", true);
	sl_puts(snapshot, true);
	sl_puts("\nsplashlog: ==== END EVIDENCE ====\n", true);
}

static void sl_con_write(struct console *co, const char *s, unsigned int count)
{
	unsigned long flags;

	if (sl_dead)
		return;
	WRITE_ONCE(sl_last_console, jiffies);
	atomic_inc(&sl_console_count);
	if (sl_bootdump >= 1)
		sl_capture_display(s, count);

	if (sl_chunk_matches(s, count)) {
		char b[64];
		int n;

		sl_last_msg = jiffies;
		sl_drop_cnt++;

		if (!(sl_drop_cnt % 2000)) {
			n = scnprintf(b, sizeof(b),
				      "splashlog: dropped %u filtered lines\n",
				      sl_drop_cnt);
			raw_spin_lock_irqsave(&sl_lock, flags);
			sl_feed_locked(b, n, true);
			raw_spin_unlock_irqrestore(&sl_lock, flags);
		}
		return;
	}
	if (sl_drop_panic_trace(s, count)) {
		WRITE_ONCE(sl_last_msg, jiffies);
		sl_drop_cnt++;
		return;
	}

	if (sl_bootdump >= 1)
		sl_evidence_feed(s, count);

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_feed_locked(s, count, true);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static struct console sl_console = {
	.name		= "splash",
	.write		= sl_con_write,
	.flags		= CON_PRINTBUFFER | CON_ENABLED,
	.index		= -1,
};

#define SL_LOADING_SLOTS 24
struct sl_loading_module {
	pid_t pid;
	char name[64];
	enum splashlog_module_phase phase;
	unsigned long started, changed;
};

static DEFINE_RAW_SPINLOCK(sl_loading_lock);
static struct sl_loading_module sl_loading[SL_LOADING_SLOTS];
static unsigned int sl_loading_overflow;

void splashlog_modphase(const char *name, enum splashlog_module_phase phase)
{
	unsigned long flags;
	struct sl_loading_module *entry = NULL, *empty = NULL;
	unsigned int index;

	if (!splashlog_modules_enabled())
		return;
	raw_spin_lock_irqsave(&sl_loading_lock, flags);
	for (index = 0; index < SL_LOADING_SLOTS; index++) {
		if (sl_loading[index].pid == current->pid) {
			entry = &sl_loading[index];
			break;
		}
		if (!sl_loading[index].pid && !empty)
			empty = &sl_loading[index];
	}
	if (phase == SL_MODULE_END) {
		if (entry)
			entry->pid = 0;
		goto out;
	}
	if (!entry) {
		entry = empty;
		if (!entry) {
			sl_loading_overflow++;
			goto out;
		}
		entry->pid = current->pid;
		entry->started = jiffies;
	}
	strscpy(entry->name, name, sizeof(entry->name));
	entry->phase = phase;
	entry->changed = jiffies;
out:
	raw_spin_unlock_irqrestore(&sl_loading_lock, flags);
}

static void sl_wait_stack(pid_t pid)
{
	struct task_struct *task;
	unsigned long entries[10];
	unsigned int count = 0, index;
	char line[192], comm[TASK_COMM_LEN];

	task = find_get_task_by_vpid(pid);
	if (!task)
		return;
	get_task_comm(comm, task);
	if (!task_is_running(task))
		count = stack_trace_save_tsk(task, entries, ARRAY_SIZE(entries), 0);
	scnprintf(line, sizeof(line), "WAIT pid=%d comm=%s state=%c frames=%u\n",
		  pid, comm, task_state_to_char(task), count);
	put_task_struct(task);
	sl_puts(line, true);
	for (index = 0; index < count; index++) {
		scnprintf(line, sizeof(line), "  %pS\n", (void *)entries[index]);
		sl_puts(line, true);
	}
}

static void sl_sample_progress(void)
{
	static unsigned long diagnosed_console;
	static unsigned int samples;
	static const char * const phases[] = {
		"end", "file/load/wait", "init", "init-returned", "async-sync", "finish",
	};
	static struct sl_loading_module snapshot[SL_LOADING_SLOTS];
	struct kmsg_dump_iter iter;
	struct task_struct *group, *task;
	pid_t display_pids[6];
	unsigned long flags, last = READ_ONCE(sl_last_console);
	u64 delivered;
	unsigned int index, active = 0, shown = 0, overflow, records = 0;
	unsigned int display_count = 0;
	char line[384];
	size_t length;

	kmsg_dump_rewind(&iter);
	delivered = READ_ONCE(sl_console.seq);
	WRITE_ONCE(sl_printk_seq, iter.next_seq);
	WRITE_ONCE(sl_delivery_seq, delivered);
	if (!sl_bootdump)
		return;
	if (last != diagnosed_console)
		samples = 0;
	if (samples >= 3 ||
	    time_before(jiffies, last + (5 + samples * 10) * HZ))
		return;
	diagnosed_console = last;
	samples++;
	scnprintf(line, sizeof(line),
		  "PROGRESS t=%ums rb=%llu cs=%llu pending=%llu callbacks=%d silent=%lus\n",
		  jiffies_to_msecs(jiffies - INITIAL_JIFFIES), iter.next_seq, delivered,
		  iter.next_seq > delivered ? iter.next_seq - delivered : 0,
		  atomic_read(&sl_console_count), (jiffies - last) / HZ);
	sl_puts(line, true);
	if (iter.next_seq > delivered) {
		iter.cur_seq = max(iter.cur_seq, max(delivered,
			iter.next_seq > 8 ? iter.next_seq - 8 : 0));
		while (records++ < 8 && iter.cur_seq < iter.next_seq &&
		       kmsg_dump_get_line(&iter, true, line, sizeof(line) - 1, &length)) {
			line[length] = '\0';
			sl_puts("RING ", true);
			sl_puts(line, true);
		}
	}
	raw_spin_lock_irqsave(&sl_loading_lock, flags);
	memcpy(snapshot, sl_loading, sizeof(snapshot));
	overflow = sl_loading_overflow;
	raw_spin_unlock_irqrestore(&sl_loading_lock, flags);
	for (index = 0; index < SL_LOADING_SLOTS; index++) {
		struct sl_loading_module *entry = &snapshot[index];

		if (!entry->pid)
			continue;
		active++;
		if (shown++ >= 8)
			continue;
		scnprintf(line, sizeof(line),
			  "LOAD pid=%d module=%s phase=%s age=%ums phase-age=%ums\n",
			  entry->pid, entry->name, phases[entry->phase],
			  jiffies_to_msecs(jiffies - entry->started),
			  jiffies_to_msecs(jiffies - entry->changed));
		sl_puts(line, true);
		if (sl_bootdump >= 2)
			sl_wait_stack(entry->pid);
	}
	scnprintf(line, sizeof(line), "LOAD active=%u overflow=%u sample=%u\n",
		  active, overflow, samples);
	sl_puts(line, true);
	if (sl_bootdump < 2)
		return;
	sl_wait_stack(1);
	rcu_read_lock();
	for_each_process_thread(group, task) {
		if (strncmp(task->comm, "crtc_commit:", 12) &&
		    strncmp(task->comm, "crtc_event:", 11) &&
		    strcmp(group->comm, "surfaceflinger"))
			continue;
		if (display_count >= ARRAY_SIZE(display_pids))
			goto display_collected;
		display_pids[display_count++] = task->pid;
	}
display_collected:
	rcu_read_unlock();
	for (index = 0; index < display_count; index++)
		sl_wait_stack(display_pids[index]);
}

static void splashlog_stage(unsigned int stage)
{
	char buf[64];
	unsigned long flags;
	int n;

	if (!sl_enabled || sl_dead)
		return;
	if (stage > SL_STAGES)
		stage = SL_STAGES;

	n = scnprintf(buf, sizeof(buf), "### stage %u/%u\n", stage, SL_STAGES);
	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_stage = stage;
	sl_feed_locked(buf, n, true);
	sl_bar_locked();
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_tick(struct timer_list *t)
{
	unsigned long flags;

	if (sl_dead || READ_ONCE(sl_native_display))
		return;

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_flip = !sl_flip;
	if (sl_ready) {
		if (sl_dirty) {
			sl_render_locked(false);
		} else {
			sl_bar_locked();

			sl_status_locked();
		}
	}
	raw_spin_unlock_irqrestore(&sl_lock, flags);

	mod_timer(&sl_timer, jiffies + msecs_to_jiffies(SL_TICK_MS));
}

static bool sl_has_blocked_task(void)
{
	struct task_struct *p;
	bool found = false;

	rcu_read_lock();
	for_each_process(p) {
		if (READ_ONCE(p->__state) == TASK_UNINTERRUPTIBLE) {
			found = true;
			break;
		}
	}
	rcu_read_unlock();
	return found;
}

static void sl_stall_tick(struct timer_list *t)
{
	unsigned long idle;
	bool blocked;

	if (sl_dead || !sl_enabled || !sl_stall_secs || !sl_stall_ok)
		return;

	idle = jiffies - sl_last_msg;

	if (time_before(jiffies, 15UL * HZ) ||
	    idle < (unsigned long)sl_stall_secs * HZ) {
		mod_timer(t, jiffies + HZ);
		return;
	}

	blocked = sl_has_blocked_task();

	pr_emerg("splashlog: STALL %lus at stage %u, blocked_task=%d -- panic on purpose\n",
		 idle / HZ, sl_stage, blocked);

	if (!blocked) {
		mod_timer(t, jiffies + HZ);
		return;
	}

	if (sl_bootdump >= 2)
		show_state_filter(TASK_UNINTERRUPTIBLE);
	panic("splashlog: boot stalled at stage %u for %lus with blocked tasks\n",
	      sl_stage, idle / HZ);
}

static inline unsigned long jiffies_uptime_secs(void)
{
	return (jiffies - INITIAL_JIFFIES) / HZ;
}

static void sl_dump_tasklist(void);

static unsigned int sl_hang_secs;
static atomic_t sl_hang_fired = ATOMIC_INIT(0);

static DEFINE_PER_CPU(struct timer_list, sl_hang_timer);
static int sl_hang_early_cpu = -1;

static int __init splashlog_hang_setup(char *str)
{
	unsigned int v;

	if (!str)
		return 0;
	if (!kstrtouint(str, 0, &v) && v >= 2 && v <= 300)
		sl_hang_secs = v;
	return 0;
}
early_param("spl.hang", splashlog_hang_setup);

#define SL_HANG_MAX_SECS	120

static void sl_hang_tick(struct timer_list *t)
{
	unsigned long idle;

	if (sl_dead || !sl_enabled || !sl_hang_secs)
		return;

	if (jiffies_uptime_secs() > max_t(unsigned long, SL_HANG_MAX_SECS,
					DIV_ROUND_UP(sl_deadline_ms, 1000) + sl_hang_secs))
		return;

	idle = jiffies - READ_ONCE(sl_last_console);
	if (idle < (unsigned long)sl_hang_secs * HZ) {
		mod_timer(t, jiffies + HZ);
		return;
	}

	if (atomic_cmpxchg(&sl_hang_fired, 0, 1))
		return;

	sl_fault_why = "console silent (hang watchdog)";
	WRITE_ONCE(sl_watchdog_fault, true);
	WRITE_ONCE(sl_fault_phase, 1);
	sl_canary_mark(true);
	queue_work(system_unbound_wq, &sl_fault_work);

	{

		char b[128];
		int n;

		n = scnprintf(b, sizeof(b),
			      "splashlog: HANG %lus silent at stage %u -- restart queued\n",
			      idle / HZ, sl_stage);
		sl_puts(b, true);
	}

}

#define SL_TASK_MAX	40
static void sl_dump_tasklist(void)
{
	struct task_struct *p;
	char buf[128];
	unsigned long flags;
	unsigned int total = 0, blocked = 0;

	struct { char comm[TASK_COMM_LEN]; unsigned int n; int first_pid; } agg[16];
	unsigned int aggn = 0, i, j;

	rcu_read_lock();
	for_each_process(p)
		total++;
	rcu_read_unlock();

	scnprintf(buf, sizeof(buf), "TASKSUM total=%u\n", total);
	sl_puts(buf, true);

	rcu_read_lock();
	for_each_process(p) {
		unsigned int st = READ_ONCE(p->__state);

		if (st != TASK_UNINTERRUPTIBLE && st != TASK_KILLABLE)
			continue;
		blocked++;
		for (i = 0; i < aggn; i++) {
			if (!strncmp(agg[i].comm, p->comm, TASK_COMM_LEN)) {
				agg[i].n++;
				break;
			}
		}
		if (i == aggn && aggn < ARRAY_SIZE(agg)) {
			strscpy(agg[i].comm, p->comm, TASK_COMM_LEN);
			agg[i].n = 1;
			agg[i].first_pid = p->pid;
			aggn++;
		}
	}
	rcu_read_unlock();

	for (i = 0; i < aggn; i++) {

		for (j = i + 1; j < aggn; j++) {
			if (agg[j].n > agg[i].n) {
				typeof(agg[0]) t = agg[i];

				agg[i] = agg[j];
				agg[j] = t;
			}
		}
		if (i >= SL_TASK_MAX)
			break;
		scnprintf(buf, sizeof(buf),
			  "TASK blocked %-16s n=%-4u first_pid=%d\n",
			  agg[i].comm, agg[i].n, agg[i].first_pid);
		raw_spin_lock_irqsave(&sl_lock, flags);
		sl_feed_locked(buf, strlen(buf), true);
		raw_spin_unlock_irqrestore(&sl_lock, flags);
	}

	scnprintf(buf, sizeof(buf), "TASKSUM blocked=%u kinds=%u\n",
		  blocked, aggn);
	sl_puts(buf, true);
}

#define SL_STACK_MAX	3
static void sl_dump_blocked_stacks(void)
{
	struct task_struct *p;
	unsigned int shown = 0;

	rcu_read_lock();
	for_each_process(p) {
		unsigned int st = READ_ONCE(p->__state);

		if (st != TASK_UNINTERRUPTIBLE && st != TASK_KILLABLE)
			continue;
		if (shown >= SL_STACK_MAX)
			break;

		pr_emerg("splashlog: ---- stack of %s/%d state=0x%x ----\n",
			 p->comm, p->pid, st);
		sched_show_task(p);
		shown++;
	}
	rcu_read_unlock();
}

static void sl_deadline_tick(struct timer_list *t)
{
	char buf[96];
	int n;

	if (!sl_enabled || !sl_deadline_ms)
		return;
	if (sl_halted)
		return;
	if (atomic_cmpxchg(&sl_dl_fired, 0, 1))
		return;

	sl_fault_why = sl_deadline_panic ?
		"boot-deadline (diagnostic panic)" :
		"boot-deadline (no panic, absolute timeout)";
	WRITE_ONCE(sl_watchdog_fault, true);
	WRITE_ONCE(sl_fault_phase, 1);
	n = scnprintf(buf, sizeof(buf),
		      "splashlog: boot deadline %u.%03us reached (stage %u) -- %s\n",
		      sl_deadline_ms / 1000, sl_deadline_ms % 1000, sl_stage,
		      sl_deadline_panic ? "diagnostic panic" : "warm restart");

	sl_puts(buf, true);
	queue_work(system_unbound_wq, &sl_fault_work);

}

#define SL_BEAT_MS	200

#define SL_BEAT_MS_LATE		2000
#define SL_BEAT_EARLY_SECS	10

#define SL_BEAT_LOG_EVERY	5

#define SL_BEAT_MAX_SECS	600
static struct timer_list sl_beat_timer;
static unsigned int sl_beat_n;
static unsigned int sl_kbeat_n;

static unsigned int sl_warnpanic_ms;
static bool sl_wp_armed;

static int __init splashlog_warnpanic_setup(char *str)
{
	unsigned int v;

	if (!str)
		return 0;
	if (!kstrtouint(str, 0, &v))
		sl_warnpanic_ms = v;
	return 0;
}
early_param("splashlog.wp", splashlog_warnpanic_setup);

static inline unsigned int sl_beat_interval(void)
{
	return jiffies_uptime_secs() < SL_BEAT_EARLY_SECS ? SL_BEAT_MS : SL_BEAT_MS_LATE;
}

static inline unsigned long sl_uptime_ms(void)
{
	return (unsigned long)(jiffies - INITIAL_JIFFIES) * 1000UL / HZ;
}

/* Heartbeats must not refresh the console-silence watchdog timestamp. */
static void sl_bb_puts(const char *s)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_bb_write_locked(s, strlen(s));
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_beat_tick(struct timer_list *t)
{
	char buf[128];

	if (!sl_enabled || sl_dead || jiffies_uptime_secs() > SL_BEAT_MAX_SECS)
		return;

	if (sl_warnpanic_ms && !sl_wp_armed && sl_uptime_ms() >= sl_warnpanic_ms) {
		sl_wp_armed = true;
		WRITE_ONCE(panic_on_warn, 1);
		sl_bb_puts("splashlog: panic_on_warn armed\n");
		pr_info("splashlog: panic_on_warn armed at %lums\n", sl_uptime_ms());
	}

	if (!(sl_beat_n++ % SL_BEAT_LOG_EVERY))
		pr_info("splashlog: alive %lus stage=%u thr=%d ctxt=%llu cur=%s/%d\n",
			(unsigned long)jiffies_uptime_secs(), sl_stage, nr_threads,
			nr_context_switches(), current->comm, current->pid);

	scnprintf(buf, sizeof(buf),
		      "splashlog: beat-raw %lu.%03lus stage=%u cpu=%d cur=%s/%d\n",
		      sl_uptime_ms() / 1000, sl_uptime_ms() % 1000, sl_stage,
		      raw_smp_processor_id(), current->comm, current->pid);
	sl_bb_puts(buf);

	mod_timer(t, jiffies + msecs_to_jiffies(sl_beat_interval()));
}

static struct task_struct *sl_beat_task;

static int sl_beat_thread(void *arg)
{
	char buf[96];
	unsigned long next_service_sample = INITIAL_JIFFIES + 15 * HZ;

	while (!kthread_should_stop()) {
		msleep(sl_beat_on ? sl_beat_interval() : 1000);
		if (!sl_enabled || sl_dead || sl_halted ||
		    jiffies_uptime_secs() > SL_BEAT_MAX_SECS)
			break;
		WRITE_ONCE(sl_last_kbeat, jiffies);
		if (sl_bootdump >= 2 && time_after_eq(jiffies, next_service_sample) &&
		    jiffies_uptime_secs() <= SL_BEAT_MAX_SECS) {
			next_service_sample = jiffies + 15 * HZ;
			schedule_work(&sl_service_work);
		}
		sl_sample_progress();
		if (!sl_beat_on)
			continue;
		if (jiffies_uptime_secs() > SL_BEAT_MAX_SECS)
			break;
		if (!(sl_kbeat_n++ % SL_BEAT_LOG_EVERY))
			pr_info("splashlog: alive-kthr %lus stage=%u cur=%s/%d\n",
				(unsigned long)jiffies_uptime_secs(), sl_stage, current->comm, current->pid);

		scnprintf(buf, sizeof(buf),
			      "splashlog: kbeat-raw %lu.%03lus stage=%u cpu=%d cur=%s/%d\n",
			      sl_uptime_ms() / 1000, sl_uptime_ms() % 1000, sl_stage,
			      raw_smp_processor_id(), current->comm, current->pid);
		sl_bb_puts(buf);
	}
	return 0;
}

static int __init splashlog_beat_thread_init(void)
{
	if (!sl_enabled || (!sl_bootdump && !sl_beat_on))
		return 0;
	sl_beat_task = kthread_run(sl_beat_thread, NULL, "splashlog-beat");
	if (IS_ERR(sl_beat_task))
		pr_warn("splashlog: beat kthread failed (%ld)\n",
			PTR_ERR(sl_beat_task));
	return 0;
}
late_initcall(splashlog_beat_thread_init);

static void sl_dump_kmsg(void)
{
	struct kmsg_dump_iter iter;
	char line[512];
	size_t len;
	unsigned long flags;

	kmsg_dump_rewind(&iter);

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_head = 0;
	sl_used = 1;
	sl_rlen[0] = 0;
	sl_bb_reset();
	raw_spin_unlock_irqrestore(&sl_lock, flags);

	while (kmsg_dump_get_line(&iter, false, line, sizeof(line), &len)) {

		if (sl_chunk_matches(line, len))
			continue;
		raw_spin_lock_irqsave(&sl_lock, flags);
		sl_feed_locked(line, len, true);
		raw_spin_unlock_irqrestore(&sl_lock, flags);
	}
}

static void sl_dump_kmsg_keep(void)
{
	static const char *const pats[] = {
		"execv", "first_stage", "Failed to mount",
		"sys.powerctl", "powerctl", "Reboot start", "Rebooting",
		"rebooting", "zygote", "Zygote", "fatal", "linkerconfig",
		"avc: denied", "vold: Failed", "vold: Error",
		"vold: Key", "vold: fscrypt", "boot_progress", "panic",
		"Attempted to kill init", "init: failed", "init: Failed",
		"init: Could not", "init: Reboot",
		"UBSAN", "CFI failure", "array index out of bounds",
		"Unknown symbol", "disagrees about version", "request_firmware",
		"inline-crypto", "default-key", "dm-", "fscrypt", "f2fs", "F2FS",
		"I/O error", "EXT4-fs error", "Cannot mount",
	};
	struct kmsg_dump_iter iter;
	const char *mark = "### KEY LINES: init / mount ###\n";
	char line[512];
	size_t len;
	unsigned long flags;
	size_t i;
	unsigned int trace_lines = 0;
	int n;

	kmsg_dump_rewind(&iter);

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_feed_locked(mark, strlen(mark), true);
	raw_spin_unlock_irqrestore(&sl_lock, flags);

	while (kmsg_dump_get_line(&iter, false, line, sizeof(line), &len)) {
		bool keep = false;
		bool in_trace = trace_lines > 0;

		if (sl_chunk_matches(line, len))
			continue;

		for (i = 0; i < ARRAY_SIZE(pats); i++) {
			if (strstr(line, pats[i])) {
				keep = true;
				break;
			}
		}
		if (in_trace) {
			keep = true;
			trace_lines--;
		}
		if (sl_bootdump >= 2 && strstr(line, "Call trace:"))
			trace_lines = 18;
		if (!keep)
			continue;

		raw_spin_lock_irqsave(&sl_lock, flags);
		sl_feed_locked(line, len, true);
		raw_spin_unlock_irqrestore(&sl_lock, flags);
	}

	n = scnprintf(line, sizeof(line), "### KEY LINES end ###\n");
	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_feed_locked(line, n, true);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
}

static void sl_dump_fault_context(void)
{
	struct kmsg_dump_iter iter;
	char line[512];
	size_t len;
	unsigned int remaining = 64;
	unsigned long flags;
	bool found = false;

	kmsg_dump_rewind(&iter);
	while (kmsg_dump_get_line(&iter, false, line, sizeof(line), &len)) {
		if (!found && (strstr(line, "Internal error:") ||
			       (strstr(line, "UBSAN:") &&
				!strstr(line, "Kernel panic")))) {
			found = true;
			raw_spin_lock_irqsave(&sl_lock, flags);
			sl_feed_locked("### FIRST FAULT CONTEXT ###\n",
				       sizeof("### FIRST FAULT CONTEXT ###\n") - 1,
				       true);
			raw_spin_unlock_irqrestore(&sl_lock, flags);
		}
		if (!found)
			continue;
		if (sl_chunk_matches(line, len))
			continue;
		if (strstr(line, "Modules linked in:"))
			continue;
		if (strstr(line, "NMI backtrace"))
			break;

		raw_spin_lock_irqsave(&sl_lock, flags);
		sl_feed_locked(line, len, true);
		raw_spin_unlock_irqrestore(&sl_lock, flags);
		if (strstr(line, "---[ end trace") || strstr(line, "Code:"))
			break;
		if (!--remaining)
			break;
	}
}

static int sl_panic_notify(struct notifier_block *nb, unsigned long ev, void *p)
{
	char buf[1024];

	if (!sl_enabled)
		return NOTIFY_DONE;

	sl_dead = true;
	sl_fault_display();
	sl_bb_frozen = false;
	sl_rp_drawn = false;
	/* Panic may have stopped a CPU inside the timer callback. */
	if (sl_timer_on)
		timer_delete(&sl_timer);
	if (p)
		sl_fault_why = p;

	scnprintf(buf, sizeof(buf), "### PANIC at stage %u\n", sl_stage);
	sl_puts(buf, true);

	/* Mark the crash before any dump that might itself stall. */
	sl_canary_mark(true);
	if (sl_bootdump >= 2)
		sl_dump_kmsg();
	if (sl_bootdump >= 1) {
		sl_dump_kmsg_keep();
		sl_evidence_dump();
		sl_dump_modring();
	}
	if (sl_first_die[0])
		sl_puts(sl_first_die, true);
	if (sl_bootdump >= 2) {
		if (sl_first_die[0]) {
			unsigned int index;

			for (index = 0; index < sl_first_die_depth; index++) {
				scnprintf(buf, sizeof(buf),
					  "splashlog: fault-stack %pS\n",
					  (void *)sl_first_die_stack[index]);
				sl_puts(buf, true);
			}
		}
		sl_dump_fault_context();
	}
	if (sl_bootdump >= 1)
		sl_dump_user_faults();
	if (sl_bootdump >= 2)
		sl_dump_service_snapshot();

	scnprintf(buf, sizeof(buf), "splashlog: PANIC why=%s\n",
		      p ? (const char *)p : "?");
	sl_puts(buf, true);
	sl_render();
	sl_border(SL_RED);

	/* Keep post-notifier CPU stop and console replay noise out of the tail. */
	sl_bb_frozen = true;
	return NOTIFY_DONE;
}

static int sl_die_notify(struct notifier_block *nb, unsigned long ev, void *p)
{
	struct die_args *args = p;
	char buf[80];

	if (!sl_enabled)
		return NOTIFY_DONE;
	sl_fault_display();
	sl_bb_frozen = false;
	if (!sl_first_die[0] && args && args->regs) {
		scnprintf(sl_first_die, sizeof(sl_first_die),
			  "splashlog: FIRST_DIE %s cpu=%u comm=%s/%d pc=%pS lr=%pS esr=%lx\n",
			  args->str ?: "?", raw_smp_processor_id(),
			  current->comm, current->pid,
			  (void *)instruction_pointer(args->regs),
			  (void *)args->regs->regs[30], args->err);
		if (sl_bootdump >= 2)
			sl_first_die_depth = stack_trace_save_regs(args->regs,
						 sl_first_die_stack,
						 ARRAY_SIZE(sl_first_die_stack), 0);
		sl_bb_puts(sl_first_die);
	}

	scnprintf(buf, sizeof(buf), "### OOPS at stage %u ev=%lu\n",
		      sl_stage, ev);
	sl_puts(buf, true);

	/* Avoid re-entering the log dump while the oops path is printing it. */
	if (panic_on_oops) {
		sl_border(SL_YEL);
		return NOTIFY_DONE;
	}

	if (sl_bootdump >= 2)
		sl_dump_kmsg();
	sl_render();
	sl_border(SL_YEL);

	if (sl_reboot_on_oops && ev == DIE_OOPS) {

		sl_canary_mark(true);
		sl_bb_frozen = true;
		sl_fault_why = "oops (kernel die, reboot_on_oops)";
		schedule_work(&sl_fault_work);
	}
	return NOTIFY_DONE;
}

static struct notifier_block sl_panic_nb = {
	.notifier_call = sl_panic_notify,
	.priority = 255,
};
static struct notifier_block sl_die_nb = {
	.notifier_call = sl_die_notify,
	.priority = 255,
};

static int sl_reboot_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	unsigned long flags;

	if (!sl_enabled)
		return NOTIFY_DONE;
	if (READ_ONCE(sl_dead) || READ_ONCE(sl_watchdog_fault) ||
	    READ_ONCE(sl_fault_phase) || READ_ONCE(sl_first_die[0]))
		return NOTIFY_DONE;

	raw_spin_lock_irqsave(&sl_lock, flags);
	sl_bb_frozen = true;
	sl_bb_reset();
	sl_canary_mark(false);
	raw_spin_unlock_irqrestore(&sl_lock, flags);
	return NOTIFY_DONE;
}

static struct notifier_block sl_reboot_nb = {
	.notifier_call = sl_reboot_notify,
	.priority = 255,
};

/*
 * Watches use finit_module() file basenames, including their hyphen spelling.
 * Add a basename here for a new investigation; no other slot count needs edits.
 * The first and last result distinguish duplicate copies in module search paths.
 */
static const char * const sl_watch_names[] = {
	"gcc-cliffs", "dispcc-pineapple", "ufs_qcom", "msm_drm",
	"smcinvoke_dlkm", "qcom_q6v5_pas", "camera",
};

#define SL_MODRING_N	16
#define SL_WATCH_N	ARRAY_SIZE(sl_watch_names)

struct sl_module_history {
	char recent[SL_MODRING_N][28];
	char failed[16][28];
	unsigned int head, count, failures, duplicates;
	struct {
		char first[56], last[56];
		unsigned int attempts;
	} watch[SL_WATCH_N];
};

static DEFINE_RAW_SPINLOCK(sl_module_lock);
static struct sl_module_history sl_modules;

bool splashlog_modules_enabled(void)
{
	return sl_enabled && sl_bootdump >= 1 && !READ_ONCE(sl_dead);
}

void splashlog_modattempt(const char *path, int err)
{
	const char *base, *tail;
	unsigned long flags;
	unsigned int index;
	size_t length;

	if (!splashlog_modules_enabled() || !path)
		return;
	base = strrchr(path, '/');
	base = base ? base + 1 : path;
	length = strlen(path);
	tail = length > 44 ? path + length - 44 : path;
	raw_spin_lock_irqsave(&sl_module_lock, flags);
	for (index = 0; index < SL_WATCH_N; index++) {
		size_t name_length = strlen(sl_watch_names[index]);

		if (strncmp(base, sl_watch_names[index], name_length) ||
		    (base[name_length] && strcmp(base + name_length, ".ko")))
			continue;
		if (!sl_modules.watch[index].attempts)
			scnprintf(sl_modules.watch[index].first,
				  sizeof(sl_modules.watch[index].first), "%s!%d", tail, err);
		sl_modules.watch[index].attempts++;
		scnprintf(sl_modules.watch[index].last,
			  sizeof(sl_modules.watch[index].last), "%s!%d", tail, err);
		break;
	}
	raw_spin_unlock_irqrestore(&sl_module_lock, flags);
}

void splashlog_modtrace(const char *name)
{
	char line[SL_LINELEN];
	unsigned long flags;

	if (!sl_enabled || READ_ONCE(sl_dead))
		return;
	if (sl_bootdump >= 1) {
		raw_spin_lock_irqsave(&sl_module_lock, flags);
		strscpy(sl_modules.recent[sl_modules.head], name,
			 sizeof(sl_modules.recent[0]));
		sl_modules.head = (sl_modules.head + 1) % SL_MODRING_N;
		if (sl_modules.count < SL_MODRING_N)
			sl_modules.count++;
		raw_spin_unlock_irqrestore(&sl_module_lock, flags);
	}
	if (!sl_mods)
		return;
	scnprintf(line, sizeof(line), "### mod %s\n", name);
	sl_puts(line, true);
}

void splashlog_modfail(const char *name, int err)
{
	unsigned long flags;

	if (!splashlog_modules_enabled())
		return;
	raw_spin_lock_irqsave(&sl_module_lock, flags);
	/* Repeated loads are normal and must not displace actual failures. */
	if (err == -EEXIST)
		sl_modules.duplicates++;
	else if (sl_modules.failures < ARRAY_SIZE(sl_modules.failed))
		scnprintf(sl_modules.failed[sl_modules.failures++],
			  sizeof(sl_modules.failed[0]), "%s!%d", name ?: "?", err);
	raw_spin_unlock_irqrestore(&sl_module_lock, flags);
}

static void sl_dump_modring(void)
{
	static struct sl_module_history snapshot;
	char line[512];
	unsigned long flags;
	unsigned int index, slot;
	int length;

	/* A CPU stopped in a writer must not prevent the panic from completing. */
	if (!raw_spin_trylock_irqsave(&sl_module_lock, flags)) {
		sl_puts("splashlog: module snapshot busy\n", true);
		return;
	}
	memcpy(&snapshot, &sl_modules, sizeof(snapshot));
	raw_spin_unlock_irqrestore(&sl_module_lock, flags);
	length = scnprintf(line, sizeof(line), "splashlog: lastmods(%u):", snapshot.count);
	for (index = 0; index < snapshot.count; index++) {
		slot = (snapshot.head + SL_MODRING_N - snapshot.count + index) % SL_MODRING_N;
		length += scnprintf(line + length, sizeof(line) - length, " %s", snapshot.recent[slot]);
	}
	scnprintf(line + length, sizeof(line) - length, "\n");
	sl_puts(line, true);
	for (index = 0; index < SL_WATCH_N; index++) {
		if (!snapshot.watch[index].attempts)
			scnprintf(line, sizeof(line), "splashlog: watch %s NEVER\n", sl_watch_names[index]);
		else
			scnprintf(line, sizeof(line), "splashlog: watch %s x%u first=%s last=%s\n",
				  sl_watch_names[index], snapshot.watch[index].attempts,
				  snapshot.watch[index].first, snapshot.watch[index].last);
		sl_puts(line, true);
	}
	length = scnprintf(line, sizeof(line), "splashlog: modfail(%u,duplicates=%u):",
			   snapshot.failures, snapshot.duplicates);
	for (index = 0; index < snapshot.failures; index++)
		length += scnprintf(line + length, sizeof(line) - length, " %s", snapshot.failed[index]);
	scnprintf(line + length, sizeof(line) - length, "\n");
	sl_puts(line, true);
}

void splashlog_warn(const char *file, int line, void *caller)
{
	char buf[SL_LINELEN];

	if (!sl_enabled || sl_dead)
		return;
	scnprintf(buf, sizeof(buf),
		  "splashlog: WARN at %pS (from %s:%d)\n",
		  caller, file ? file : "?", line);
	sl_puts(buf, true);
}

static void sl_geometry(void)
{
	unsigned int live;

	sl_stride = sl_w * 4;
	sl_frame_bytes = (size_t)sl_stride * sl_h;
	sl_frames = sl_size / sl_frame_bytes;
	if (sl_frames < 1)
		sl_frames = 1;
	if (sl_frames > 4)
		sl_frames = 4;

	sl_cw = 4 * sl_scale_x2;
	sl_ch = 8 * sl_scale_x2;
	sl_cols = (sl_w - 2 * SL_MARGIN) / sl_cw;
	sl_rows = (sl_h - (sl_bootdump ? 2 : 1) * sl_ch -
		   SL_BAR_H - 2 * SL_MARGIN) / sl_ch;
	if (sl_cols > SL_LINELEN - 1)
		sl_cols = SL_LINELEN - 1;
	if (sl_rows > SL_MAXROWS)
		sl_rows = SL_MAXROWS;

	live = sl_rows > SL_KICK_ROWS + 6 ? SL_KICK_ROWS : sl_rows / 2;
	sl_rp_y = (SL_BAR_H + SL_MARGIN + live * sl_ch) & ~1u;

	sl_rp_cw = 8 * SL_RP_SCALE;
	sl_rp_ch = 16 * SL_RP_SCALE;
	sl_rp_cols = (sl_w - 2 * SL_MARGIN) / sl_rp_cw;
	if (sl_rp_cols > SL_LINELEN - 1)
		sl_rp_cols = SL_LINELEN - 1;
}

void __init splashlog_early_init(void)
{
	bool have_old;
	bool bb_reserved;

	if (!sl_enabled || sl_ready)
		return;

	sl_size = sl_phys_size;
	sl_geometry();
	if (!memblock_is_reserved(sl_phys) ||
	    !memblock_is_reserved(sl_phys + sl_size - 1))
		sl_puts("splashlog: WARNING splash_region is NOT reserved\n", false);
	if (sl_fb_mode && sl_size >= sl_frame_bytes &&
	    sl_cols >= 16 && sl_rows >= 4) {
		if (sl_fb_mode >= 2)
			sl_frames = 1;
		sl_fb = (u8 *)__va(sl_phys);
		sl_ready = true;
		sl_fb_frozen = sl_fb_mode == 3;
		sl_force = true;
		sl_rlen[0] = 0;
		sl_used = 1;
	}

	if (!sl_bb_phys && !sl_bb_disabled)
		sl_bb_phys = sl_phys + SL_BB_OFF;

	bb_reserved = !sl_bb_disabled && sl_bb_phys &&
		      memblock_is_reserved(sl_bb_phys) &&
		      memblock_is_reserved(sl_bb_phys + SL_BB_SIZE - 1);
	have_old = false;
	if (bb_reserved) {
		have_old = sl_bb_setup();
	} else {
		sl_bb_disabled = true;
		sl_bb_ok = false;
	}

	sl_have_old = have_old;
	if (!sl_fb_mode)
		sl_puts("splashlog: framebuffer writes disabled; blackbox active\n", false);
	else if (sl_fb_mode == 2)
		sl_puts("splashlog: single framebuffer; continuous updates enabled\n", false);
	else if (sl_fb_mode == 3)
		sl_puts("splashlog: single framebuffer; freeze after console replay\n", false);

	sl_rp_full = have_old && (sl_halt == 2 || (sl_halt == 1 && sl_prev_panic));
	sl_rp_hold = sl_rp_full;

	if (sl_fb_mode != 3)
		sl_spare_probe();

	sl_render();

	sl_puts("splashlog: alive after paging_init()\n", true);

	if (sl_bb_disabled)
		sl_puts("splashlog: black box DISABLED (not reserved or off)\n", false);
	if (sl_ready) {
		char buf[96];

		scnprintf(buf, sizeof(buf),
			  "splashlog: fb %pa +%zu KiB, %ux%u px, %ux%u chars\n",
			  &sl_phys, sl_size >> 10, sl_w, sl_h, sl_cols, sl_rows);
		sl_puts(buf, true);
	}

	if (have_old)
		sl_bb_replay();
	if (have_old && (sl_replay_at & 1))
		sl_rp_show();
	sl_bb_reset();

	atomic_notifier_chain_register(&panic_notifier_list, &sl_panic_nb);
	register_die_notifier(&sl_die_nb);
	register_reboot_notifier(&sl_reboot_nb);
}

static int __init splashlog_init(void)
{
	if (!sl_enabled)
		return 0;

	if (sl_deadline_ms) {
		struct timer_list *t = this_cpu_ptr(&sl_dl_timer);

		timer_setup(t, sl_deadline_tick, 0);
		t->expires = jiffies + msecs_to_jiffies(sl_deadline_ms);
		add_timer_on(t, smp_processor_id());
		sl_dl_early_cpu = smp_processor_id();
	}

	if (sl_oops_continue && panic_on_oops) {
		panic_on_oops = 0;
		pr_info("splashlog: panic_on_oops disabled (diagnostic)\n");
	}

	timer_setup(&sl_rtimer, sl_render_tick, 0);
	sl_rtimer_ok = true;

	register_console(&sl_console);
	if (sl_fb_mode == 3 && sl_ready) {
		unsigned long flags;

		pr_info("splashlog: freezing framebuffer after console replay\n");
		raw_spin_lock_irqsave(&sl_lock, flags);
		sl_fb_frozen = false;
		sl_force = true;
		sl_render_locked(false);
		sl_fb_frozen = true;
		raw_spin_unlock_irqrestore(&sl_lock, flags);
		pr_info("splashlog: framebuffer frozen; blackbox recording continues\n");
	}

	if (!sl_quiet && sl_fb_mode != 3) {
		timer_setup(&sl_timer, sl_tick, 0);
		sl_timer_on = true;
		mod_timer(&sl_timer, jiffies + msecs_to_jiffies(SL_TICK_MS));
	}

	if (sl_stall_secs) {
		timer_setup(&sl_stall_timer, sl_stall_tick, 0);
		sl_stall_ok = true;
		sl_last_msg = jiffies;
		mod_timer(&sl_stall_timer, jiffies + HZ);
	}

	timer_setup(&sl_beat_timer, sl_beat_tick, 0);
	if (sl_beat_on || sl_warnpanic_ms)
		mod_timer(&sl_beat_timer, jiffies + msecs_to_jiffies(sl_beat_interval()));

	if (sl_hang_secs) {
		struct timer_list *t = this_cpu_ptr(&sl_hang_timer);

		timer_setup(t, sl_hang_tick, 0);
		add_timer_on(t, smp_processor_id());
		sl_hang_early_cpu = smp_processor_id();
	}

	return 0;
}
early_initcall(splashlog_init);

static int __init splashlog_deadline_arm(void)
{
	int cpu, n = 0;

	if (!sl_enabled || !sl_deadline_ms)
		return 0;

	for_each_online_cpu(cpu) {
		struct timer_list *t = per_cpu_ptr(&sl_dl_timer, cpu);

		if (cpu == sl_dl_early_cpu)
			continue;

		timer_setup(t, sl_deadline_tick, 0);
		t->expires = jiffies + msecs_to_jiffies(sl_deadline_ms);
		add_timer_on(t, cpu);
		n++;
	}

	pr_info("splashlog: %u.%03us boot deadline armed on %d CPU(s)\n",
		sl_deadline_ms / 1000, sl_deadline_ms % 1000, n);
	return 0;
}
late_initcall(splashlog_deadline_arm);

static int __init splashlog_hang_arm(void)
{
	int cpu, n = 0;

	if (!sl_enabled || !sl_hang_secs)
		return 0;

	for_each_online_cpu(cpu) {
		struct timer_list *t = per_cpu_ptr(&sl_hang_timer, cpu);

		if (cpu == sl_hang_early_cpu)
			continue;
		timer_setup(t, sl_hang_tick, 0);
		add_timer_on(t, cpu);
		n++;
	}

	pr_info("splashlog: hang watchdog %us armed on %d CPU(s)\n",
		sl_hang_secs, n);
	return 0;
}
late_initcall(splashlog_hang_arm);

static int __init sl_core(void)     { splashlog_stage(2); return 0; }
static int __init sl_postcore(void) { splashlog_stage(3); return 0; }
static int __init sl_arch(void)     { splashlog_stage(4); return 0; }
static int __init sl_subsys(void)   { splashlog_stage(5); return 0; }
static int __init sl_fs(void)       { splashlog_stage(6); return 0; }
static int __init sl_device(void)   { splashlog_stage(7); return 0; }
static int __init sl_late(void)     { splashlog_stage(8); return 0; }

core_initcall(sl_core);
postcore_initcall(sl_postcore);
arch_initcall(sl_arch);
subsys_initcall(sl_subsys);
fs_initcall(sl_fs);
device_initcall(sl_device);
late_initcall(sl_late);

void splashlog_before_init(void)
{
	splashlog_stage(9);

	if (sl_replay_at & 2)
		sl_rp_show();

	if (sl_rp_full && sl_rp_rows && sl_ready)
		sl_halt_forever();
}
