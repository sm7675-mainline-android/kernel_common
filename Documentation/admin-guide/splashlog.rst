=========
Splashlog
=========

Splashlog is an ARM64 early framebuffer console with a persistent boot log.
Enable ``CONFIG_SPLASHLOG=y`` to build it into the kernel; the default is off.
It selects ``FONT_SUPPORT`` and ``FONT_8x16``. The SM7675 target enables the
option, but runtime activation still requires ``spl.enable=1``.

Diagnostic levels
=================

``spl.bootdump=0`` (default)
  Minimal console, compact status and a one-line kernel fault marker. Module
  tracking, module pathname allocation/resolution, evidence summaries, stack
  collection and periodic service sampling are disabled. This does not filter
  the original call trace emitted after a kernel Oops. Routine stack dumps
  before an Oops are suppressed.

``spl.bootdump=1``
  Adds selected evidence, module attempts/failures and bounded module/progress
  summaries. Names are copied before module initialization can release their
  storage. Duplicate loads are counted separately from actual failures.

``spl.bootdump=2``
  Adds kernel fault stacks, blocked/init task context and periodic service
  snapshots. Service sampling runs about every 15 seconds during the first
  600 seconds. These diagnostics can add scheduling and logging overhead.

The former ``/data`` directory scanning has been removed. Splashlog does not
open or enumerate ``/data`` to collect diagnostics. Level 2 can inspect selected
tasks and their existing file descriptors; that is not a filesystem scan.
Block-device enumeration and heartbeat output require their own options.

Reserved memory and display ownership
=====================================

The bootloader framebuffer must be reserved RAM with a valid linear mapping.
The current target defaults are physical address ``0xe3940000``, a
``0x2b00000``-byte reservation, 1240 by 2772 pixels and four bytes per pixel.
These values describe this target, not a portable framebuffer discovery ABI.
Verify the actual bootloader geometry and every frame used for scanout before
enabling writes on another device.

Reserve an independent 1 MiB region for the persistent black box and pass its
physical base with ``spl.bb=<address>``. Its ring header is compatible with the
persistent-RAM layout, but its final 32-byte trailer is private to splashlog.
It must not share any byte with a ramoops record, console, ftrace or pmsg zone,
or with active framebuffer pixels or other reserved-memory users. Configure
ramoops separately; splashlog does not allocate or partition ramoops storage.

The legacy black-box address fallback is framebuffer base plus ``0x2800000``.
That fallback is not evidence that the memory is free: verify all scanout
frames and reserved-memory ranges, or provide a separate reserved region.
The legacy spare boot counter uses framebuffer base plus ``0x2900000`` and
also needs reserved, non-overlapping storage. Endpoint reservation checks
alone do not prove that the whole range has no other owner.
``spl.bb=off`` or ``spl.bb=0`` disables persistent recording.

Previous-boot replay uses separate working storage, and payload cache lines
are cleaned before publishing the persistent write cursor. Persistence across
a reset still depends on the platform retaining that RAM. A cold power loss
can discard it.

The native display driver must call ``splashlog_display_takeover()`` when it
owns scanout. Normal framebuffer writes and redraw timers then stop while
black-box recording continues. ``splashlog_buffer_in_use()`` lets the display
integration retain memory still owned by splashlog. A fault can request a
final snapshot; do not treat takeover as a framebuffer-memory release API.

Boot parameters and compatibility
=================================

The following spellings are registered by ``drivers/misc/splashlog.c``.
``spl.`` and ``splashlog.`` are not interchangeable prefixes.

``spl.enable[=1]``
  Activate splashlog. ``=0`` leaves it disabled on a normal single occurrence.
``spl.bootdump=0|1|2``
  Select the cumulative diagnostic level above. Invalid values select 0.
  ``splashlog.bootdump`` is not a registered alias.
``spl.fb=0|1|2|3``
  0 disables framebuffer writes, 1 uses the available frame buffers (default),
  2 uses one frame with continuous updates, and 3 freezes one frame after
  console replay. Black-box recording is independent of framebuffer rendering.
``splashlog.phys=<address>``
  Set the framebuffer physical base. This does not reserve memory.
``splashlog.width=<pixels>`` / ``splashlog.height=<pixels>``
  Override width (320--4096) and height (320--8192).
``splashlog.scale=1|1.5|2|3``
  Set live glyph scale; the default is 1.5 (12 by 24 pixels). Replay keeps
  its fixed 8 by 16 pixel glyphs.
``spl.bb=<address>|off|0``
  Select the independent reserved black-box region, or disable it.
``splashlog.replay=0|1|2|3|off``
  Select previous-boot replay: bit 0 shows it early and bit 1 before init.
  The default is 3; 0 and ``off`` disable it.
``splashlog.replay_at=0|1|2|3``
  Compatibility spelling for the same numeric replay bitmask. Unlike
  ``splashlog.replay``, this parser does not accept ``off``.
``splashlog.drop=<patterns>`` / ``spl.drop=<patterns>``
  Aliases for the same console drop-pattern filter. Filtering can hide useful
  evidence; it does not restore messages lost before console delivery.
``splashlog.quiet``
  Disable the normal animation timer; it does not silence the kernel console.
``splashlog.mods``
  Request per-module display messages when module diagnostics are enabled.
``spl.beat``
  Explicitly enable heartbeat output and persistent heartbeat samples.
``spl.blk=1|y|Y``
  Explicitly request block-device enumeration during fault handling.

Fault-policy options
====================

These options deliberately change failure or restart behavior. They are
disabled by default and are independent of the diagnostic level. Raising
``spl.bootdump`` does not enable a deadline, hang restart or panic-on-WARN.
Use them only for a diagnostic boot with an understood recovery path.

``spl.deadline=<seconds[.milliseconds]>``
  Arm a boot deadline that queues diagnostics and a warm restart. Zero leaves
  it disabled. The integer seconds are capped at 600; up to three fractional
  digits are accepted. This is a diagnostic timer, not a userspace health
  monitor, and can restart a boot that would otherwise complete.
``spl.deadline_panic[=1]``
  With a deadline, request a diagnostic panic instead of the ordinary deadline
  restart path. ``=0`` disables this policy.
``spl.hang=<seconds>``
  Arm the console-silence watchdog (accepted range 2--300). Silence can be
  normal; this option can restart a functioning system. Omit it to disable.
``splashlog.stall=<seconds>``
  Arm the boot-stall check (0--600; default 0). A qualifying stalled boot with
  blocked tasks deliberately panics.
``splashlog.wp=<milliseconds>``
  Arm ``panic_on_warn`` after the requested uptime; default 0 is disabled.
``splashlog.oops[=continue]``
  Clear ``panic_on_oops`` during splashlog initialization. Continuing after an
  oops can leave corrupted kernel state; this is a diagnostic override.
``spl.reboot_on_oops[=1]``
  Request the diagnostic restart path after an oops. ``=0`` disables it.
``spl.halt=0|1|2``
  0 continues boot, 1 holds replay after a previous panic, and 2 holds any
  available previous log. A held display needs manual restart.

Integration and limits
======================

All direct kernel hooks use ``include/linux/splashlog.h``. With
``CONFIG_SPLASHLOG=n`` they become inline no-ops, and the module-diagnostics
predicate returns false. With diagnostic level 0, ``finit_module()`` follows
the normal loading path without allocating or resolving a diagnostic pathname.

Splashlog retains the kernel's normal ``/dev/kmsg`` rate limit. Userspace
bursts may therefore lose lines before splashlog sees them; raising the global
burst limit is not part of this console integration.

Static interface checks do not establish bootability, crash-path safety or
reserved-memory ownership. Verify the target memory map and perform separate
enabled/disabled build and cold-boot tests before shipping a new integration.
