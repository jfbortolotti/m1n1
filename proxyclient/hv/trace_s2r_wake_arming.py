# SPDX-License-Identifier: MIT
#
# trace_s2r_wake_arming.py -- find how macOS arms the AON/SMC/PMGR wake logic before sleep
#
# WHAT THIS IS
#   A hypervisor script (run with `run_guest.py -m`) that boots macOS under the m1n1
#   hypervisor and records, with timestamps and live-flushed to a file, everything the guest
#   does that could arm a wake source for a power-gated boot core:
#
#     MMIO writes  : PMGR (all regs < 1 MiB: PS regs, cpu_start @ pmgr+0x54000, AON PMGR,
#                    cluster regs), per-CPU cpu-impl-reg (RVBAR) and cpu-uttdbg-reg,
#                    GPIO / AOP-GPIO / NUB-GPIO / SMC-GPIO (wake events), WDT, SPMI (PMU),
#                    AIC configuration (target-cpu / config; set S2R_TRACE_AIC=full for all)
#     SMC mailbox  : decoded RTKit/SMC messages: key reads/writes (NTAP, ...), notifications,
#                    results, plus the SMC firmware syslog (e.g. "SMC HID Event: ...")
#     PMP / AOP    : (optional) raw ASC mailbox traffic
#     sysregs      : the hypervisor already traps and logs every guest MSR to an implementation
#                    defined register ("Pass:/Skip:/Shadow: msr NAME, xN = value"); this script
#                    captures those lines, in particular ACC_OVRD_EL1 (deep sleep bit 34) and
#                    CYC_OVRD_EL1 (deep WFI). Plain reads are dropped unless they hit
#                    ACC_/CYC_ registers (S2R_TRACE_SYSREG_READS=1 keeps all of them).
#
#   Every event goes to stdout AND to a line-buffered log file. The hypervisor loses control
#   when the last core goes to sleep, so nothing is kept only in memory on the host. The
#   host-side process (this one) survives the guest going away, so the file is complete up to
#   the last event the hypervisor saw.
#
# HOW THE TRACE ENDS (important)
#   The hypervisor does not let macOS write ACC_OVRD (the write is skipped and logged) and
#   when macOS writes CYC_OVRD with disWfiRetn (bit 0) set, it removes the CPU from the guest
#   ("Guest is shutting down CPU"). So the last events in the log are the last steps macOS
#   does before the final WFI of each core. MMIO writes are traced in ASYNC mode by default
#   (logged right after they happen, the guest is not stopped); S2R_TRACE_SYNC=1 switches to
#   WSYNC where the log line is written BEFORE the write reaches the hardware.
#   The following events are automatically marked as "ANCHOR" lines (tag MARK):
#     - a guest write of ACC_OVRD_EL1 with enDeepSleep (bit 34),
#     - a guest write of CYC_OVRD_EL1 with WFI mode bits (24-25) set,
#     - "Guest is shutting down CPU".
#   On every ANCHOR the last S2R_TRACE_WINDOW seconds (default 3) are dumped to
#   <logfile>.window, so after the run <logfile>.window holds the "last N seconds before the
#   final WFI". The companion tool analyze_s2r_trace.py does a more selective analysis.
#
# HOW TO RUN, trace 5 (boot profile: PMP/SEP from boot, MMIO + AIC added before sleep)
#     cd ~/apple_m1/m1n1
#     S2R_TRACE_PROFILE=boot S2R_TRACE_LOG=/tmp/s2r_trace5.log python3 proxyclient/tools/run_guest.py \
#         -m proxyclient/hv/trace_s2r_wake_arming.py ../macos/macos_13.6/kernel.macho -- \
#         "debug=0x14e serial=3 apcie=0xfffffffe -enable-kprintf-spam wdt=-1 clpc=0"
#   log in, start the delayed sleep in Terminal (sudo -v && (sleep 30; sudo pmset sleepnow) &),
#   then Ctrl-C:  >>> s2r_attach_sleep()   >>> hv.cont()
#
# TRACE 6 (PMP ep 0x20 buffer dump): same command as trace 5 with S2R_TRACE_LOG=/tmp/s2r_trace6.log.
#   The buffer macOS hands the PMP (type 1, DVA 0x3c000) is dumped through dart-pmp at boot to
#   /tmp/s2r_pmp_buf_type1_*.bin and _type2_*.bin (tag PMPBUF in the log), and again at the
#   sleep-time 0x63 command (_cmd8_*). The boot dumps alone are enough; sleep is optional.
#   From the shell: s2r_pmp_dump("manual") dumps it again at any time.
#
# TRACE 9 (SEP set-up, app loading and sleep): same command as trace 5 with
#   S2R_TRACE_LOG=/tmp/s2r_trace9.log. The SEP mailbox is decoded in the SEP's own format
#   (">sep ... ep= tag= type= param= data=" lines) and the buffers behind the cntl OOL set-up,
#   the app endpoints (xarm 0x13, stac 0x18, hdcp 0x0e) and the sleep message are dumped
#   through dart-sep to /tmp/s2r_sep_<n>_<tag>.bin (SEPBUF lines), plus the dart-sep map
#   (SEPMAP). Sleep is needed this time: the sleep-time buffers are part of what we need.
#
# HOW TO RUN (late attach)
#   Tracing from the first instruction slows macOS so much that it reboots before the login
#   screen. Boot WITHOUT this script, log in, then attach it from the hypervisor shell right
#   before going to sleep:
#
#     cd ~/apple_m1/m1n1
#     python3 proxyclient/tools/run_guest.py ../macos/macos_13.6/kernel.macho -- \
#         "debug=0x14e serial=3 apcie=0xfffffffe -enable-kprintf-spam wdt=-1 clpc=0"
#
#   For the PMP (power management processor) mailbox, also set S2R_TRACE_PMP=1 in step 2
#   (traced in ASYNC mode so the guest is not stopped on its idle doorbell writes):
#          >>> os.environ["S2R_TRACE_PMP"] = "1"
#
#   1. Boot macOS and log in; open Terminal and type (do not press Enter yet):
#          sudo pmset sleepnow
#   2. In the run_guest terminal press Ctrl-C to break into the hypervisor shell, then:
#          >>> import os; os.environ["S2R_TRACE_LOG"] = "/tmp/s2r_trace.log"
#          >>> hv.run_script("proxyclient/hv/trace_s2r_wake_arming.py")
#          >>> hv.cont()
#   3. Press Enter in the macOS Terminal (sudo password if asked).
#   4. Wait until the CPUs leave the guest ("Guest is shutting down CPU" / anchors).
#   Afterwards:   python3 proxyclient/hv/analyze_s2r_trace.py /tmp/s2r_trace.log --window 1.0
#
#   Early attach (whole boot traced, much slower) still works with -m:
#     S2R_TRACE_LOG=/tmp/s2r_trace.log python3 proxyclient/tools/run_guest.py \
#         -m proxyclient/hv/trace_s2r_wake_arming.py <kernel.macho> -- "<boot-args>"
#
#   In the hypervisor shell you can also use:
#     s2r_mark("about to sleep")   insert a marker line in the log
#     s2r_window(5)                dump the last 5 s to <logfile>.window now
#     s2r_log_path                 where the log is
#
# OPTIONS (environment variables)
#   S2R_TRACE_LOG            log file (default /tmp/s2r_wake_trace_<date>.log)
#   S2R_TRACE_WINDOW         seconds kept for the .window dump (default 3.0)
#   S2R_TRACE_AIC            "ring" (default: all AIC writes, last S2R_TRACE_AIC_KEEP=400 dumped at
#                            each anchor), "cfg" (config + target regs), "full", "off"
#   S2R_TRACE_PMP            0 (default) / 1: PMP (power management processor) mailbox
#   S2R_TRACE_PMP_RAW        0 (default) / 1: keep the raw PMP doorbell writes (very chatty)
#   S2R_TRACE_SYNC           0 (default, ASYNC) / 1: WSYNC (guest stopped on every write)
#   S2R_TRACE_ECHO_MMIO      0 (default) / 1: also print MMIO lines on the terminal
#   S2R_TRACE_SKIP           comma-separated 16K pages not traced (default: PMP doorbell page)
#   S2R_TRACE_HOT_KEEP       0/1 writes to 0x23b738008 kept per CPU for the anchor dumps (200)
#   S2R_TRACE_SEP            1 (default) / 0: SEP mailbox (ASYNC, raw)
#   S2R_TRACE_SMC_SYSLOG     SMC syslog buffer for late attach (default 0x23fe5c000, "0" = none)
#   S2R_TRACE_SMC_SHMEM      SMC shared memory address for key decoding on late attach
#                            (default 0x23fe56000, "" = learn it from the Initialize reply)
#   S2R_TRACE_AOP            0 (default) / 1: AOP mailbox (very chatty)
#   S2R_TRACE_SPMI           1 (default) / 0: nub-spmi/nub-spmi1 MMIO writes (PMU access)
#   S2R_TRACE_SYSREG_READS   0 (default) / 1
#
# LOG FORMAT (one event per line, easy to grep/parse):
#   <seconds since start> <cpuN|---> <TAG> <text>
#   TAG: MMIO (write), SMC / PMP / AOP (mailbox), SYSREG, HV (hypervisor messages), MARK
#
# KNOWN LIMITATIONS
#   - Reads of MMIO registers are not traced (only writes), except through the mailbox tracers.
#   - Coresight / ED registers are NOT touched or traced (production chips raise an SError).
#   - macOS may never get to the end of its sleep sequence under the hypervisor (it expects
#     PMGR/SMC state changes the hypervisor virtualises or blocks); the log then stops where
#     macOS gave up. The SMC / PMGR writes up to that point are still the ones we want.
#   - The final WFI itself is not trapped (we do not trap WFI), the anchor is the CYC_OVRD
#     / ACC_OVRD write that precedes it by a few instructions.

import atexit
import collections
import datetime
import os
import re
import sys
import time

from m1n1.hv import TraceMode
from m1n1.utils import irange


def _env(name, default):
    return os.environ.get(name, default)


S2R_LOG_PATH = _env("S2R_TRACE_LOG",
                    "/tmp/s2r_wake_trace_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
S2R_WINDOW_S = float(_env("S2R_TRACE_WINDOW", "3.0"))
# "ring" (default since trace 5): the whole AIC is traced (ASYNC) but its writes only go to a
# ring of the last S2R_TRACE_AIC_KEEP writes, dumped at every anchor (AppleInterruptController
# quiesce happens right before the PMU "go hib"); "cfg": config + target regs only (logged);
# "full": every AIC write logged; "off".
S2R_AIC_MODE = _env("S2R_TRACE_AIC", "ring")
S2R_AIC_KEEP = int(_env("S2R_TRACE_AIC_KEEP", "400"))
S2R_PMP = _env("S2R_TRACE_PMP", "0") == "1"
S2R_AOP = _env("S2R_TRACE_AOP", "0") == "1"
# The PMP mailbox tracer logs every raw write of 0x2 to the PMP doorbell (0x23bc34000), one per
# CPU idle exit: 254k lines in the first boot trace, versus ~260 real PMP messages. Those raw
# doorbell writes are counted and summarised unless S2R_TRACE_PMP_RAW=1.
S2R_PMP_RAW = _env("S2R_TRACE_PMP_RAW", "0") == "1"
_RE_PMP_DOORBELL = re.compile(r"MMIO: W\.4\s+0x[0-9a-f]+ = 0x2$")
S2R_SPMI = _env("S2R_TRACE_SPMI", "1") == "1"
S2R_SYSREG_READS = _env("S2R_TRACE_SYSREG_READS", "0") == "1"
# ASYNC: the guest is not stopped on every traced write (events are queued and logged
# shortly after). SYNC/WSYNC stop the guest for each write and make macOS crawl.
S2R_MMIO_MODE = TraceMode.WSYNC if _env("S2R_TRACE_SYNC", "0") == "1" else TraceMode.ASYNC
# Echo MMIO lines to the hypervisor terminal too (slow with a busy guest); they always go
# to the log file.
S2R_ECHO_MMIO = _env("S2R_TRACE_ECHO_MMIO", "0") == "1"

# 16K pages left untraced (passed through to the hardware). Measured on the first run
# (2026-11-03), these three pages produced 99.9% of all events and slowed macOS down so
# much that it rebooted before the login screen:
#   0x23b738000  PMGR[0]+0x38008: per-CPU idle handshake, written on every idle exit
#   0x23bc34000  PMGR[22]+0x8000: PMP mailbox doorbell, rung after each of the above
#   0x23d12c000  PMGR[23]+0x2c004: ASCII byte FIFO (debug log), one write per character
# Trace 4 (2026-11-04): only the PMP doorbell page is skipped by default (the PMP ASC tracer
# covers it when S2R_TRACE_PMP=1). The other two are traced again (ASYNC) and filtered here:
#   0x23b738008: writes of 0/1 are kept in a per-CPU ring (S2R_TRACE_HOT_KEEP, default 200)
#                that is written to the log at every anchor; any other value is logged at once
#   0x23d12c004: bytes are assembled into text lines (tag DBGTXT)
# Override with S2R_TRACE_SKIP="0x...,0x..." (empty string = trace everything).
S2R_PAGE = 0x4000
S2R_SKIP_PAGES = sorted(
    int(x, 16) & ~(S2R_PAGE - 1)
    for x in _env("S2R_TRACE_SKIP", "0x23bc34000").split(",")
    if x.strip())
S2R_HOT_ADDR = 0x23b738008
S2R_HOT_KEEP = int(_env("S2R_TRACE_HOT_KEEP", "200"))
S2R_DBGTXT_ADDR = 0x23d12c004
S2R_SEP = _env("S2R_TRACE_SEP", "1") == "1"

ACC_OVRD_DEEP_SLEEP = 1 << 34


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

class S2RLog:
    def __init__(self, path):
        self.path = path
        self.t0 = time.monotonic()
        self.f = open(path, "w", buffering=1)
        self.ring = collections.deque(maxlen=500000)
        self.last_fsync = 0.0
        self.noisy = {}
        self.anchors = 0
        self.f.write(f"# trace_s2r_wake_arming log, started {datetime.datetime.now().isoformat()}\n")
        self.f.write("# format: <t seconds> <cpu> <TAG> <text>\n")

    def now(self):
        return time.monotonic() - self.t0

    def emit(self, tag, cpu, text):
        t = self.now()
        cpus = "---" if cpu is None else f"cpu{cpu}"
        line = f"{t:10.4f} {cpus:<4} {tag:<6} {text}"
        self.ring.append((t, line))
        if tag not in ("MMIO", "HOT", "AIC") or S2R_ECHO_MMIO:
            print(line)
            sys.stdout.flush()
        try:
            self.f.write(line + "\n")
            self.f.flush()
            if t - self.last_fsync > 1.0 or tag == "MARK":
                os.fsync(self.f.fileno())
                self.last_fsync = t
        except Exception as e:  # never break the guest because of the log
            print(f"(log write failed: {e})")
        return t

    def dump_window(self, seconds, label):
        if not self.ring:
            return
        tend = self.ring[-1][0]
        lines = [l for (t, l) in self.ring if t >= tend - seconds]
        try:
            with open(self.path + ".window", "w") as f:
                f.write(f"# last {seconds:.1f}s up to {label} (t={tend:.4f}), {len(lines)} lines\n")
                for l in lines:
                    f.write(l + "\n")
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            print(f"(window dump failed: {e})")

    def close(self):
        try:
            self.f.flush()
            os.fsync(self.f.fileno())
            self.f.close()
        except Exception:
            pass


s2r_log = S2RLog(S2R_LOG_PATH)
s2r_log_path = S2R_LOG_PATH
atexit.register(s2r_log.close)


def s2r_mark(label="manual"):
    s2r_log.emit("MARK", None, f"MANUAL {label}")


def s2r_window(seconds=None):
    s2r_log.dump_window(seconds or S2R_WINDOW_S, "manual dump")
    print(f"window written to {S2R_LOG_PATH}.window")


_RE_ACC = re.compile(r"Skip: msr ACC_OVRD_EL1, x\d+ = ([0-9a-fA-F]+)")
_RE_CYC = re.compile(r"msr CYC_OVRD_EL1, x\d+ = ([0-9a-fA-F]+)")
_RE_DEV = re.compile(r"^\[(\w+)@(/[\w/\-]+)\] (.*)$")
_RE_SYSREG_MSR = re.compile(r"^(Pass|Skip|Shadow): msr ")
_RE_SYSREG_MRS = re.compile(r"^(Pass|Skip|Shadow): mrs ")


def s2r_anchor(reason):
    s2r_log.anchors += 1
    try:
        s2r_hot_flush(f"ANCHOR #{s2r_log.anchors}")
    except NameError:   # defined further down, anchors cannot fire before that
        pass
    s2r_log.emit("MARK", None, f"ANCHOR #{s2r_log.anchors}: {reason}")
    s2r_log.dump_window(S2R_WINDOW_S, reason)
    try:
        # first anchor + every CPU shutdown (the last one is the state the SoC sleeps with)
        if S2R_SNAPSHOT and (not _s2r_snap_done[0] or "shutting down CPU" in reason):
            _s2r_snap_done[0] = True
            # Sysreg traps (the enDeepSleep anchor) run in the synchronous exception context,
            # where proxy commands work: snapshot right away, before the CPUs shut down. A
            # deferred interrupt is not serviced once the guest CPUs leave (trace 7).
            if hv.ctx is not None and getattr(hv, "_in_handler", False):
                s2r_snapshot(f"anchor{s2r_log.anchors}")
            else:
                s2r_snapshot_later(f"anchor{s2r_log.anchors}")
    except NameError:
        pass


def s2r_classify(text):
    """Return (tag, text) or None to drop the line."""
    m = _RE_DEV.match(text)
    if m:
        dev = m.group(2).rstrip("/").split("/")[-1].upper()
        tag, body = dev.replace("-", "_")[:6], m.group(3)
        if tag == "PMP" and not S2R_PMP_RAW and _RE_PMP_DOORBELL.match(body.strip()):
            n = s2r_log.noisy.get("pmp-doorbell", 0) + 1
            s2r_log.noisy["pmp-doorbell"] = n
            if n % 1000:
                return None
            return tag, f"({n} raw PMP doorbell writes suppressed so far)"
        return tag, body
    if _RE_SYSREG_MRS.match(text):
        if S2R_SYSREG_READS or re.search(r"ACC_|CYC_|OVRD|WFI|SLEEP|WAKE", text):
            return "SYSREG", text
        return None
    if _RE_SYSREG_MSR.match(text):
        return "SYSREG", text
    if text.startswith(("PMGR R ", "CPU STATE R ")):
        # polled by the guest, keep the first few occurrences of each value only
        n = s2r_log.noisy.get(text, 0) + 1
        s2r_log.noisy[text] = n
        if n > 3:
            return None
    return "HV", text


def s2r_check_anchor(tag, text):
    if tag == "SYSREG":
        m = _RE_ACC.search(text)
        if m and int(m.group(1), 16) & ACC_OVRD_DEEP_SLEEP:
            s2r_anchor("guest wrote ACC_OVRD with enDeepSleep: " + text)
            return
        m = _RE_CYC.search(text)
        if m and (int(m.group(1), 16) >> 24) & 3:
            s2r_anchor("guest wrote CYC_OVRD with deep WFI mode: " + text)
            return
    elif tag == "HV" and "Guest is shutting down CPU" in text:
        s2r_anchor(text)


# Route every hv.log() line (sysreg traps, CPUSTART, PT setup, tracer logs) into our log.
_orig_hv_log = hv.log


def s2r_hv_log(s, *args, show_cpu=True, **kwargs):
    text = " ".join([str(s)] + [str(a) for a in args])
    cpu = hv.ctx.cpu_id if (hv.ctx is not None and show_cpu) else None
    for line in text.split("\n"):
        if not line.strip():
            continue
        r = s2r_classify(line)
        if r is None:
            continue
        tag, msg_text = r
        s2r_log.emit(tag, cpu, msg_text)
        s2r_check_anchor(tag, msg_text)
    lf = getattr(hv.print_tracer, "log_file", None)
    if lf:
        lf.write(f"# [cpu{cpu}] {text}\n")


hv.log = s2r_hv_log
hv.show_timestamps = False


# ---------------------------------------------------------------------------
# MMIO write tracing
# ---------------------------------------------------------------------------

_ps_map = {}            # PS register address -> device name
_pmgr_cpustart = None   # (start, end)

PS_STATES = {0xf: "ACTIVE", 0x4: "CLKGATE", 0x0: "PWRGATE"}


def s2r_build_ps_map():
    try:
        pmgr = hv.adt["/arm-io/pmgr"]
        for dev in pmgr.devices:
            ps = pmgr.ps_regs[dev.psreg]
            addr = pmgr.get_reg(ps.reg)[0] + ps.offset + dev.psidx * 8
            _ps_map[addr] = dev.name
    except Exception as e:
        print(f"s2r: PS map unavailable: {e}")


def s2r_decode(name, addr, val):
    dev = _ps_map.get(addr)
    if dev:
        tgt, act = val & 0xf, (val >> 4) & 0xf
        return (f"PS {dev}: target={PS_STATES.get(tgt, hex(tgt))} "
                f"actual={PS_STATES.get(act, hex(act))}"
                + (" RESET" if val & (1 << 31) else "")
                + (" AUTO" if val & (1 << 28) else "")
                + (" DEV_DISABLE" if val & (1 << 10) else ""))
    if name.startswith("PMGR") and _pmgr_cpustart and _pmgr_cpustart[0] <= addr < _pmgr_cpustart[1]:
        off = addr - _pmgr_cpustart[0]
        if off == 4:
            return f"cpu_start select/stop mask {val:#x}"
        if off >= 8:
            return f"cpu_start cluster{(off - 8) // 4} mask {val:#x}"
    return ""


def s2r_sym(pc):
    try:
        s = hv.sym(pc)
        return f" {s}" if s else ""
    except Exception:
        return ""


_hot_ring = {}      # cpu -> deque of (t, line) for 0/1 writes to S2R_HOT_ADDR
_aic_ring = collections.deque(maxlen=S2R_AIC_KEEP)
_hot_count = 0
_dbgtxt = {}        # cpu -> list of chars


def s2r_hot_flush(reason):
    """Write the buffered idle-handshake (and AIC ring) writes to the log (oldest first)."""
    if _aic_ring:
        s2r_log.emit("AIC", None, f"--- last {len(_aic_ring)} AIC writes before: {reason} ---")
        for (t, cpu, line) in _aic_ring:
            s2r_log.emit("AIC", cpu, f"[t={t:.4f}] {line}")
        _aic_ring.clear()
    total = sum(len(d) for d in _hot_ring.values())
    if not total:
        return
    s2r_log.emit("HOT", None, f"--- last {S2R_HOT_KEEP} per CPU 0/1 writes to "
                              f"{S2R_HOT_ADDR:#x} before: {reason} ({_hot_count} so far) ---")
    for cpu in sorted(_hot_ring):
        for (t, line) in _hot_ring[cpu]:
            s2r_log.emit("HOT", cpu, f"[t={t:.4f}] {line}")
        _hot_ring[cpu].clear()


def s2r_mmio_write(evt, name=None, base=None):
    global _hot_count
    bits = 8 << evt.flags.WIDTH
    cpu = evt.flags.CPU

    if evt.addr == S2R_HOT_ADDR and evt.data in (0, 1):
        _hot_count += 1
        ring = _hot_ring.setdefault(cpu, collections.deque(maxlen=S2R_HOT_KEEP))
        ring.append((s2r_log.now(), f"W{bits} {evt.addr:#x} = {evt.data:#x} pc={evt.pc:#x}"))
        if _hot_count % 20000 == 0:
            s2r_log.emit("HOT", None, f"({_hot_count} 0/1 writes to {S2R_HOT_ADDR:#x} so far)")
        return

    if name == "AIC" and S2R_AIC_MODE == "ring":
        _aic_ring.append((s2r_log.now(), cpu,
                          f"W{bits} AIC+{evt.addr - base:#06x} = {evt.data:#x} pc={evt.pc:#x}"))
        return

    if evt.addr == S2R_DBGTXT_ADDR:
        buf = _dbgtxt.setdefault(cpu, [])
        c = evt.data & 0xff
        if c in (0x0a, 0x0d, 0x00) or len(buf) >= 160:
            if buf:
                s2r_log.emit("DBGTXT", cpu, "".join(buf))
                buf.clear()
        elif 0x20 <= c < 0x7f:
            buf.append(chr(c))
        else:
            buf.append(f"\\x{c:02x}")
        return

    off = evt.addr - base
    dec = s2r_decode(name, evt.addr, evt.data)
    s2r_log.emit("MMIO", evt.flags.CPU,
                 f"W{bits:<2} {name}+{off:#07x} = {evt.data:#x} pc={evt.pc:#x}{s2r_sym(evt.pc)}"
                 + (f"  # {dec}" if dec else ""))


_traced = []   # (start, end, name)


def s2r_split_skipped(start, end):
    """Split [start, end) into sub-ranges that avoid the S2R_SKIP_PAGES pages."""
    parts = [(start, end)]
    for page in S2R_SKIP_PAGES:
        pend = page + S2R_PAGE
        out = []
        for (a, b) in parts:
            if pend <= a or b <= page:
                out.append((a, b))
                continue
            if a < page:
                out.append((a, page))
            if pend < b:
                out.append((pend, b))
            s2r_log.emit("HV", None, f"  page {page:#x} left untraced (hot)")
        parts = out
    return parts


def s2r_trace_range(name, start, size, mode=None):
    if mode is None:
        mode = S2R_MMIO_MODE
    end = start + size
    for (s, e, n) in _traced:
        if s <= start and end <= e:
            return False   # already covered by a traced range
    _traced.append((start, end, name))
    for (a, b) in s2r_split_skipped(start, end):
        # base stays the start of the whole region so offsets in the log do not change
        hv.add_tracer(irange(a, b - a), "S2R:" + name, mode,
                      write=s2r_mmio_write, base=start, name=name)
    s2r_log.emit("HV", None, f"tracing writes {name}: {start:#x}-{end:#x} ({mode.name})")
    return True


def s2r_trace_node_regs(path, name, max_size=0x100000, ranges=None):
    try:
        node = hv.adt[path]
    except Exception:
        s2r_log.emit("HV", None, f"{path}: not in ADT, skipped")
        return
    for i in range(len(node.reg)):
        if ranges is not None and i not in ranges:
            continue
        addr, size = node.get_reg(i)
        if size > max_size:
            s2r_log.emit("HV", None, f"{path} reg{i} {addr:#x}+{size:#x}: too large, skipped")
            continue
        s2r_trace_range(f"{name}[{i}]" if len(node.reg) > 1 else name, addr, size)


def s2r_setup_mmio():
    global _pmgr_cpustart
    s2r_build_ps_map()

    # PMGR (+ AON PMGR, cluster regs). cpu_start (pmgr0+0x54000) is already hooked by the
    # hypervisor itself ("CPUSTART W ..." lines come from hv.log).
    s2r_trace_node_regs("/arm-io/pmgr", "PMGR")
    try:
        p0 = hv.adt["/arm-io/pmgr"].get_reg(0)[0]
        _pmgr_cpustart = (p0 + 0x54000, p0 + 0x54020)
    except Exception:
        pass

    # per-CPU implementation regs (RVBAR, ...) and uttdbg
    try:
        for node in list(hv.adt["/cpus"]):
            cid = getattr(node, "cpu_id", "?")
            for prop, label in (("cpu_impl_reg", "CPUIMPL"), ("cpu_uttdbg_reg", "CPUUTT")):
                reg = getattr(node, prop, None)
                if reg:
                    s2r_trace_range(f"{label}{cid}", reg[0], reg[1])
    except Exception as e:
        s2r_log.emit("HV", None, f"cpu regs not traced: {e}")

    # wake related GPIO controllers, WDT, SPMI
    for path, name in (("/arm-io/gpio", "GPIO"), ("/arm-io/aop-gpio", "AOPGPIO"),
                       ("/arm-io/nub-gpio", "NUBGPIO"), ("/arm-io/smc-gpio", "SMCGPIO"),
                       ("/arm-io/wdt", "WDT")):
        s2r_trace_node_regs(path, name, ranges=[0])
    if S2R_SPMI:
        for path, name in (("/arm-io/nub-spmi", "SPMI0"), ("/arm-io/nub-spmi1", "SPMI1"),
                           ("/arm-io/aop-spmi1", "AOPSPMI")):
            s2r_trace_node_regs(path, name, ranges=[0])

    # AIC: IPIs/event/mask regs are hot (every interrupt); config + target regs are not
    if S2R_AIC_MODE != "off":
        try:
            aic0, aic_size = hv.adt["/arm-io/aic"].get_reg(0)
            if S2R_AIC_MODE in ("full", "ring"):
                s2r_trace_range("AIC", aic0, aic_size)
            else:
                s2r_trace_range("AIC-cfg", aic0, 0x2000)
                s2r_trace_range("AIC-target", aic0 + 0x3000, 0x1000)
        except Exception as e:
            s2r_log.emit("HV", None, f"AIC not traced: {e}")


# ---------------------------------------------------------------------------
# Mailbox tracers (SMC with key decoding, PMP/AOP raw)
# ---------------------------------------------------------------------------

S2R_PMP_BUFSZ = int(_env("S2R_TRACE_PMP_BUFSZ", "0x10000"), 16)
S2R_PMP_DUMP_DIR = _env("S2R_TRACE_PMP_DUMP_DIR", "/tmp")
_s2r_pmp_last_dva = [None]


def s2r_pmp_dump(tag, dva=None, size=None):
    """Dump the PMP ep 0x20 buffer (DVA via /arm-io/dart-pmp stream 0) to a .bin file and the log.

    Called automatically on the PMP ep 0x20 type 1 / type 2 / cmd 8 messages; can also be called
    from the hypervisor shell: s2r_pmp_dump("manual") (uses the last DVA seen, default 0x3c000).
    """
    import struct
    from m1n1.hw.dart import DART
    if dva is None:
        dva = _s2r_pmp_last_dva[0] if _s2r_pmp_last_dva[0] is not None else 0x3c000
    _s2r_pmp_last_dva[0] = dva
    size = size or S2R_PMP_BUFSZ
    try:
        dart = DART.from_adt(hv.u, "/arm-io/dart-pmp")
    except Exception as e:
        s2r_log.emit("PMPBUF", None, f"{tag}: dart-pmp not usable: {e}")
        return
    page = 0x4000
    data = bytearray()
    phys = []
    for off in range(0, size, page):
        try:
            ranges = dart.iotranslate(0, dva + off, page)
            pa = ranges[0][0] if ranges else None
        except Exception as e:
            s2r_log.emit("PMPBUF", None, f"{tag}: translate {dva + off:#x} failed: {e}")
            pa = None
        if pa is None:
            phys.append(f"{dva + off:#x}->unmapped")
            break
        phys.append(f"{dva + off:#x}->{pa:#x}")
        data += hv.iface.readmem(pa, page)
    path = f"{S2R_PMP_DUMP_DIR}/s2r_pmp_buf_{tag}_{datetime.datetime.now().strftime('%H%M%S')}.bin"
    try:
        with open(path, "wb") as f:
            f.write(data)
    except Exception as e:
        path = f"(not saved: {e})"
    nz = sum(1 for b in data if b)
    s2r_log.emit("PMPBUF", None, f"{tag}: dva {dva:#x}, {len(data):#x} bytes, {nz} non-zero, "
                 f"pages {' '.join(phys)}, saved {path}")
    # Log the non-zero 16-byte lines (capped) so the trace log alone is useful.
    shown = 0
    for off in range(0, len(data), 16):
        chunk = bytes(data[off:off + 16])
        if not any(chunk):
            continue
        if shown >= 512:
            s2r_log.emit("PMPBUF", None, f"{tag}: ... (more in {path})")
            break
        words = " ".join(f"{w:08x}" for w in struct.unpack("<4I", chunk))
        asc = "".join(chr(c) if 0x20 <= c < 0x7f else "." for c in chunk)
        s2r_log.emit("PMPBUF", None, f"{tag} +{off:05x}: {words}  {asc}")
        shown += 1


_s2r_pmp_pending = []

# Snapshot of the power/clock state (same register list and "SNAP" format as s2r_snapshot() in
# src/s2r.c), taken automatically at the first ANCHOR (the guest's first enDeepSleep write, i.e.
# right before macOS powers the last cores down). Diff with hv/s2r_snapdiff.py.
S2R_SNAPSHOT = _env("S2R_TRACE_SNAPSHOT", "1") == "1"
S2R_SNAP_RANGES = [
    (0x23b040000, 86, 4, 0, "CLK_CFG0"), (0x23b040200, 8, 4, 0, "CLK_CFG1"),
    (0x23b040280, 2, 4, 0, "CLK_CFG2"), (0x23b71c000, 36, 4, 0, "PMGR0_PG"),
    (0x23d29c000, 72, 4, 0, "PMGR1_PG"), (0x23b718000, 2, 4, 0, "PMGR0_718"),
    (0x23b734000, 1, 4, 0, "PMGR0_734"), (0x23b77c000, 1, 4, 0, "PMGR0_77c"),
    (0x23b778000, 1, 4, 0, "PMGR0_778"), (0x23b748000, 4, 0x400, 0, "CPUTVM"),
    (0x23d2c4000, 1, 4, 0, "QUIESCE"), (0x210e20020, 1, 8, 1, "ECPU_20020"),
    (0x211e20020, 1, 8, 1, "PCPU_20020"), (0x210e20660, 1, 8, 1, "ECPU_20660"),
    (0x211e20660, 1, 8, 1, "PCPU_20660"), (0x210e48080, 1, 8, 1, "ECPU_48080"),
    (0x211e48080, 1, 8, 1, "PCPU_48080"), (0x210e40240, 2, 0x10, 1, "ECPU_4024x"),
    (0x211e40240, 2, 0x10, 1, "PCPU_4024x"), (0x210e48400, 1, 8, 1, "ECPU_48400"),
    (0x211e48400, 1, 8, 1, "PCPU_48400"),
]
_s2r_snap_done = [False]


def s2r_snapshot(tag="manual"):
    """Read every PMGR power-state register (ADT devices, die 0) plus S2R_SNAP_RANGES."""
    p = hv.p
    lines = [f"SNAP BEGIN macos {tag}"]
    pm = hv.adt["/arm-io/pmgr"]
    ps_regs = pm.ps_regs
    for d in pm.devices:
        if d.flags.no_ps:
            continue
        try:
            r = ps_regs[d.psreg]
            addr = pm.get_reg(r.reg)[0] + r.offset + (d.psidx << 3)
            lines.append(f"SNAP PS {d.name} {addr:x} {p.read32(addr):08x}")
        except Exception as e:
            lines.append(f"SNAP PSERR {d.name} {e}")
    for base, count, stride, w64, what in S2R_SNAP_RANGES:
        for i in range(count):
            a = base + i * stride
            try:
                v = p.read64(a) if w64 else p.read32(a)
                lines.append(f"SNAP R {what} {a:x} {v:x}")
            except Exception as e:
                lines.append(f"SNAP RERR {what} {a:x} {e}")
    lines.append("SNAP END")
    path = f"{S2R_PMP_DUMP_DIR}/s2r_snap_macos_{tag}_{datetime.datetime.now().strftime('%H%M%S')}.txt"
    try:
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
    except Exception as e:
        path = f"(not saved: {e})"
    for l in lines:
        s2r_log.emit("SNAP", None, l)
    s2r_log.emit("SNAP", None, f"{tag}: {len(lines) - 2} registers, saved {path}")
    print(f"S2R snapshot ({tag}) saved to {path}")


# ---------------------------------------------------------------------------
# TRACE 9: SEP set-up and app loading. The SEP speaks its own 64-bit format (not RTKit):
#   EP = 7:0, TAG = 15:8, TYPE = 23:16, PARAM = 31:24, DATA = 63:32.
# macOS sets up per-endpoint out-of-line buffers over cntl (ep 0): TYPE 2/3 = in/out address
# (DATA in 4 KiB pages, PARAM = endpoint), TYPE 4/5 = in/out size. Every buffer referenced by
# the set-up, by the app loading on xarm (ep 0x13) and by the sleep message is dumped through
# /arm-io/dart-sep stream 0 to S2R_TRACE_PMP_DUMP_DIR/s2r_sep_<n>_<tag>.bin.
# ---------------------------------------------------------------------------
S2R_SEP_DUMP_MAX = int(_env("S2R_TRACE_SEP_DUMPS", "400"))
S2R_SEP_DUMP_CAP = int(_env("S2R_TRACE_SEP_DUMP_CAP", "0x200000"), 16)
_s2r_sep = {"n": 0, "dart": None, "ool": {}, "mapdump": 0}


def _s2r_sep_dart():
    from m1n1.hw.dart import DART
    if _s2r_sep["dart"] is None:
        _s2r_sep["dart"] = DART.from_adt(hv.u, "/arm-io/dart-sep")
    return _s2r_sep["dart"]


def s2r_sep_dump(tag, iova, size):
    """Dump [iova, iova+size) of the SEP's DART stream 0 (unmapped pages are reported)."""
    import struct
    if _s2r_sep["n"] >= S2R_SEP_DUMP_MAX:
        return
    _s2r_sep["n"] += 1
    n = _s2r_sep["n"]
    size = min(size, S2R_SEP_DUMP_CAP)
    page = 0x4000
    start = iova & ~(page - 1)
    end = (iova + size + page - 1) & ~(page - 1)
    try:
        dart = _s2r_sep_dart()
    except Exception as e:
        s2r_log.emit("SEPBUF", None, f"#{n} {tag}: dart-sep not usable: {e}")
        return
    data = bytearray()
    phys = []
    for off in range(start, end, page):
        pa = None
        try:
            r = dart.iotranslate(0, off, page)
            pa = r[0][0] if r else None
        except Exception:
            pa = None
        if pa is None:
            phys.append(f"{off:#x}->unmapped")
            data += bytes(page)
            continue
        phys.append(f"{off:#x}->{pa:#x}")
        data += hv.iface.readmem(pa, page)
    data = data[iova - start:iova - start + size]
    path = f"{S2R_PMP_DUMP_DIR}/s2r_sep_{n:03d}_{tag}.bin"
    try:
        with open(path, "wb") as f:
            f.write(data)
    except Exception as e:
        path = f"(not saved: {e})"
    nz = sum(1 for b in data if b)
    shown_phys = " ".join(phys[:6]) + (" ..." if len(phys) > 6 else "")
    s2r_log.emit("SEPBUF", None, f"#{n} {tag}: iova {iova:#x} size {size:#x}, {nz} non-zero, "
                 f"{shown_phys}, saved {path}")
    shown = 0
    for off in range(0, len(data), 16):
        chunk = bytes(data[off:off + 16])
        if not any(chunk):
            continue
        if shown >= 32:
            s2r_log.emit("SEPBUF", None, f"#{n} ... (more in {path})")
            break
        words = " ".join(f"{w:08x}" for w in struct.unpack("<4I", chunk.ljust(16, b"\0")))
        asc = "".join(chr(c) if 0x20 <= c < 0x7f else "." for c in chunk)
        s2r_log.emit("SEPBUF", None, f"#{n} +{off:05x}: {words}  {asc}")
        shown += 1


def s2r_sep_dump_map(tag):
    """Log the whole dart-sep stream 0 mapping table (DART.dump_device output)."""
    import io, contextlib
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            _s2r_sep_dart().dump_device(0)
    except Exception as e:
        buf.write(f"dump_device failed: {e}")
    for line in buf.getvalue().splitlines():
        s2r_log.emit("SEPMAP", None, f"{tag}: {line}")
    s2r_log.emit("SEPMAP", None, f"{tag}: end of dart-sep stream 0 map")


def s2r_sep_later(fn, what):
    _s2r_pmp_pending.append(("__call__", fn))
    hv.interrupt()


def s2r_snapshot_later(tag):
    _s2r_pmp_pending.append(("__snap__", tag))
    s2r_log.emit("SNAP", None, f"{tag}: snapshot queued")
    hv.interrupt()


def s2r_pmp_dump_later(tag, dva):
    """Queue a PMP buffer dump and interrupt the guest so it runs in a synchronous context."""
    _s2r_pmp_pending.append((tag, dva))
    s2r_log.emit("PMPBUF", None, f"{tag}: dump queued (dva {dva if dva is None else hex(dva)})")
    hv.interrupt()


def s2r_user_irq(reason, code, info):
    """USER_INTERRUPT handler: run queued PMP dumps and resume the guest; Ctrl-C still gives the shell."""
    if not _s2r_pmp_pending or hv._sigint_pending:
        return hv.handle_exception(reason, code, info)
    from m1n1.proxy import EXC_RET
    import traceback
    hv.exc_info = info
    hv._in_handler = True
    try:
        hv._load_context()
        while _s2r_pmp_pending:
            tag, dva = _s2r_pmp_pending.pop(0)
            if tag == "__snap__":
                s2r_snapshot(dva)
            elif tag == "__call__":
                dva()
            else:
                s2r_pmp_dump(tag, dva)
    except Exception:
        s2r_log.emit("PMPBUF", None, "dump failed: " + traceback.format_exc().replace("\n", " | "))
    try:
        hv._commit_context()
    except Exception:
        pass
    hv.ctx = None
    hv.exc_orig_cpu = None
    hv.p.exit(EXC_RET.HANDLED)
    hv._in_handler = False
    if hv._sigint_pending:
        hv._handle_sigint()


def s2r_install_user_irq():
    from m1n1.proxy import START
    from m1n1.hv.types import HV_EVENT
    hv.iface.set_handler(START.HV, HV_EVENT.USER_INTERRUPT, s2r_user_irq)


def s2r_setup_mailboxes():
    from m1n1.trace.asc import ASCTracer, EP, msg, msg_log, DIR, Syslog
    from m1n1.fw.asc.syslog import Syslog_Log
    import struct

    # Late attach misses the syslog Init/GetBuf messages sent at boot, and the stock Syslog
    # endpoint then raises on the first Log message. Fall back to preset values (seen in the
    # 2026-11-03 trace for the SMC) or log a placeholder line.
    class S2RSyslog(Syslog):
        PRESET_BUF = None
        PRESET_ENTRYSIZE = None

        @msg(5, DIR.RX, Syslog_Log)
        def Log(self, m):
            buf = getattr(self.state, "syslog_buf", None) or self.PRESET_BUF
            esz = getattr(self.state, "entrysize", None) or self.PRESET_ENTRYSIZE
            if not buf or not esz:
                self.log(f"* syslog entry #{m.INDEX} (buffer unknown: attached after GetBuf)")
                return True
            # Behind a DART (PMP: DVA 0x34000) the buffer address is an IOVA: reading it as a
            # physical address makes m1n1 itself take a data abort. Only read buffers in the
            # physical SoC/DRAM range (the SMC's is SRAM at 0x23fe5c000).
            if buf < 0x200000000:
                self.log(f"* syslog entry #{m.INDEX} (buffer DVA {buf:#x} behind the IOMMU)")
                return True
            try:
                stride = 0x20 + esz
                raw = self.tracer.ioread(buf + m.INDEX * stride, stride)
                hdr, unk, context, logmsg = struct.unpack(f"<II24s{esz}s", raw)
                context = context.split(b"\x00")[0].decode("ascii", "replace")
                logmsg = logmsg.split(b"\x00")[0].decode("ascii", "replace").rstrip("\n")
                self.log(f"* [{context}]{logmsg}")
            except Exception as e:
                self.log(f"* syslog entry #{m.INDEX} (read failed: {e})")
            return True

    class S2RSMCSyslog(S2RSyslog):
        PRESET_BUF = int(_env("S2R_TRACE_SMC_SYSLOG", "0x23fe5c000"), 16) or None
        PRESET_ENTRYSIZE = 0x60
    from m1n1.fw.smc import (SMCMessage, SMCInitialize, SMCWriteKey, SMCReadKey,
                             SMCReadWriteKey, SMCGetKeyInfo, SMCGetKeyByIndex, SMCResult,
                             SMC_INITIALIZE, SMC_NOTIFICATION, SMC_WRITE_KEY, SMC_READ_KEY,
                             SMC_RW_KEY, SMC_GET_KEY_INFO, SMC_GET_KEY_BY_INDEX)
    from m1n1.utils import Register64

    # keys whose names suggest sleep/wake/AP-power semantics get a "<<<" flag in the log
    INTERESTING = {"NTAP", "MSLD", "bHLD", "SLPS", "WKRS", "AWAK", "MBSE", "clam", "LSOF",
                   "ACFP", "CHLS", "AC-W", "PMUC", "MSDW", "WOLC", "ASTE", "sleep"}

    def keyname(k):
        return k.to_bytes(4, "big").decode("ascii", errors="replace")

    class S2RSMCEp(EP):
        BASE_MESSAGE = SMCMessage

        def __init__(self, tracer, epid):
            super().__init__(tracer, epid)
            # Late attach misses the Initialize reply that carries the shared memory address:
            # preset the value seen on this machine (2026-11-03 boot), S2R_TRACE_SMC_SHMEM
            # overrides it ("" = learn it from the Initialize reply only).
            shm = _env("S2R_TRACE_SMC_SHMEM", "0x23fe56000")
            self.state.sram_addr = int(shm, 16) if shm else None
            self.state.verbose = 1
            self.state.rb = {}

        def flag(self, key):
            return "  <<< sleep/wake related?" if key in INTERESTING else ""

        Initialize = msg_log(SMC_INITIALIZE, DIR.TX, SMCInitialize)

        @msg(SMC_NOTIFICATION, DIR.RX, SMCResult)
        def Notification(self, m):
            self.log(f"NOTIFICATION {m.VALUE:#010x} (type {m.VALUE >> 16:#x}, "
                     f"d1={(m.VALUE >> 8) & 0xff:#x}, d2={m.VALUE & 0xff:#x})")
            return True

        @msg(SMC_WRITE_KEY, DIR.TX, SMCWriteKey)
        def WriteKey(self, m):
            key = keyname(m.KEY)
            self.state.rb[m.ID] = m.TYPE, key, m.SIZE
            data = self.hv.iface.readmem(self.state.sram_addr, m.SIZE) if self.state.sram_addr else b""
            self.log(f"[{m.ID:x}] >W: <{key}> = {data.hex()} ({m.SIZE}){self.flag(key)}")
            return True

        @msg(SMC_READ_KEY, DIR.TX, SMCReadKey)
        def ReadKey(self, m):
            key = keyname(m.KEY)
            self.state.rb[m.ID] = m.TYPE, key, m.SIZE
            self.log(f"[{m.ID:x}] >R: <{key}> ({m.SIZE}){self.flag(key)}")
            return True

        @msg(SMC_RW_KEY, DIR.TX, SMCReadWriteKey)
        def ReadKeyPayload(self, m):
            key = keyname(m.KEY)
            self.state.rb[m.ID] = m.TYPE, key, m.RSIZE
            data = self.hv.iface.readmem(self.state.sram_addr, m.WSIZE) if self.state.sram_addr else b""
            self.log(f"[{m.ID:x}] >RW: <{key}> = {data.hex()} ({m.WSIZE},{m.RSIZE}){self.flag(key)}")
            return True

        @msg(SMC_GET_KEY_INFO, DIR.TX, SMCGetKeyInfo)
        def GetInfo(self, m):
            key = keyname(m.KEY)
            self.state.rb[m.ID] = m.TYPE, key, None
            return True   # key-info probing is noise

        @msg(SMC_GET_KEY_BY_INDEX, DIR.TX, SMCGetKeyByIndex)
        def GetKeyByIndex(self, m):
            self.state.rb[m.ID] = m.TYPE, m.INDEX, None
            return True   # key enumeration is noise

        @msg(None, DIR.RX, Register64)
        def RXMsg(self, m):
            # the Initialize reply is a page aligned DRAM address, never a valid result
            # (a result has RESULT/ID in bits 0-15 and a small SIZE in bits 16-31)
            looks_like_shmem = ((m.value >> 32) in (0x2, 0x3) and (m.value & 0xfff) == 0
                                and ((m.value >> 16) & 0xffff) >= 0x100)
            if self.state.sram_addr is None or looks_like_shmem:
                self.log(f"shared memory address: {m.value:#x}")
                self.state.sram_addr = m.value
                return True

            r = SMCResult(m.value)
            if r.RESULT != 0:
                self.log(f"[{r.ID:x}] <Err: {r.RESULT:#04x}")
                return True

            if r.ID in self.state.rb:
                mtype, key, size = self.state.rb.pop(r.ID)
                if mtype in (SMC_READ_KEY, SMC_RW_KEY):
                    if size is not None and size <= 4:
                        data = hex(r.VALUE)
                    else:
                        data = self.hv.iface.readmem(self.state.sram_addr, r.SIZE).hex()
                    self.log(f"[{r.ID:x}] <R: <{key}> = {data}")
                    return True
                if mtype == SMC_WRITE_KEY:
                    self.log(f"[{r.ID:x}] <W ok: <{key}>")
                    return True
                return True   # info / index replies: noise
            self.log(f"[{r.ID:x}] <OK {r!r}")
            return True

    class S2RSMCTracer(ASCTracer):
        ENDPOINTS = {2: S2RSMCSyslog, 0x20: S2RSMCEp}

    # ASCTracer traps its mailbox page in SYNC mode (the guest stops on every access). The
    # PMP page also holds the doorbell written on every CPU idle exit, which made macOS too
    # slow to ever reach sleep. ASYNC: accesses are logged in order without stopping the
    # guest; the mailbox messages are still decoded (from the queued read/write values).
    class S2RAsyncASCTracer(ASCTracer):
        DEFAULT_MODE = TraceMode.ASYNC
        ENDPOINTS = {2: S2RSyslog}

    # PMP endpoint 0x20: macOS hands the PMP a buffer with the type-1 message
    # (0x0001000000000000 | DVA, trace 5: DVA 0x3c000). m1n1 sending the same DVA with a zeroed
    # buffer crashes the PMP, so the content matters. Translate the DVA through /arm-io/dart-pmp
    # (the live page tables, stream 0) and dump the buffer at type 1 (what macOS wrote), at the
    # type-2 reply (what the PMP changed) and at the sleep-time 0x63 command.
    class S2RPMPEp20(EP):
        NAME = "pmpep20"

        def __init__(self, tracer, epid):
            super().__init__(tracer, epid)
            self.state.buf_dva = None
            self.state.ndump = 0

        def handle_msg(self, direction, r0, r1):
            v = r0.value
            d = ">" if direction == DIR.TX else "<"
            self.tracer.log(f"{d}ep:20 {v:016x}")
            typ = (v >> 48) & 0xff
            cmd = (v >> 44) & 0xf
            # This runs in the ASYNC context (guest running): proxy commands are not allowed
            # here, so queue the dump and stop the guest; s2r_user_irq() does it.
            if direction == DIR.TX and typ == 1:
                self.state.buf_dva = v & 0xffffffffff
                s2r_pmp_dump_later("type1", self.state.buf_dva)
            elif direction == DIR.RX and typ == 2:
                s2r_pmp_dump_later("type2", self.state.buf_dva)
            elif direction == DIR.TX and cmd == 0x8:
                s2r_pmp_dump_later(f"cmd8_{v & 0xffffffff:08x}", self.state.buf_dva)
            return True

    class S2RPMPTracer(S2RAsyncASCTracer):
        ENDPOINTS = {2: S2RSyslog, 0x20: S2RPMPEp20}

    class S2RSEPTracer(S2RAsyncASCTracer):
        """SEP mailbox in the SEP's own format, with buffer dumps (trace 9)."""

        def handle_msg(self, direction, r0, r1):
            v = r0.value
            ep, tag, typ, prm, data = v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, \
                (v >> 24) & 0xff, v >> 32
            d = ">" if direction == DIR.TX else "<"
            self.log(f"{d}sep {v:016x} ep={ep:02x} tag={tag:02x} type={typ:02x} "
                     f"param={prm:02x} data={data:08x}")
            st = _s2r_sep
            ool = st["ool"]
            dumps = []
            if direction == DIR.TX and ep == 0xfe and typ == 0x18:        # SET_SHMEM
                dumps.append((f"shmem", data << 12, 0x40000))
            elif direction == DIR.TX and ep == 0xff and typ == 0x06:      # BOOT_IMG4
                self.log(f"sepOS image at iova {data << 12:#x}")
            elif direction == DIR.TX and ep == 0 and typ in (2, 3, 4, 5):
                e = ool.setdefault(prm, {})
                e[{2: "in", 3: "out", 4: "insz", 5: "outsz"}[typ]] = \
                    (data << 12) if typ in (2, 3) else data
                if typ in (2, 3):
                    io_ = "in" if typ == 2 else "out"
                    sz = e.get(io_ + "sz", 0x8000)
                    dumps.append((f"ool_{prm:02x}_{io_}_setup", data << 12, sz))
            elif ep == 0 and typ == 0x0c and direction == DIR.TX:         # sleep
                for e_, b in sorted(ool.items()):
                    for io_ in ("in", "out"):
                        if io_ in b:
                            dumps.append((f"ool_{e_:02x}_{io_}_sleep", b[io_],
                                          b.get(io_ + "sz", 0x8000)))
                if st["mapdump"] < 3:
                    st["mapdump"] += 1
                    s2r_sep_later(lambda: s2r_sep_dump_map("sleep"), "map")
            elif ep not in (0, 0xfd, 0xfe, 0xff):
                # an app endpoint (xarm 0x13, stac 0x18, hdcp 0x0e, ...): its OOL buffers, and
                # the region DATA points to when it looks like a page number
                b = ool.get(ep, {})
                io_ = "in" if direction == DIR.TX else "out"
                if io_ in b:
                    dumps.append((f"ep{ep:02x}_{d == '>' and 'tx' or 'rx'}_{v & 0xffffffff:08x}"
                                  f"_{io_}", b[io_], b.get(io_ + "sz", 0x8000)))
                if direction == DIR.TX and data > 0x10 and ep in (0x13, 0x18, 0x0e):
                    # xarm app loading: DATA looks like an IOVA page; 1 MiB is a guess
                    dumps.append((f"ep{ep:02x}_data_{data:x}", data << 12,
                                  0x100000 if ep == 0x13 else 0x10000))
                if ep == 0x13 and st["mapdump"] < 1:
                    st["mapdump"] += 1
                    s2r_sep_later(lambda: s2r_sep_dump_map("first_xarm"), "map")
            for t, iova, sz in dumps:
                s2r_sep_later(lambda t=t, iova=iova, sz=sz: s2r_sep_dump(t, iova, sz), t)
            return

    tracers = []
    t = S2RSMCTracer(hv, "/arm-io/smc", verbose=1)
    t.start()
    tracers.append(t)

    if S2R_PMP or S2R_SNAPSHOT or S2R_SEP:
        s2r_install_user_irq()
    if S2R_PMP:
        try:
            p = S2RPMPTracer(hv, "/arm-io/pmp", verbose=1)
            p.start()
            tracers.append(p)
        except Exception as e:
            s2r_log.emit("HV", None, f"PMP mailbox not traced: {e}")
    if S2R_SEP:
        try:
            sp = S2RSEPTracer(hv, "/arm-io/sep", verbose=1)
            sp.start()
            tracers.append(sp)
        except Exception as e:
            s2r_log.emit("HV", None, f"SEP mailbox not traced: {e}")
    if S2R_AOP:
        try:
            a = S2RAsyncASCTracer(hv, "/arm-io/aop", verbose=1)
            a.start()
            tracers.append(a)
        except Exception as e:
            s2r_log.emit("HV", None, f"AOP mailbox not traced: {e}")
    return tracers


# ---------------------------------------------------------------------------
# Context from the ADT (static info that helps interpret the trace)
# ---------------------------------------------------------------------------

def s2r_log_adt_context():
    for path in ("/arm-io/gpio", "/arm-io/aop-gpio", "/arm-io/nub-gpio", "/arm-io/smc-gpio"):
        try:
            n = hv.adt[path]
            we = getattr(n, "wake_events", None)
            s2r_log.emit("HV", None,
                         f"ADT {path}: wake-events={we!r} "
                         f"wake-no-interrupt-group={getattr(n, 'wake_no_interrupt_group', None)!r} "
                         f"interrupts={getattr(n, 'interrupts', None)!r}")
        except Exception:
            pass
    try:
        n = hv.adt["/arm-io/smc"]
        s2r_log.emit("HV", None, f"ADT /arm-io/smc interrupts={getattr(n, 'interrupts', None)!r}")
    except Exception:
        pass
    try:
        for node in list(hv.adt["/cpus"]):
            s2r_log.emit("HV", None,
                         f"ADT cpu{getattr(node, 'cpu_id', '?')} reg={getattr(node, 'reg', None)!r} "
                         f"impl={getattr(node, 'cpu_impl_reg', None)!r} "
                         f"uttdbg={getattr(node, 'cpu_uttdbg_reg', None)!r}")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Go
# ---------------------------------------------------------------------------

# S2R_TRACE_PROFILE:
#   "full" (default): everything is traced from the moment the script runs (late attach).
#   "boot" (trace 5, early attach with run_guest.py -m): only the light ASYNC mailbox tracers
#          (PMP, SEP) and the SMC tracer run from the start, so the boot-time PMP/SEP set-up
#          (e.g. how macOS opens PMP endpoint 0x20) is captured without slowing the boot down.
#          After logging in: Ctrl-C, then s2r_attach_sleep(), then hv.cont(), then sleep: this
#          adds the MMIO tracers (PMGR, GPIO, SPMI, AIC ring, idle page, debug FIFO text).
S2R_PROFILE = _env("S2R_TRACE_PROFILE", "full")
_s2r_mmio_attached = False


def s2r_attach_sleep():
    """Add the MMIO tracers (boot profile): call from the hypervisor shell right before sleep."""
    global _s2r_mmio_attached
    if _s2r_mmio_attached:
        print("MMIO tracers already attached")
        return
    s2r_mark("attach MMIO tracers for the sleep sequence")
    s2r_setup_mmio()
    _s2r_mmio_attached = True
    print("MMIO tracers added, they take effect when the guest continues (hv.cont()).")


if S2R_PROFILE == "boot":
    S2R_PMP = True   # the PMP boot-time set-up is the point of this profile

s2r_log.emit("MARK", None, f"START log={S2R_LOG_PATH} profile={S2R_PROFILE} "
                           f"window={S2R_WINDOW_S}s aic={S2R_AIC_MODE} "
                           f"pmp={int(S2R_PMP)} sep={int(S2R_SEP)} aop={int(S2R_AOP)} "
                           f"spmi={int(S2R_SPMI)}")
s2r_log_adt_context()
if S2R_PROFILE != "boot":
    s2r_setup_mmio()
    _s2r_mmio_attached = True
try:
    s2r_tracers = s2r_setup_mailboxes()
except Exception as e:
    import traceback
    traceback.print_exc()
    s2r_log.emit("HV", None, f"mailbox tracers NOT installed: {e}")

print("\n" + "=" * 70)
print(f"S2R wake-arming trace ready (profile {S2R_PROFILE}). Log: {S2R_LOG_PATH}")
if S2R_PROFILE == "boot":
    print("Only the SMC/PMP/SEP mailboxes are traced now. After logging in to macOS:")
    print("    sudo -v && (sleep 30; sudo pmset sleepnow) &      (in a macOS terminal)")
    print("then Ctrl-C here and:   s2r_attach_sleep()   then   hv.cont()")
else:
    print("Continue the guest, boot macOS, then run   sudo pmset sleepnow   in a macOS terminal.")
print("Anchors (deep ACC_OVRD / CYC_OVRD / CPU shutdown) are marked and the last")
print(f"{S2R_WINDOW_S:.0f}s before them go to {S2R_LOG_PATH}.window")
print("=" * 70 + "\n")
