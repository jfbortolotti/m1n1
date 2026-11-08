#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# analyze_s2r_trace.py -- summarise a trace_s2r_wake_arming.py log
#
# Prints, in order, the PMGR / SMC / GPIO / CPU-impl writes (and the OVRD sysreg writes) done
# in the last --window seconds before the final-WFI anchor, with polling noise removed:
#   - the anchor is, in this order: --anchor-time, the last "ANCHOR" MARK line, the last
#     "Guest is shutting down CPU", the last ACC_OVRD/CYC_OVRD write, else the end of the log
#   - AIC/PS-register repeats with the same value are collapsed (--min-repeat)
#   - plain sysreg traffic (anything other than ACC_/CYC_/OVRD/IPI) is hidden (--all-sysreg)
#
# usage: analyze_s2r_trace.py LOG [--window 1.0] [--tags MMIO,SMC,...] [--all] [--list-anchors]

import argparse
import re
import sys
from collections import Counter, OrderedDict

LINE = re.compile(r"^\s*(\d+\.\d+) (cpu\d+|---)\s+(\w+)\s+(.*)$")
MMIO = re.compile(r"^W(\d+)\s+(\S+?)\+(0x[0-9a-fA-F]+) = (0x[0-9a-fA-F]+)")
INTERESTING_SYSREG = re.compile(r"ACC_|CYC_|OVRD|IPI|WFI|SLEEP|WAKE|DEEP")


def parse(path):
    events = []
    with open(path, errors="replace") as f:
        for raw in f:
            if raw.startswith("#") or not raw.strip():
                continue
            m = LINE.match(raw.rstrip("\n"))
            if not m:
                continue
            t, cpu, tag, text = float(m.group(1)), m.group(2), m.group(3), m.group(4)
            events.append((t, cpu, tag, text))
    return events


def find_anchor(events, forced):
    if forced is not None:
        return forced, "--anchor-time"
    for t, cpu, tag, text in reversed(events):
        if tag == "MARK" and text.startswith("ANCHOR"):
            return t, text
    for t, cpu, tag, text in reversed(events):
        if "Guest is shutting down CPU" in text:
            return t, text
    for t, cpu, tag, text in reversed(events):
        if tag == "SYSREG" and re.search(r"msr (ACC|CYC)_OVRD_EL1", text):
            return t, text
    return events[-1][0], "end of log"


def list_anchors(events):
    for t, cpu, tag, text in events:
        if tag == "MARK" and (text.startswith("ANCHOR") or text.startswith("MANUAL")):
            print(f"{t:10.4f} {cpu} {text}")


def select(events, anchor_t, window, tags, all_sysreg):
    out = []
    for t, cpu, tag, text in events:
        if t < anchor_t - window or t > anchor_t + 1e-9:
            continue
        if tags and tag not in tags:
            continue
        if tag == "SYSREG" and not all_sysreg and not INTERESTING_SYSREG.search(text):
            continue
        out.append((t, cpu, tag, text))
    return out


def collapse(events, min_repeat):
    """Collapse runs of identical (cpu, tag, region+offset = value) lines."""
    res = []
    run_key, run = None, []

    def flush():
        if not run:
            return
        if len(run) >= min_repeat:
            t0, cpu, tag, text = run[0]
            res.append((t0, cpu, tag, text + f"   [x{len(run)}, last at {run[-1][0]:.4f}]"))
        else:
            res.extend(run)

    for ev in events:
        t, cpu, tag, text = ev
        key = None
        if tag == "MMIO":
            m = MMIO.match(text)
            if m:
                key = (cpu, tag, m.group(2), m.group(3), m.group(4))
        if key is not None and key == run_key:
            run.append(ev)
            continue
        flush()
        run_key, run = key, [ev]
    flush()
    return res


def summarise(events):
    c = Counter()
    for t, cpu, tag, text in events:
        if tag == "MMIO":
            m = MMIO.match(text)
            if m:
                c[(m.group(2), m.group(3))] += 1
    return c.most_common(10)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--window", type=float, default=1.0, help="seconds before the anchor (default 1.0)")
    ap.add_argument("--anchor-time", type=float, default=None, help="use this log time as the anchor")
    ap.add_argument("--tags", default="MMIO,SMC,PMP,AOP,SEP,HOT,AIC,DBGTXT,SYSREG,MARK,HV",
                    help="comma list of tags to show (default: all)")
    ap.add_argument("--min-repeat", type=int, default=4, help="collapse >= N identical writes in a row")
    ap.add_argument("--all-sysreg", action="store_true", help="also show non-OVRD sysreg writes")
    ap.add_argument("--no-hv", action="store_true", help="hide HV lines (PT setup, PMGR hook reads)")
    ap.add_argument("--list-anchors", action="store_true")
    a = ap.parse_args()

    events = parse(a.log)
    if not events:
        sys.exit("no events parsed from " + a.log)
    if a.list_anchors:
        list_anchors(events)
        return

    anchor_t, why = find_anchor(events, a.anchor_time)
    tags = set(a.tags.split(",")) if a.tags else None
    if a.no_hv and tags:
        tags.discard("HV")

    sel = select(events, anchor_t, a.window, tags, a.all_sysreg)
    # the PT[...] mapping lines and ADT context are setup noise, never part of the window
    sel = [e for e in sel if not e[3].startswith(("PT[", "tracing writes", "ADT ", "START "))
           and "not in ADT" not in e[3] and ": too large, skipped" not in e[3]]
    sel = collapse(sel, a.min_repeat)

    print(f"# log: {a.log}, {len(events)} events, last event at t={events[-1][0]:.4f}")
    print(f"# anchor t={anchor_t:.4f}: {why}")
    print(f"# window: {a.window:.2f}s before the anchor ({len(sel)} lines after filtering)")
    print("# columns: dt-to-anchor(ms)  cpu  tag  text\n")
    for t, cpu, tag, text in sel:
        print(f"{(t - anchor_t) * 1000:+10.1f} {cpu:<4} {tag:<6} {text}")

    smc = [e for e in sel if e[2] == "SMC" and (">W:" in e[3] or ">RW:" in e[3] or "NOTIFICATION" in e[3])]
    print(f"\n# --- SMC key writes / notifications in window: {len(smc)}")
    for t, cpu, tag, text in smc:
        print(f"{(t - anchor_t) * 1000:+10.1f} {text}")

    top = summarise([e for e in events if anchor_t - a.window <= e[0] <= anchor_t])
    if top:
        print("\n# --- most written MMIO registers in window (candidates for polling noise)")
        for (region, off), n in top:
            print(f"#   {region}{off}: {n} writes")

    sleepish = [e for e in sel if e[2] == "SMC" and "<<<" in e[3]]
    if sleepish:
        print("\n# --- SMC keys flagged as sleep/wake related")
        for t, cpu, tag, text in sleepish:
            print(f"{(t - anchor_t) * 1000:+10.1f} {text}")


if __name__ == "__main__":
    main()
