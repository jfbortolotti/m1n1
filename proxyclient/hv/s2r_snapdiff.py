#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Diff two S2R snapshots ("SNAP ..." lines).

    python3.12 proxyclient/hv/s2r_snapdiff.py <macos snapshot or trace log> <m1n1 console log>

Either file may be a raw snapshot (/tmp/s2r_snap_macos_*.txt), a trace log (lines tagged SNAP)
or a captured m1n1 console (lines prefixed "TTY> "). For power-state registers the target and
actual states are decoded; registers equal on both sides are not printed.
"""
import re, sys

PS_STATES = {0: "OFF", 4: "CLKGATE", 0xf: "ACTIVE"}


def load(path):
    regs, order = {}, []
    for line in open(path, errors="replace"):
        m = re.search(r"SNAP (PS|R) (\S+) ([0-9a-f]+) ([0-9a-f]+)\s*$", line)
        if not m:
            continue
        kind, name, addr, val = m.group(1), m.group(2), int(m.group(3), 16), int(m.group(4), 16)
        if addr not in regs:
            order.append(addr)
        regs[addr] = (kind, name, val)   # last snapshot in the file wins
    return regs, order


def ps(v):
    t, a = v & 0xf, (v >> 4) & 0xf
    s = f"tgt={PS_STATES.get(t, hex(t))} act={PS_STATES.get(a, hex(a))}"
    if v & (1 << 28):
        s += f" AUTO({(v >> 24) & 0xf:x})"
    if v & (1 << 10):
        s += " DEVDIS"
    return s


def main():
    a, ao = load(sys.argv[1])
    b, bo = load(sys.argv[2])
    print(f"{len(a)} registers in {sys.argv[1]}, {len(b)} in {sys.argv[2]}\n")
    print(f"{'name':16} {'addr':>10}  {'macOS':>16}  {'m1n1':>16}")
    for addr in ao + [x for x in bo if x not in a]:
        ka, na, va = a.get(addr, (None, None, None))
        kb, nb, vb = b.get(addr, (None, None, None))
        if va == vb:
            continue
        kind, name = ka or kb, na or nb
        sa = "-" if va is None else f"{va:x}"
        sb = "-" if vb is None else f"{vb:x}"
        extra = ""
        if kind == "PS":
            da = ps(va) if va is not None else "-"
            db = ps(vb) if vb is not None else "-"
            if va is not None and vb is not None and (va & 0xff) == (vb & 0xff) and \
               (va >> 24) == (vb >> 24):
                continue   # only sticky status bits differ
            extra = f"   macOS {da:30} m1n1 {db}"
        print(f"{name:16} {addr:10x}  {sa:>16}  {sb:>16}{extra}")


if __name__ == "__main__":
    main()
