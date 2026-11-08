#!/usr/bin/env python3
# S2R battery drain test (m1n1 s2r_test with S2R_BATTERY_MAGIC): reading A, unplug the cable,
# Level 3 for N seconds (default 3600), then the results are printed on the screen forever.
# Usage: M1N1TIMEOUT=7200 python3.12 proxyclient/tools/s2r_battery_test.py [l3_seconds] [--panel-on] [--lp[=MASK]]
#   --panel-on : keep the panel on during Level 3 (also with --lp: progress visible on screen)
#   --lidwake  : aop-gpio pin 22 (lid) any-edge IRQ -> AIC 274 wakes L3; ends when the lid is back open (works after slpa)
#   --probe    : 10 s heartbeat, print AIC line / AON GPIO register changes at each wake (find the slpa wake line)
#   --no-settle: skip the 30 s gauge settle before Level 3 (short test loops)
#   --wake     : stay in Level 3 until a short power-button press or lid open (l3_seconds = max)
#   --lp[=MASK]: low-power Level 3 (build m1n1_batt_lp.bin), MASK default 0xff:
#       0x01 DCP fully asleep   0x02 NVMe shutdown       0x04 macOS domains off (E2o)
#       0x08 secondaries in L4  0x10 PS auto mem/fabric  0x20 macOS PS snapshot (E2p)
#       0x40 APCIE_GP/DEBUG off 0x80 ATC0/ATC1 (USB-C) off
#       0x100 SMC sleep keys (slpw, WKTP=2, slpa; needs --wake)  0x200 + SMC SetAPPower(IDLE)
#   NOTE: 0x08 (secondaries L4) sets the GLOBAL deep-sleep bit -> boot core in L4 too -> no wake possible yet
import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))
from m1n1.setup import *

args = [a for a in sys.argv[1:] if not a.startswith("--")]
wake = "--wake" in sys.argv
secs = int(args[0]) if args else (0 if wake else 3600)   # with --wake: max time, 0 = 7 days
magic = 0xBA78 if "--panel-on" in sys.argv else 0xBA77
for a in sys.argv[1:]:
    if a.startswith("--lp"):
        mask = int(a.split("=", 1)[1], 0) if "=" in a else 0xff
        # with --panel-on: panel kept on (DCP sleep bit dropped), each step printed on screen
        magic = (0xBA7B if "--panel-on" in sys.argv else 0xBA7A) | ((mask & 0xff) << 16) | (((mask >> 8) & 3) << 25)
if wake:
    magic |= 1 << 24
if "--no-settle" in sys.argv:
    magic |= 1 << 27
if "--probe" in sys.argv:
    magic |= 1 << 28
if "--lidwake" in sys.argv:
    magic |= 1 << 29
p.smp_start_secondaries()
print("s2r_test returned", p.s2r_test(secs, magic))
