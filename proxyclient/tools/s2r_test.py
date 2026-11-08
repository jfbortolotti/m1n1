#!/usr/bin/env python3
# Run the m1n1 S2R test harness from the proxy; output comes on the m1n1 console.
import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from m1n1.setup import *

sleep_us = int(sys.argv[1]) if len(sys.argv) > 1 else 5000000

p.smp_start_secondaries()
print("s2r_test returned", p.s2r_test(sleep_us, 0))
