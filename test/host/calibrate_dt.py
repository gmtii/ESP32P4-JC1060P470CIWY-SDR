#!/usr/bin/env python3
"""
Calibrates FT8_DT_BIAS_S (ft8_decoder.h): runs ft8_host_test on every corpus
WAV that has a WSJT-X reference .txt next to it, pairs our decodes with the
reference by message text, and reports our DT minus WSJT-X's DT. With
FT8_DT_BIAS_S already set correctly the median should be ~0.

usage: ./calibrate_dt.py path/to/ft8_lib/test/wav
"""
import glob, os, re, statistics, subprocess, sys

root = sys.argv[1]
diffs = []
for wav in sorted(glob.glob(os.path.join(root, "**", "*.wav"), recursive=True)):
    ref = wav[:-4] + ".txt"
    if not os.path.exists(ref):
        continue
    out = subprocess.run(["./ft8_host_test", wav], capture_output=True, text=True).stdout
    ours = {}
    for line in out.splitlines():
        m = re.match(r"\d\d:\d\d \d{4}Hz [+-]\d\ddB [+-]\d\.\d ~ (.*?)( \d{4}km)?\t([+-]\d+\.\d+)$", line)
        if m:
            ours[m.group(1).strip()] = float(m.group(3))
    for line in open(ref):
        f = line.split()
        if len(f) < 6 or f[4] != "~":
            continue
        try:
            dt_ref = float(f[2])
        except ValueError:
            continue
        msg = " ".join(f[5:])
        for k, v in ours.items():
            if msg.startswith(k):
                diffs.append(v - dt_ref)
                break
if not diffs:
    sys.exit("no matched decodes")
diffs.sort()
print(f"matched {len(diffs)} decodes")
print(f"our DT - WSJT-X DT: median {statistics.median(diffs):+.3f} s, "
      f"mean {statistics.mean(diffs):+.3f} s, stdev {statistics.pstdev(diffs):.3f} s")
print(f"10/90 percentile: {diffs[len(diffs)//10]:+.2f} / {diffs[9*len(diffs)//10]:+.2f} s")
