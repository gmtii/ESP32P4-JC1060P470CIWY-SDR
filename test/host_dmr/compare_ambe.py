#!/usr/bin/env python3
"""Bit-exact comparison of decoded AMBE+2 parameter frames: ours
(ambe_dump.bin, written by the DMR_VOICE_AMBE_DUMP build) against dsd-fme's
per-call .amb files (dsd-fme -d DIR). Both use the .amb frame layout:
err byte + 7 bytes holding the 49 parameter bits.
usage: compare_ambe.py ambe_dump.bin dsdfme_amb_dir"""
import glob, os, sys
ours = open(sys.argv[1], 'rb').read()
ref = b''.join(open(f, 'rb').read()[4:] for f in sorted(glob.glob(os.path.join(sys.argv[2], '*.amb'))))
O = [ours[i:i + 8] for i in range(0, len(ours), 8)]
R = [ref[i:i + 8] for i in range(0, len(ref), 8)]
m, off = max((sum(1 for i in range(len(O)) if 0 <= i + o < len(R) and O[i][1:] == R[i + o][1:]), o) for o in range(-80, 81))
pairs = [(O[i], R[i + off]) for i in range(len(O)) if 0 <= i + off < len(R)]
diff = [(o, r) for o, r in pairs if o[1:] != r[1:]]
clean_diff = sum(1 for o, r in diff if o[0] == 0 and r[0] == 0)
print(f"frames: ours {len(O)}, dsd-fme {len(R)}; identical {m}/{len(pairs)} ({100 * m / len(pairs):.1f} %)")
print(f"differing frames: {len(diff)}, of which {clean_diff} with no FEC errors reported by either decoder"
      " (= unprotected C2/C3 bits sliced differently)")
