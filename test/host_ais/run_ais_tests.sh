#!/bin/sh
# AIS host tests. Needs only gcc; the AIS-catcher cross-check runs if
# AISCATCHER=/path/to/AIS-catcher is set (github.com/jvde-github/AIS-catcher).
set -e
cd "$(dirname "$0")"
make -s
echo "== 6 known messages on both channels (board orientation, 30 dB)"
./ais_synth /tmp/ais_t.cf32 192000 30 0 >/dev/null
./ais_host_test /tmp/ais_t.cf32 | tee /tmp/ais_t.txt
grep -q 'decoded 6 ' /tmp/ais_t.txt || { echo FAIL; exit 1; }
echo "== sensitivity (180 messages per point, +500 Hz)"
for snr in 20 15 12 10; do
  AIS_SYNTH_REPS=30 ./ais_synth /tmp/ais_s.cf32 192000 $snr 500 >/dev/null
  echo "SNR $snr dB: $(./ais_host_test /tmp/ais_s.cf32 | tail -1 | sed 's/.*decoded \([0-9]*\).*/\1/')/180"
done
if [ -n "$AISCATCHER" ] && [ -x "$AISCATCHER" ]; then
  echo "== independent decoder on our synthetic signal (standard orientation)"
  ./ais_synth /tmp/ais_n.cf32 192000 30 0 normal >/dev/null
  "$AISCATCHER" -r CF32 /tmp/ais_n.cf32 -s 192000 -o 1 2>/dev/null | grep '^!AIVDM' | wc -l | sed 's/^/NMEA sentences (6 messages, type 5 takes two): /'
fi
echo "ALL AIS TESTS PASSED"
