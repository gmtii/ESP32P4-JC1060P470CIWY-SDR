#!/bin/sh
# DMR host test suite. Needs only gcc; the real-capture and dsd-fme steps run
# only if their files are present.
#   DSDCC_SAMPLE=/path/to/dsdcc/samples/dmr_it_8.dis  (github.com/f4exb/dsdcc)
#   DSDFME=/path/to/dsd-fme                           (github.com/lwvmobile/dsd-fme)
#   MBELIB=/path/to/mbelib                            (tools/fetch_mbelib.sh; voice tests)
set -e
cd "$(dirname "$0")"
make -s
echo "== FEC (codes vs dsd-fme generator matrices, BPTC, embedded LC, RS)"
./dmr_fec_test | tail -3
echo "== Synthetic IQ through the full receive chain (signal at -12 kHz)"
for m in bs ms; do
  for snr in 20 8; do
    for fe in 0 800; do
      ./dmr_synth $m /tmp/dmr_t $snr $fe >/dev/null
      r=$(./dmr_host_test -iq /tmp/dmr_t.cf32)
      echo "$m snr=${snr}dB ferr=${fe}Hz: $(echo "$r" | grep -E 'ids yes' | head -1)"
      echo "$r" | grep -q 'TG 214 <- 2140123  alias "EA8DGL"' || { echo FAIL; exit 1; }
    done
  done
done
if [ -n "$DSDCC_SAMPLE" ] && [ -f "$DSDCC_SAMPLE" ]; then
  echo "== Real repeater capture ($DSDCC_SAMPLE); dsd-fme reference: CC4, TS2, TG 19535, SRC 2222223"
  ./dmr_host_test "$DSDCC_SAMPLE" | tail -5
fi
if [ -n "$DSDFME" ] && [ -x "$DSDFME" ]; then
  echo "== Independent decoder on our synthetic encoding (must show TGT=214 SRC=2140123 and the alias)"
  ./dmr_synth bs /tmp/dmr_x 30 0 >/dev/null
  "$DSDFME" -fs -i /tmp/dmr_x.wav -o null 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -E 'TGT|Talker Alias:|ERR' | sort | uniq -c
fi
if [ -n "$MBELIB" ] && [ -f "$MBELIB/mbelib.c" ] && [ -n "$DSDCC_SAMPLE" ] && [ -f "$DSDCC_SAMPLE" ]; then
  echo "== Voice (mbelib): decode the real capture's speech"
  cc -O2 -I../../main/dmr -I"$MBELIB" -include ../../components/mbelib/mbe_fastmath.h -DDMR_VOICE_ENABLED=1 -DDMR_VOICE_AMBE_DUMP dmr_host_test.c ambe_dump.c \
     ../../main/dmr/dmr_fec.c ../../main/dmr/dmr_proto.c ../../main/dmr/dmr_demod.c ../../main/dmr/dmr_voice.c \
     "$MBELIB"/*.c -lm -o dmr_host_test_dump 2>/dev/null
  rm -f ambe_dump.bin
  ./dmr_host_test_dump -voice /tmp/dmr_voice.wav "$DSDCC_SAMPLE" | grep voice:
  if [ -n "$DSDFME" ] && [ -x "$DSDFME" ]; then
    rm -rf /tmp/dmr_amb && mkdir -p /tmp/dmr_amb
    python3 -c "import sys;d=open(sys.argv[1],'rb').read();import wave;w=wave.open('/tmp/dmr_cap.wav','wb');w.setnchannels(1);w.setsampwidth(2);w.setframerate(48000);w.writeframes(d);w.close()" "$DSDCC_SAMPLE"
    "$DSDFME" -fs -i /tmp/dmr_cap.wav -o null -d /tmp/dmr_amb/ >/dev/null 2>&1
    python3 compare_ambe.py ambe_dump.bin /tmp/dmr_amb
  fi
fi
echo "ALL DMR TESTS PASSED"
