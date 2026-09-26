# Host build of the ESP32-P4 DMR demodulator/decoder (main/dmr) for tests.
DMR = ../../main/dmr
CFLAGS = -O2 -Wall -Wextra -Werror -I$(DMR)
CORE = $(DMR)/dmr_fec.c $(DMR)/dmr_proto.c $(DMR)/dmr_demod.c $(DMR)/dmr_voice.c
# Voice build: point MBELIB at a checkout of mbelib (tools/fetch_mbelib.sh puts
# one in components/mbelib/mbelib), then `make dmr_host_test_voice`.
MBELIB ?= ../../components/mbelib/mbelib
MBE_SRC = $(wildcard $(MBELIB)/*.c)

all: dmr_fec_test dmr_host_test dmr_synth

dmr_synth: dmr_synth.c $(DMR)/dmr_fec.c $(wildcard $(DMR)/*.h)
	$(CC) $(CFLAGS) $< $(DMR)/dmr_fec.c -lm -o $@

dmr_fec_test: dmr_fec_test.c $(DMR)/dmr_fec.c $(wildcard $(DMR)/*.h)
	$(CC) $(CFLAGS) $< $(DMR)/dmr_fec.c -o $@

dmr_host_test: dmr_host_test.c $(CORE) $(wildcard $(DMR)/*.h)
	$(CC) $(CFLAGS) $< $(CORE) -lm -o $@

clean:
	rm -f dmr_fec_test dmr_host_test dmr_host_test_voice dmr_host_test_dump dmr_synth ambe_dump.bin

dmr_host_test_voice: dmr_host_test.c $(CORE) $(MBE_SRC) $(wildcard $(DMR)/*.h)
	$(CC) -O2 -Wall -I$(DMR) -I$(MBELIB) -DDMR_VOICE_ENABLED=1 $< $(CORE) $(MBE_SRC) -lm -o $@
