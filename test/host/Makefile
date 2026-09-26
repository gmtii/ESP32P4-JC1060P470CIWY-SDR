# Host build of the ESP32-P4 FT8 pipeline for WAV regression tests.
FT8LIB ?= ../../components/ft8_lib
SRC = ft8_host_test.c \
      ../../main/ft8/ft8_ram.c ../../main/ft8/ft8_fft1024.c ../../main/ft8/ft8_decimator.c \
      ../../main/ft8/ft8_waterfall_adapter.c ../../main/ft8/ft8_decoder.c \
      $(FT8LIB)/ft8/constants.c $(FT8LIB)/ft8/crc.c $(FT8LIB)/ft8/decode.c \
      $(FT8LIB)/ft8/ldpc.c $(FT8LIB)/ft8/message.c $(FT8LIB)/ft8/text.c
CFLAGS = -O2 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -I../../main/ft8 -I$(FT8LIB)

all: ft8_host_test band_sync_sim

ft8_host_test: $(SRC) $(wildcard ../../main/ft8/*.h)
	$(CC) $(CFLAGS) $(SRC) -lm -o $@

clean:
	rm -f ft8_host_test band_sync_sim

SIM_SRC = band_sync_sim.c ../../main/ft8/ft8_band_sync.c $(filter-out ft8_host_test.c,$(SRC))
band_sync_sim: $(SIM_SRC) $(wildcard ../../main/ft8/*.h)
	$(CC) $(CFLAGS) $(SIM_SRC) -lm -o $@
