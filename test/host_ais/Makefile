# Host tests for the AIS receiver (main/ais).
AIS = ../../main/ais
CFLAGS = -O2 -Wall -Wextra -Werror -I$(AIS)
all: ais_synth ais_host_test
ais_synth: ais_synth.c $(AIS)/ais_demod.c $(AIS)/ais_demod.h
	$(CC) $(CFLAGS) ais_synth.c $(AIS)/ais_demod.c -lm -o $@
ais_host_test: ais_host_test.c $(AIS)/ais_demod.c $(AIS)/ais_msg.c $(wildcard $(AIS)/*.h)
	$(CC) $(CFLAGS) ais_host_test.c $(AIS)/ais_demod.c $(AIS)/ais_msg.c -lm -o $@
clean:
	rm -f ais_synth ais_host_test
