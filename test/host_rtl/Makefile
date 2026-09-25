# Host tests for the RTL front end (main/rtl_dsp.c NCO + rtl_source.c tuning logic)
all: nco_test
	./nco_test
	python3 tune_model.py

nco_test: nco_test.c ../../main/rtl_dsp.c ../../main/include/rtl_dsp.h
	$(CC) -O2 -Wall -I../../main/include nco_test.c ../../main/rtl_dsp.c -lm -o $@

clean:
	rm -f nco_test
