#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rtl_dsp.h"
/* tone at f_true Hz above the hardware LO, dongle-style cu8 at 960 kSps */
static void mk(uint8_t *b, int n, double f, double *ph){ for(int k=0;k<n;k++){ b[2*k]=(uint8_t)lrint(127.5+80*cos(*ph)); b[2*k+1]=(uint8_t)lrint(127.5+80*sin(*ph)); *ph+=2*M_PI*f/960000.0; } }
static double tone_freq(const int16_t *o, int n){ /* mean phase increment -> frequency at 48 kSps */
  double acc=0; for(int k=1;k<n;k++){ double re=o[2*k]*(double)o[2*k-2]+o[2*k+1]*(double)o[2*k-1]; double im=o[2*k+1]*(double)o[2*k-2]-o[2*k]*(double)o[2*k-1]; acc+=atan2(im,re);} return acc/(n-1)*48000/(2*M_PI); }
int main(void){
  static rtl_dsp_t d, d0; static uint8_t in[2*96000]; static int16_t out[2*20000], out0[2*20000]; double ph=0;
  /* 1) offset 0 is bit-identical to the untouched chain */
  rtl_dsp_init(&d,false); rtl_dsp_init(&d0,false); ph=0; mk(in,96000,5000,&ph);
  size_t n=rtl_dsp_process(&d,in,2*96000,out,20000); size_t n0=rtl_dsp_process(&d0,in,2*96000,out0,20000);
  printf("offset 0: %zu frames, identical to plain chain: %s\n", n, (n==n0 && !memcmp(out,out0,n*4))?"yes":"NO");
  /* 2) tone at +17 kHz, digital offset +10 kHz -> must come out at +7 kHz */
  rtl_dsp_init(&d,false); rtl_dsp_set_offset(&d,10000); ph=0;
  for(int b=0;b<3;b++){ mk(in,96000,17000,&ph); n=rtl_dsp_process(&d,in,2*96000,out,20000);} 
  printf("tone +17000 Hz, offset +10000 Hz -> measured %.2f Hz (expect 7000)\n", tone_freq(out+400,n-200));
  rtl_dsp_set_offset(&d,-20000); mk(in,96000,17000,&ph); n=rtl_dsp_process(&d,in,2*96000,out,20000); mk(in,96000,17000,&ph); n=rtl_dsp_process(&d,in,2*96000,out,20000);
  printf("same tone, offset -20000 Hz -> measured %.2f Hz (expect 37000 -> outside +-24k, filtered)\n", tone_freq(out+400,n-200));
  /* 3) phase continuity across an offset change: tone exactly at the new offset => DC; look for a jump */
  rtl_dsp_init(&d,false); rtl_dsp_set_offset(&d,3000); ph=0; double maxjump=0, prevang=0; int first=1;
  for(int b=0;b<8;b++){ if(b==4) rtl_dsp_set_offset(&d,3500); mk(in,19200,3000,&ph); n=rtl_dsp_process(&d,in,2*19200,out,20000);
    for(size_t k=0;k<n;k++){ double a=atan2(out[2*k+1],out[2*k]); if(!first && b>0){ double dj=fabs(remainder(a-prevang,2*M_PI)); if(dj>maxjump) maxjump=dj;} prevang=a; first=0; } }
  printf("offset step 3000->3500 Hz on a 3 kHz tone: largest sample-to-sample phase step %.3f rad (a 500 Hz beat is %.3f rad/sample; a click would be ~pi)\n", maxjump, 2*M_PI*500/48000);
  { double ph2=0; rtl_dsp_t a; rtl_dsp_init(&a,false); rtl_dsp_set_offset(&a,10000); double e_in=0,e_out=0; size_t m=0;
    for(int b=0;b<3;b++){ mk(in,96000,17000,&ph2); m=rtl_dsp_process(&a,in,2*96000,out,20000);} for(size_t k=200;k<m;k++) e_in+=out[2*k]*(double)out[2*k]+out[2*k+1]*(double)out[2*k+1];
    rtl_dsp_init(&a,false); rtl_dsp_set_offset(&a,-20000); ph2=0; for(int b=0;b<3;b++){ mk(in,96000,17000,&ph2); m=rtl_dsp_process(&a,in,2*96000,out,20000);} for(size_t k=200;k<m;k++) e_out+=out[2*k]*(double)out[2*k]+out[2*k+1]*(double)out[2*k+1];
    printf("tone moved to 37 kHz (outside the +-24 kHz channel): level %.1f dB relative to in-band\n", 10*log10(e_out/e_in)); }
  return 0;
}

