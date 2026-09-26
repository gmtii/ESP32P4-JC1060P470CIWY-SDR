#!/usr/bin/env python3
# Keep in sync with rtl_source.c (rtl_source_set_freq, rtl_source_set_wide, ctl_task).
# Regression: the first NCO version looped forever on entering WFM (rest == 0 in
# wide mode went to the "retune again" branch) - scenario 2/3 below catch it.
# Model of rtl_source.c's tuning logic (set_freq / set_wide / ctl loop), used
# to check that no scenario causes endless physical retunes.
W=24000; RECENTRE_MS=1500
class S: pass
def reset():
    s=S(); s.want=100_000_000; s.hw=s.want; s.valid=True; s.off=0; s.pend=False; s.wide=False; s.t=0; s.last=0; s.retunes=0; return s
def set_freq(s,lo):
    s.want=lo; s.last=s.t
    if s.valid and not s.wide and not s.pend:
        d=lo-s.hw
        if -W<=d<=W: s.off=d; return
    s.pend=True
def set_wide(s,w):
    if s.wide==w: return
    s.wide=w
    if w and s.off!=0: s.pend=True
def ctl_iter(s):
    if s.off!=0 and (s.t-s.last)>RECENTRE_MS: s.pend=True
    if s.pend:
        s.pend=False; lo=s.want; s.retunes+=1; s.hw=lo
        rest=s.want-lo
        if rest==0: s.off=0
        elif not s.wide and -W<=rest<=W: s.off=rest
        else: s.off=0; s.pend=True
def run(s,ms):
    for _ in range(ms//250): s.t+=250; ctl_iter(s)
def scen(name,f):
    s=reset(); f(s); run(s,5000)
    print(f"{name:55s} physical retunes: {s.retunes:3d}  final hw={s.hw} want={s.want} off={s.off} pend={s.pend}")
    assert s.retunes<20 and not s.pend and s.hw+s.off==s.want, "FAIL"
def drag(s):
    for k in range(40): set_freq(s,s.want+1000); s.t+=20; ctl_iter(s) if k%12==0 else None
scen("drag 40 kHz in 1 kHz steps (then idle)", drag)
scen("enter WFM: set_freq then set_wide(true)", lambda s:(set_freq(s,s.want+12000), set_wide(s,True)))
scen("enter WFM: set_wide(true) then set_freq", lambda s:(set_wide(s,True), set_freq(s,s.want+12000)))
def wfm_tune(s):
    set_wide(s,True); set_freq(s,s.want+12000); run(s,1000)
    for k in range(10): set_freq(s,s.want+100000); s.t+=100; ctl_iter(s)
scen("in WFM: tune 1 MHz in 100 kHz steps", wfm_tune)
def leave(s):
    set_wide(s,True); set_freq(s,s.want); run(s,1000); set_wide(s,False); set_freq(s,s.want-12000)
scen("leave WFM back to NFM (-12 kHz LO offset)", leave)
scen("big jump 5 MHz (keypad)", lambda s:set_freq(s,s.want+5_000_000))
print("ALL SCENARIOS OK")
