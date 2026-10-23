#!/usr/bin/env python3
"""Actual C bass kernel: oracle, aliases, packet continuity, channel isolation.
Requires numpy/scipy. No hardware performance claims are made by this test.
"""
import ctypes as C
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from scipy import signal
ROOT = Path(__file__).resolve().parents[2]
DSP = ROOT/'firmware/DSPi'
sys.path.insert(0, str(ROOT/'scripts'))
from gen_rta_bass import design

WRAPPER = r'''
#include "rta_bass.c"
static RtaBassCoeffs coeff;
static RtaBassChannel state[9];
void reset(unsigned rate) { memset(state,0,sizeof(state)); rta_bass_configure(&coeff,rate,0); }
void feed(unsigned ch, const void *x, unsigned n) { rta_bass_push(&coeff,&state[ch],x,n); }
float power(unsigned ch,unsigned b) { return rta_bass_power(&coeff,&state[ch],b); }
unsigned state_size(void) { return sizeof(state[0]); }
'''

def native(x, flt):
    return np.asarray(x, dtype=np.float32) if flt else np.asarray(np.clip(x,-3.99,3.99)*2**29,dtype=np.int32)

def db(p):
    return 10*np.log10(np.maximum(p,1e-30))

with tempfile.TemporaryDirectory() as t:
    t=Path(t); (t/'test.c').write_text(WRAPPER)
    for flt in (0,1):
        libpath=t/f'bass{flt}.dylib'
        subprocess.run(['clang','-std=c11','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',
                        '-DRTA_HOST',f'-DRTA_SAMPLE_FLOAT={flt}','-I'+str(DSP),str(t/'test.c'),'-o',str(libpath)],check=True)
        lib=C.CDLL(str(libpath));lib.feed.argtypes=[C.c_uint,C.c_void_p,C.c_uint]
        lib.power.argtypes=[C.c_uint,C.c_uint];lib.power.restype=C.c_float
        def feed(x,ch=0):
            x=native(x,flt);lib.feed(ch,x.ctypes.data,len(x))
        worst=0.; quiet=0.; worst_alias=-300.; worst_octave=-300.; latency={}
        for rate in (44100,48000,96000):
            d,sos,bands=design(rate)
            n=np.arange(int(2*rate));time=n/rate
            for b,q in enumerate(bands):
                fc=1000*10**((b-20)/10)
                x=.5*np.sin(2*np.pi*fc*time+.37)
                lib.reset(rate);feed(x)
                # Independent float64 oracle, including quantisation, CIC
                # phase, IIR decimation, bandpass and envelope phase.
                shift=3*int(np.log2(d));scale=2**(27-shift)
                xn=native(x,flt)
                xq=np.trunc(xn.astype(float)*scale)/scale if flt else (xn>>(2+shift)).astype(float)/scale
                for _ in range(3): xq=signal.lfilter(np.ones(d)/d,[1],xq)
                low=signal.sosfilt(sos,xq[d-1::d])[7::8]
                sections,env,comp=q
                y=low
                for b0,a1,a2 in sections: y=signal.lfilter([b0,0,-b0],[1,a1,a2],y)
                expected=signal.lfilter([env],[1,env-1],2*y*y)[-1]*comp
                error=abs(db(lib.power(0,b))-db(expected));worst=max(worst,error)
                assert error<.25,(flt,rate,b,'oracle',error)
                assert abs(db(lib.power(0,b))+6.0206)<.65,(flt,rate,b,'centre')
                lib.reset(rate);feed(x*.002) # -60 dBFS amplitude
                err=abs(db(lib.power(0,b))+60);quiet=max(quiet,err)
                assert err<1.2,(flt,rate,b,'quiet',err)
                lib.reset(rate);feed(x*(10**(-70/20)/.5))
                assert abs(db(lib.power(0,b))+70)<1.5,(flt,rate,b,'-70 dBFS')
            # A tone must not light distant bands: one octave away >= 40 dB
            # down, two octaves >= 60 dB (a single biquad gave ~16 and ~26).
            for tb in (0,3,6,10,13):
                lib.reset(rate);feed(.5*np.sin(2*np.pi*1000*10**((tb-20)/10)*time))
                for b in range(14):
                    leak=db(lib.power(0,b))+6.0206
                    if abs(b-tb)==3: worst_octave=max(worst_octave,leak)
                    if abs(b-tb)>=6: assert leak< -60,(flt,rate,tb,b,leak)
                    elif abs(b-tb)>=3: assert leak< -40,(flt,rate,tb,b,leak)
            # Full-scale and +6 dBFS must not clip inside the decimator.
            for b in (0,3,13):
                fc=1000*10**((b-20)/10)
                for amplitude in (1.,2.,3.9):
                    lib.reset(rate);feed(amplitude*np.sin(2*np.pi*fc*time+.37))
                    assert abs(db(lib.power(0,b))-20*np.log10(amplitude))<.7, (flt,rate,b,amplitude)
            for b in (0,3,13):
                fc=1000*10**((b-20)/10)
                for freq in (rate/d/8-fc,rate/d-fc,rate/d+fc):
                    lib.reset(rate);feed(.5*np.sin(2*np.pi*freq*time))
                    rejection=db(lib.power(0,b))+6.0206;worst_alias=max(worst_alias,rejection)
                    assert rejection < -75,(flt,rate,b,freq,rejection)
            # Sustained full-scale DC must not appear as bass after settling.
            lib.reset(rate);feed(np.ones(3*rate))
            assert max(lib.power(0,b) for b in range(14)) < 1e-8
            # Arbitrary packet boundaries must yield exactly the same result.
            x=.3*np.sin(2*np.pi*19.952623*time)
            lib.reset(rate);feed(x);whole=[lib.power(0,b) for b in range(14)]
            lib.reset(rate)
            for start in range(0,len(x),137): feed(x[start:start+137])
            assert whole == [lib.power(0,b) for b in range(14)]
            assert all(lib.power(1,b)==0 for b in range(14))
            # Feeding other channels cannot alter a channel's measurement.
            for ch in range(1,9):feed(x*.5,ch)
            assert whole == [lib.power(0,b) for b in range(14)]
        for b in (0,3,13):
            rate=48000;fc=1000*10**((b-20)/10);lib.reset(rate)
            for start in range(0,rate,480):
                feed(.5*np.sin(2*np.pi*fc*np.arange(start,start+480)/rate))
                if lib.power(0,b) >= .25*.9:
                    latency[b]=(start+480)/rate;break
            assert b in latency and latency[b] < (.8 if b==0 else .5 if b==3 else .15), (flt,b,latency)
        print(f'PASS bass {"float" if flt else "Q27"}: {lib.state_size()} B/channel, '
              f'oracle {worst:.3f} dB, -60 dBFS error {quiet:.3f} dB, '
              f'alias rejection {-worst_alias:.1f} dB, one-octave rejection {-worst_octave:.1f} dB, '
              f'90% power response {latency}')
