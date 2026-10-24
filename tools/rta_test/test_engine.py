#!/usr/bin/env python3
"""Host smoke tests of the actual engine, with hardware/time stubs only."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DSP = ROOT / 'firmware/DSPi'
HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#define CONFIG_H
#define AUDIO_PIPELINE_H
#define USB_AUDIO_H
#define UPMIX_H
#define BG_JOB_H
#define DSP_TIME_CRITICAL
#define __not_in_flash_func(x) x
#if PICO_RP2350
#define NUM_INPUT_CHANNELS 8
#define NUM_OUTPUT_CHANNELS 9
#else
#define NUM_INPUT_CHANNELS 2
#define NUM_OUTPUT_CHANNELS 5
#endif
#define NUM_STEREO_INPUTS 2
static struct { uint32_t freq; } audio_state = {48000};
static struct { struct { bool enabled; } outputs[NUM_OUTPUT_CHANNELS]; } matrix_mixer;
typedef struct { uint8_t n_derived; } UpmixCoeffs;
static const UpmixCoeffs *current_upmix_coeffs;
static uint8_t active_input_channel_count(void) { return 2; }
static uint32_t now;
static uint32_t time_us_32(void) { return now; }
static void __dmb(void) {}
// Minimal bg_job.h: one job, and a deadline that never passes, so a step runs
// until the frame reaches its publish (chunking is covered by test_rta.py).
enum { BG_JOB_RTA_STEP };
#define BG_UNIT_UNMEASURED 0xFFFFu
typedef bool (*BgStepFn)(void *ctx, uint32_t deadline_us);
typedef struct { int id; BgStepFn step; void *ctx; volatile bool active; } BgJob;
static BgJob *bg_registered;
static void bg_job_register(BgJob *j) { bg_registered = j; }
static void bg_job_start(BgJob *j) { j->active = true; }
static void bg_job_cancel(BgJob *j) { j->active = false; }
static bool bg_out_of_time(uint32_t d, uint32_t u) { (void)d; (void)u; return false; }
static void bg_run_one(void) {
    BgJob *j = bg_registered;
    if (j && j->active && j->step(j->ctx, 0)) j->active = false;
}
#include "rta.c"
static int evt_count; static RtaConfig evt_cfg; static ParamSource evt_src;
void notify_push_rta_config(const void *c, ParamSource src) {
    evt_count++; memcpy(&evt_cfg,c,sizeof(evt_cfg)); evt_src=src;
}
static double tone_hz=100;
static void packet(bool tone) {
    rta_pipe_t buf[48];
    for (int i=0; i<48; ++i) {
        double x = tone ? .5*sin(6.283185307179586*tone_hz*((double)now/1000000+i/48000.)) : 0;
#if PICO_RP2350
        buf[i] = x;
#else
        buf[i] = x*536870912.;
#endif
    }
    rta_pipe_t original[48]; memcpy(original,buf,sizeof(buf));
    rta_packet_begin(48);
    for (int ch=0; ch<NUM_INPUT_CHANNELS; ++ch)
        rta_tap(RTA_TAP_INPUT, ch, buf, 48);
    for (int ch=0; ch<NUM_OUTPUT_CHANNELS; ++ch)
        if (matrix_mixer.outputs[ch].enabled) rta_tap(RTA_TAP_OUTPUT, ch, buf, 48);
    assert(memcmp(buf,original,sizeof(buf))==0);
    rta_packet_end(48);
    now += 1000;
    // The firmware main loop makes many passes per 1 ms packet; four is a
    // conservative model of that for the transform job.
    for (int pass=0; pass<4; ++pass) { rta_service(); bg_run_one(); }
}
int main(void) {
    (void)current_upmix_coeffs;
    for (int ch=0; ch<NUM_OUTPUT_CHANNELS; ++ch) matrix_mixer.outputs[ch].enabled=true;
    rta_init();
    RtaCaps caps; rta_get_caps(&caps);
    assert(caps.version==3 && caps.fft_order_max==10 && caps.max_bin_frame==16+512+1);
    RtaConfig cfg; rta_get_config(&cfg);
    cfg.version=1; assert(!rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET));
    cfg.version=2; assert(!rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET));
    cfg.version=3; cfg.fft_order=11; assert(!rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET));
    assert(evt_count==0);
    cfg.fft_order=caps.fft_order_default; cfg.avg_ms=0;
    assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET)); rta_service();
    // One RTA_CONFIG per applied SET, identical or not, carrying the applied
    // (clamped) bytes and the staging source; restarts and CONTROL never emit.
    assert(evt_count==1 && evt_src==PARAM_SRC_HOST_SET && memcmp(&evt_cfg,&rta.cfg,sizeof(cfg))==0);
    rta_service(); assert(evt_count==1);
    RtaConfig big=cfg; big.avg_ms=60000; big.peak_decay_db_s=200; big.reserved=7;
    assert(rta_set_config(&big,sizeof(big),PARAM_SRC_UART)); rta_service();
    assert(evt_count==2 && evt_src==PARAM_SRC_UART && evt_cfg.avg_ms==10000 &&
           evt_cfg.peak_decay_db_s==100 && evt_cfg.reserved==0);
    assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_I2C)); rta_service();
    assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET)); rta_service();
    assert(evt_count==4 && evt_src==PARAM_SRC_HOST_SET && evt_cfg.avg_ms==0);
    rta_restart(); rta_service(); assert(rta_control(RTA_CTL_RESET_AVG)); assert(evt_count==4);
    rta_note_read(); rta_service();
    for (int i=0;i<300;i++) packet(true);
    for (int ch=0;ch<NUM_OUTPUT_CHANNELS;ch++) {
        RtaBandFrame f; assert(rta_get_band_frame(ch,&f));
        assert(f.age_ms<=400 && f.avg[10]>220);
        // Every selected channel received the same continuous stream despite
        // different FFT capture/transform/publication turns.
        assert(memcmp(&rta.bass[ch],&rta.bass[0],sizeof(rta.bass[0]))==0);
    }
    uint16_t len; const uint8_t *bytes=rta_bin_frame(&len);
    const RtaBinFrameHeader *h=(const RtaBinFrameHeader *)bytes;
    assert(len==16+h->n_bins+1 && h->n_bins==(1u<<caps.fft_order_default)/2);
    assert(h->seq==bytes[len-1] && h->seq!=255);
    RtaStatus st; rta_get_status(&st); assert(st.state!=RTA_STATE_IDLE && st.first_band!=0xFF);
    // A channel that goes dead mid-fill must not stall the rotation.
    matrix_mixer.outputs[0].enabled=false;
    for (int i=0;i<300;i++) packet(false);
    matrix_mixer.outputs[0].enabled=true;
    for (int i=0;i<300;i++) packet(false);
    RtaBandFrame f; rta_get_band_frame(0,&f); assert(f.avg[10]==0 && f.age_ms<=400);
    // Reset and rate/source/preset restart use the existing common restart hook.
    rta_restart(); rta_service();
    for (int i=0;i<300;i++) packet(false);
    for (int ch=0;ch<NUM_OUTPUT_CHANNELS;ch++) {
        rta_get_band_frame(ch,&f); assert(f.avg[10]==0 && f.age_ms<=400);
    }
    cfg.fft_order=10; assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET)); rta_service();
    for (int i=0;i<600;i++) packet(true);
    rta_get_band_frame(0,&f); assert(f.n_bands==34 && f.avg[10]>220);
    bytes=rta_bin_frame(&len); assert(len==16+512+1);
    assert(caps.bass_bands==14 && caps.max_bands==37 && sizeof(f)==82);
    // Lowest band works for all channels with the existing rotating readout.
    tone_hz=10; rta_restart();
    for (int i=0;i<1000;i++) packet(true);
    for (int ch=0;ch<NUM_OUTPUT_CHANNELS;ch++) {
        rta_get_band_frame(ch,&f); assert(f.avg[0]>=229 && f.avg[0]<=233);
    }
    // Changing averaging does not reset the stream; RESET_AVG does.
    RtaBassChannel saved=rta.bass[0]; cfg.avg_ms=1000;
    assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET));rta_service();
    assert(memcmp(&saved,&rta.bass[0],sizeof(saved))==0);
    assert(rta_control(RTA_CTL_RESET_AVG));
    rta_get_band_frame(0,&f);assert(f.age_ms==0xFFFF && f.avg[0]==0);
    assert(rta_bass_power(&rta.bass_coeffs,&rta.bass[0],0)==0);
    // Tap/mask change clears old output history and only feeds selected inputs.
    cfg.tap=RTA_TAP_INPUT;cfg.channel_mask=2;cfg.avg_ms=0;
    assert(rta_set_config(&cfg,sizeof(cfg),PARAM_SRC_HOST_SET));rta_service();
    int before=evt_count;
    for (int i=0;i<1000;i++) packet(true);
    rta_get_band_frame(1,&f);assert(f.avg[0]>=229 && f.avg[0]<=233);
    assert(rta_bass_power(&rta.bass_coeffs,&rta.bass[0],0)==0);
    // STOP freezes the last readings even while audio packets continue.
    assert(rta_control(RTA_CTL_STOP));saved=rta.bass[1];packet(true);
    assert(memcmp(&saved,&rta.bass[1],sizeof(saved))==0);
    rta_get_band_frame(1,&f);assert(f.avg[0]>=229 && f.avg[0]<=233);
    assert(rta_control(RTA_CTL_START));
    // Rate changes must discard both the filter tails and a partial FFT.
    audio_state.freq=96000;rta_service();
    assert(rta_bass_power(&rta.bass_coeffs,&rta.bass[1],0)==0);
    assert(rta.bass_coeffs.decimation==16 && rta.wr==0);
    rta_get_band_frame(1,&f);assert(f.age_ms==0xFFFF && f.n_bands==37);
    assert(rta_control(RTA_CTL_STOP));saved=rta.bass[1];packet(true);
    assert(memcmp(&saved,&rta.bass[1],sizeof(saved))==0);
    rta_note_read();assert(rta.state!=RTA_STATE_IDLE);
    now+=6000000; rta_service(); rta_get_status(&st); assert(st.state==RTA_STATE_IDLE);
    rta_note_read(); rta_service(); assert(evt_count==before);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as temp:
    tmp = Path(temp)
    for name in ('pico/time.h', 'hardware/sync.h'):
        p = tmp / name; p.parent.mkdir(exist_ok=True); p.write_text('')
    source = tmp / 'engine.c'; source.write_text(HARNESS)
    for flt in (0, 1):
        exe = tmp / ('engine%d' % flt)
        subprocess.run(['clang','-O2','-Wall','-Wextra','-Werror',
                        '-fsanitize=undefined','-fno-sanitize-recover=all',
                        '-DRTA_HOST','-DPICO_RP2350=%d'%flt,'-DRTA_SAMPLE_FLOAT=%d'%flt,
                        '-I'+str(tmp),'-I'+str(DSP),str(source),
                        str(DSP/'rta_fft.c'),str(DSP/'rta_bass.c'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print('PASS engine %s: protocol, all-channel cadence, bins, liveness, restart, off, timeout' % ('float' if flt else 'Q28'))
