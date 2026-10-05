#include <psyz.h>
#include <libspu.h>
#include <psyz/log.h>
#include "../../decomp/src/libspu/libspu_private.h"

static SPU_RXX spu_RXX;
static unsigned _dma_dpcr;
union SpuUnion* _spu_RXX = (union SpuUnion*)&spu_RXX;
unsigned* dma_dpcr = &_dma_dpcr; // TODO this must be removed
volatile u16 _spu_RQ[10] = {0};

#include <math.h>

/* PSY-Q's note -> pitch: a note is (note number << 8 | fine), fine in 1/128
 * semitone; the sample's own note plays at pitch 0x1000. */
static unsigned short note_to_pitch(unsigned short sample_note, unsigned short note) {
    double semis = (double)((int)(note >> 8) - (int)(sample_note >> 8)) +
                   ((double)(note & 0xFF) - (double)(sample_note & 0xFF)) / 128.0;
    double pitch = 4096.0 * pow(2.0, semis / 12.0);
    if (pitch < 0) pitch = 0;
    if (pitch > 0x3FFF) pitch = 0x3FFF;
    return (unsigned short)(pitch + 0.5);
}

/* volume: the attribute is -0x4000..0x3FFF; the register holds it as 15-bit
 * two's complement with bit 15 clear (the sweep modes, bit 15 set, are not
 * emulated: a sweep is written as its direct value) */
static unsigned short vol_reg(short v) { return (unsigned short)(v & 0x7FFF); }

/* ADSR register encoding (psx-spx):
 *   adsr1: bit15 attack exp, 14-8 attack rate, 7-4 decay rate, 3-0 sustain level
 *   adsr2: bit15 sustain exp, bit14 sustain decrease, 12-6 sustain rate,
 *          bit5 release exp, 4-0 release rate */
void SpuSetVoiceAttr(SpuVoiceAttr* a) {
    u_long mask = a->mask;
    int v;
    for (v = 0; v < NUM_VOICES; v++) {
        volatile SPU_VOICE_REG* r;
        if (!(a->voice & (1ul << v))) continue;
        r = &_spu_RXX->rxx.voice[v];
        if (mask & SPU_VOICE_VOLL) r->volume.left = vol_reg(a->volume.left);
        if (mask & SPU_VOICE_VOLR) r->volume.right = vol_reg(a->volume.right);
        if (mask & SPU_VOICE_PITCH) {
            r->pitch = a->pitch & 0x3FFF;
        } else if (mask & SPU_VOICE_NOTE) {
            r->pitch = note_to_pitch(a->sample_note, a->note);
        } else if (mask & SPU_VOICE_SAMPLE_NOTE) {
            /* a new sample note alone: re-derive from the current note is not
             * possible, keep the pitch */
        }
        if (mask & SPU_VOICE_WDSA) r->addr = (u16)(a->addr >> 3);
        if (mask & SPU_VOICE_LSAX) r->loop_addr = (u16)(a->loop_addr >> 3);
        if (mask & SPU_VOICE_ADSR_ADSR1) {
            r->adsr[0] = a->adsr1;
        } else if (mask & (SPU_VOICE_ADSR_AMODE | SPU_VOICE_ADSR_AR | SPU_VOICE_ADSR_DR |
                           SPU_VOICE_ADSR_SL)) {
            u16 adsr1 = r->adsr[0];
            if (mask & SPU_VOICE_ADSR_AMODE) {
                adsr1 = (adsr1 & 0x7FFF) | ((a->a_mode == SPU_VOICE_EXPIncN) ? 0x8000 : 0);
            }
            if (mask & SPU_VOICE_ADSR_AR) adsr1 = (adsr1 & ~0x7F00) | ((a->ar & 0x7F) << 8);
            if (mask & SPU_VOICE_ADSR_DR) adsr1 = (adsr1 & ~0x00F0) | ((a->dr & 0x0F) << 4);
            if (mask & SPU_VOICE_ADSR_SL) adsr1 = (adsr1 & ~0x000F) | (a->sl & 0x0F);
            r->adsr[0] = adsr1;
        }
        if (mask & SPU_VOICE_ADSR_ADSR2) {
            r->adsr[1] = a->adsr2;
        } else if (mask & (SPU_VOICE_ADSR_SMODE | SPU_VOICE_ADSR_RMODE | SPU_VOICE_ADSR_SR |
                           SPU_VOICE_ADSR_RR)) {
            u16 adsr2 = r->adsr[1];
            if (mask & SPU_VOICE_ADSR_SMODE) {
                int exp = (a->s_mode == SPU_VOICE_EXPIncN || a->s_mode == SPU_VOICE_EXPDec);
                int dec = (a->s_mode == SPU_VOICE_LINEARDecN || a->s_mode == SPU_VOICE_LINEARDecR ||
                           a->s_mode == SPU_VOICE_EXPDec);
                adsr2 = (adsr2 & 0x3FFF) | (exp ? 0x8000 : 0) | (dec ? 0x4000 : 0);
            }
            if (mask & SPU_VOICE_ADSR_SR) adsr2 = (adsr2 & ~0x1FC0) | ((a->sr & 0x7F) << 6);
            if (mask & SPU_VOICE_ADSR_RMODE) {
                adsr2 = (adsr2 & ~0x0020) | ((a->r_mode == SPU_VOICE_EXPDec) ? 0x0020 : 0);
            }
            if (mask & SPU_VOICE_ADSR_RR) adsr2 = (adsr2 & ~0x001F) | (a->rr & 0x1F);
            r->adsr[1] = adsr2;
        }
    }
}

long SpuSetReverbModeParam(SpuReverbAttr* attr) {
    NOT_IMPLEMENTED;
    return 0;
}

/* The per-voice bit registers (noise, reverb, pitch LFO): reg_lo / reg_hi are
 * halfword indices into the register block (voices 0-15 / 16-23). */
u_long _SpuSetAnyVoice(long on_off, u_long voice_bit, int reg_lo, int reg_hi) {
    u_long cur = ((u_long)_spu_RXX->raw[reg_hi] << 16) | _spu_RXX->raw[reg_lo];
    voice_bit &= 0xFFFFFF;
    switch (on_off) {
    case SPU_ON:
        cur |= voice_bit;
        break;
    case SPU_OFF:
        cur &= ~voice_bit;
        break;
    case SPU_BIT:
        cur = voice_bit;
        break;
    default:
        return cur & 0xFFFFFF;
    }
    cur &= 0xFFFFFF;
    _spu_RXX->raw[reg_lo] = (u16)cur;
    _spu_RXX->raw[reg_hi] = (u16)(cur >> 16);
    return cur;
}

void SpuNGetVoiceAttr(int vNum, SpuVoiceAttr* a) {
    volatile SPU_VOICE_REG* r;
    if (vNum < 0 || vNum >= NUM_VOICES) return;
    r = &_spu_RXX->rxx.voice[vNum];
    a->voice = 1ul << vNum;
    a->volume.left = (short)((r->volume.left & 0x4000) ? (r->volume.left & 0x7FFF) - 0x8000
                                                       : (r->volume.left & 0x7FFF));
    a->volume.right = (short)((r->volume.right & 0x4000) ? (r->volume.right & 0x7FFF) - 0x8000
                                                         : (r->volume.right & 0x7FFF));
    a->volmode.left = a->volmode.right = SPU_VOICE_DIRECT;
    a->volumex.left = a->volumex.right = (short)r->volumex;
    a->envx = (short)r->volumex;
    a->pitch = r->pitch;
    a->addr = (u_long)r->addr << 3;
    a->loop_addr = (u_long)r->loop_addr << 3;
    a->adsr1 = r->adsr[0];
    a->adsr2 = r->adsr[1];
    a->a_mode = (r->adsr[0] & 0x8000) ? SPU_VOICE_EXPIncN : SPU_VOICE_LINEARIncN;
    a->ar = (r->adsr[0] >> 8) & 0x7F;
    a->dr = (r->adsr[0] >> 4) & 0x0F;
    a->sl = r->adsr[0] & 0x0F;
    a->s_mode = (r->adsr[1] & 0x8000) ? ((r->adsr[1] & 0x4000) ? SPU_VOICE_EXPDec : SPU_VOICE_EXPIncN)
                                      : ((r->adsr[1] & 0x4000) ? SPU_VOICE_LINEARDecN : SPU_VOICE_LINEARIncN);
    a->sr = (r->adsr[1] >> 6) & 0x7F;
    a->r_mode = (r->adsr[1] & 0x0020) ? SPU_VOICE_EXPDec : SPU_VOICE_LINEARDecN;
    a->rr = r->adsr[1] & 0x1F;
}
