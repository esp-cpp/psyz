#include <psyz.h>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define PSYZ_ESP_XRAM EXT_RAM_BSS_ATTR
#else
#define PSYZ_ESP_XRAM
#endif
#include <psyz/log.h>
#include <assert.h>
#include <string.h>
#include "../../decomp/src/libspu/libspu_private.h"
#include "spu_gauss.h"

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(SPU_RXX) == 0x200, "SPU_RXX must be 0x200 bytes");
_Static_assert(
    sizeof(union SpuUnion) == sizeof(SPU_RXX), "SpuUnion must alias SPU_RXX");
#endif

// One ADPCM block decodes to 28 samples
#define ADPCM_BLOCK_BYTES 16
#define ADPCM_BLOCK_SAMPLES 28

static inline short clamp16(int v) {
    if (v < -32768)
        return -32768;
    if (v > 32767)
        return 32767;
    return v;
}

static inline short clamp15(int v) {
    if (v < -16384)
        return -16384;
    if (v > 16383)
        return 16383;
    return v;
}

static inline int sign4(int n) { return (n & 0x8) ? (n - 0x10) : n; }

// Decode one 16-byte SPU ADPCM block into 28 short samples. PS1 SPU ADPCM:
//   block[0]    = shift_filter: low nibble = shift (12-shift_in, or 9 if >12)
//                               high nibble = filter index (clamped 0..4)
//   block[1]    = flags: bit0 = loop-end, bit1 = repeat, bit2 = loop-start
//   block[2..15]= 14 bytes of 4-bit nibbles (low nibble first)
static void spu_adpcm_decode_block(
    const unsigned char block[ADPCM_BLOCK_BYTES], short* hist1, short* hist2,
    short out[ADPCM_BLOCK_SAMPLES], u8* flags_out) {
    static const int pos[5] = {0, 60, 115, 98, 122};
    static const int neg[5] = {0, 0, -52, -55, -60};

    int shift_in = block[0] & 0x0F;
    int shift = (shift_in > 12) ? 9 : (12 - shift_in);
    int filter = (block[0] >> 4) & 0x07;
    if (filter > 4)
        filter = 4;
    int f0 = pos[filter];
    int f1 = neg[filter];
    if (flags_out)
        *flags_out = block[1];

    short prev = *hist1;
    short prev2 = *hist2;
    for (int i = 0; i < 14; i++) {
        unsigned short byte = block[2 + i];
        for (int n = 0; n < 2; n++) {
            int t = sign4((byte >> (n * 4)) & 0x0F);
            int s = (t << shift) + ((prev * f0) >> 6) + ((prev2 * f1) >> 6);
            short final = clamp16(s);
            out[i * 2 + n] = final;
            prev2 = prev;
            prev = final;
        }
    }
    *hist1 = prev;
    *hist2 = prev2;
}

#define N_CHANNELS 2                      // stereo interleaved
#define CD_RING_FRAMES 4096               // must be power of 2
#define CD_RING_MASK (CD_RING_FRAMES - 1) // ring wrapper
#define CD_RING_LOW_WATER 1024            // refill when short of N frames
#define ENV_KEYON_DELAY_TICKS 6 // latency to simulate PS1 SPU latching a key-on

typedef enum {
    ADSR_ATTACK = 0,
    ADSR_DECAY,
    ADSR_SUSTAIN,
    ADSR_RELEASE,
    ADSR_OFF,
} AdsrState;

typedef struct {
    unsigned cur_addr;    // current 16-byte block address in SPU RAM
    unsigned repeat_addr; // from voice loop_addr reg, or block flag bit 2
    short hist1, hist2;
    short samples[ADPCM_BLOCK_SAMPLES];
    u8 block_flags; // flags byte of the most-recently-decoded block
    u8 sample_idx;  // next sample to consume from samples[] (0..28)
    unsigned spos;  // 16.16 fixed-point counter (1.0 = 0x10000)
    unsigned sinc;  // pitch_reg << 4 is the per-output-tick increment
    short gwin[4];  // GAUSS interpolation window
    u8 gpos;        // index to next decoded sample in gwin
    u8 active;
    u8 needs_decode;
    int env_vol;          // mirrors SPU_VOICE_REG::volumex
    unsigned env_counter; // rate divider, increases by 1 for all voice duration
    AdsrState env_state;  // voice phase, works like a state machine
    u8 key_off;           // set by key-off; drives the release phase
    u8 delay_ticks;       // key-on -> envelope-start pipeline latency
    u16 adsr_lo;          // mirrors SPU_VOICE_REG::adsr[0]
    u16 adsr_hi;          // mirrors SPU_VOICE_REG::adsr[1]
} VoiceState;

// Full SPU state
typedef struct {
    u8 ram[PSYZ_SPU_RAM_SIZE];

    // Transfer address (byte address into SPU RAM)
    unsigned transfer_addr;

    // Capture buffer position
    u16 capture_pos;

    // bit v: voice v is active (spu_tick visits only these)
    unsigned active_mask;

    VoiceState voice[PSYZ_SPU_NUM_VOICES];

    // CD audio ring buffer, refilled by Psyz_CdPullSamples
    short cd_ring[CD_RING_FRAMES * N_CHANNELS];
    unsigned cd_ring_read;
    unsigned cd_ring_count; // number of valid frames in ring

    u8 initialized;
} SpuState;
#ifdef ESP_PLATFORM
/* in the cart's arena (port/esp32_arena.c), bound at launch */
static SpuState* spu_p;
void Psyz_SpuBind(void* p) { spu_p = (SpuState*)p; }
size_t Psyz_SpuStateSize(void) { return sizeof(SpuState); }
#else
static SpuState spu_storage;
static SpuState* spu_p = &spu_storage;
#endif
#define spu (*spu_p)

u8* Psyz_SpuGetRam(void) { return spu.ram; }

void Psyz_SpuInit(void) {
    if (spu.initialized)
        return;
    memset(&spu, 0, sizeof(spu));
    spu.initialized = 1;
    INFOF("SPU emulation initialized");
}

static void spu_reset(void) {
    _spu_RXX->rxx.spustat = 0;
    for (int v = 0; v < PSYZ_SPU_NUM_VOICES; v++) {
        _spu_RXX->rxx.voice[v].volumex = 0;
    }
    memset(&spu, 0, sizeof(spu));
}

static void spu_reset_hot(void) {
    /* everything but SPU RAM: the state's fields after it */
    _spu_RXX->rxx.spustat = 0;
    for (int v = 0; v < PSYZ_SPU_NUM_VOICES; v++) {
        _spu_RXX->rxx.voice[v].volumex = 0;
    }
    memset((u8*)&spu + PSYZ_SPU_RAM_SIZE, 0, sizeof(spu) - PSYZ_SPU_RAM_SIZE);
}

void Psyz_SpuReset(int hot) {
    if (hot) {
        spu_reset_hot();
    } else {
        spu_reset();
    }
    spu.initialized = 1;
}

void Psyz_SpuSetTransferAddr(unsigned int addr) {
    spu.transfer_addr = addr & (PSYZ_SPU_RAM_SIZE - 1);
}

unsigned int Psyz_SpuGetTransferAddr(void) { return spu.transfer_addr; }

void Psyz_SpuFifoWrite(unsigned short word) {
#ifdef PLATFORM_LE
    *(unsigned short*)&spu.ram[spu.transfer_addr] = word;
#else
    spu.ram[spu.transfer_addr] = (unsigned char)(word & 0xFF);
    spu.ram[spu.transfer_addr + 1] = (unsigned char)((word >> 8) & 0xFF);
#endif
    spu.transfer_addr = (spu.transfer_addr + 2) & (PSYZ_SPU_RAM_SIZE - 1);
}

void Psyz_SpuFifoWriteBulk(const unsigned char* src, unsigned int size) {
    Psyz_SpuMemWrite(spu.transfer_addr, src, size);
    spu.transfer_addr = (spu.transfer_addr + size) & (PSYZ_SPU_RAM_SIZE - 1);
}

void Psyz_SpuMemRead(unsigned int offset, void* dst, unsigned int size) {
    unsigned int start = offset & (PSYZ_SPU_RAM_SIZE - 1);
    unsigned int head = PSYZ_SPU_RAM_SIZE - start;
    if (size <= head) {
        memcpy(dst, &spu.ram[start], size);
    } else {
        memcpy(dst, &spu.ram[start], head);
        memcpy((unsigned char*)dst + head, &spu.ram[0], size - head);
    }
}

void Psyz_SpuMemWrite(unsigned int offset, const void* src, unsigned int size) {
    unsigned int start = offset & (PSYZ_SPU_RAM_SIZE - 1);
    unsigned int head = PSYZ_SPU_RAM_SIZE - start;
    if (size <= head) {
        memcpy(&spu.ram[start], src, size);
    } else {
        memcpy(&spu.ram[start], src, head);
        memcpy(&spu.ram[0], (const unsigned char*)src + head, size - head);
    }
}

static void spu_key_on_voice(int v) {
    volatile SPU_RXX* rxx = &_spu_RXX->rxx;
    VoiceState* vs = &spu.voice[v];
    vs->cur_addr =
        ((unsigned)rxx->voice[v].addr << 3) & (PSYZ_SPU_RAM_SIZE - 1);
    vs->repeat_addr =
        ((unsigned)rxx->voice[v].loop_addr << 3) & (PSYZ_SPU_RAM_SIZE - 1);
    vs->hist1 = vs->hist2 = 0;
    vs->sample_idx = ADPCM_BLOCK_SAMPLES; // force decode on first consume
    vs->block_flags = 0;
    vs->needs_decode = 1;
    // sinc = pitch << 4; pitch=0x1000 -> sinc=0x10000 (1.0 in 16.16)
    unsigned pitch = rxx->voice[v].pitch;
    vs->sinc = (pitch & 0x3FFF) << 4;
    if (vs->sinc == 0)
        vs->sinc = 1;

    // Start spos at 1.0 so the first voice_step consumes exactly one sample.
    // The gauss window stays mostly zero-padded for several ticks, producing
    // the silent warmup-then-ringing pattern observed in PS1 captures
    vs->spos = 0x10000;

    // We do not reset vs->gpos at key-on for accuracy. On real PS1 hardware,
    // a voice on will carry the residual state from prior voice activity
    vs->active = 1;
    spu.active_mask |= 1u << v;

    // reset the envelope to a fresh attack from zero
    vs->adsr_lo = rxx->voice[v].adsr[0];
    vs->adsr_hi = rxx->voice[v].adsr[1];
    vs->env_vol = 0;
    vs->env_counter = 0;
    vs->env_state = ADSR_ATTACK;
    vs->key_off = 0;
    vs->delay_ticks = ENV_KEYON_DELAY_TICKS;
    rxx->voice[v].volumex = 0;
}

extern volatile int psyz_pending_spu_irq;
extern volatile unsigned psyz_spu_irq_raised, psyz_spu_irq_caught_up;
void Psyz_SpuWrite(unsigned int reg_offset, unsigned short value) {
    if (reg_offset >= sizeof(SPU_RXX) || (reg_offset & 1)) {
        WARNF("Psyz_SpuWrite: bad offset 0x%X", reg_offset);
        return;
    }
    if (reg_offset == offsetof(SPU_RXX, spustat)) { // read-only
        return;
    }
    _spu_RXX->raw[reg_offset >> 1] = value;
    // MSVC's C-mode offsetof is not an integer constant expression, so it
    // cannot appear in a case label. Use an if-chain instead.
    if (reg_offset == offsetof(SPU_RXX, spucnt)) {
        if (!(value & SPU_CTRL_MASK_IRQ9_ENABLE)) { // acknowledge
            _spu_RXX->rxx.spustat &= ~(1u << 6);
        }
    } else if (reg_offset == offsetof(SPU_RXX, irq_addr)) {
        // Re-arming behind a voice that has already passed the new address.
        //
        // Samples are produced in chunks and the interrupt is noticed and
        // handled some milliseconds after the crossing. A driver that uses
        // a looping voice as a clock (MGS: a blank sample, the IRQ address
        // toggled between two points 256 bytes apart, ~98 Hz) re-arms to a
        // point the voice may by then have passed; the hardware would wait a
        // whole lap and the clock loses a beat -- heard as streamed speech
        // repeating a block. Raise it now instead: the beat arrives late
        // rather than never, and the stream refill clocked by the count
        // keeps pace with the samples.
        if (_spu_RXX->rxx.spucnt & SPU_CTRL_MASK_IRQ9_ENABLE) {
            for (int v = 0; v < PSYZ_SPU_NUM_VOICES; v++) {
                VoiceState* vs = &spu.voice[v];
                if (vs->active) {
                    unsigned d = ((vs->cur_addr >> 3) - (unsigned)value) & 0xFFFFu;
                    if (d > 0 && d < 32) { // passed it within the last 256 bytes
                        _spu_RXX->rxx.spustat |= 1u << 6;
                        __atomic_fetch_add(&psyz_pending_spu_irq, 1, __ATOMIC_RELAXED);
                        psyz_spu_irq_raised++;
                        psyz_spu_irq_caught_up++;
                        break;
                    }
                }
            }
        }
    } else if (reg_offset == offsetof(SPU_RXX, key_on[0])) { // voices 0..15
        for (int v = 0; v < 16; v++) {
            if (value & (1u << v))
                spu_key_on_voice(v);
        }
    } else if (reg_offset == offsetof(SPU_RXX, key_on[1])) { // voices 16..23
        for (int v = 0; v < 8; v++) {
            if (value & (1u << v))
                spu_key_on_voice(16 + v);
        }
    } else if (reg_offset == offsetof(SPU_RXX, key_off[0])) { // voices 0..15
        for (int v = 0; v < 16; v++) {
            if (value & (1u << v))
                spu.voice[v].key_off = 1;
        }
    } else if (reg_offset == offsetof(SPU_RXX, key_off[1])) { // voices 16..23
        for (int v = 0; v < 8; v++) {
            if (value & (1u << v))
                spu.voice[16 + v].key_off = 1;
        }
    } else if (reg_offset == offsetof(SPU_RXX, trans_addr)) {
        Psyz_SpuSetTransferAddr((unsigned)value << 3);
    } else if (reg_offset == offsetof(SPU_RXX, trans_fifo)) {
        Psyz_SpuFifoWrite(value);
    }
}

unsigned short Psyz_SpuRead(unsigned int reg_offset) {
    if (reg_offset >= sizeof(SPU_RXX) || (reg_offset & 1)) {
        WARNF("Psyz_SpuRead: bad offset 0x%X", reg_offset);
        return 0;
    }
    // offsetof is not an ICE under MSVC C mode, so it cannot be a case label.
    if (reg_offset == offsetof(SPU_RXX, trans_addr)) {
        return (unsigned short)((spu.transfer_addr >> 3) & 0xFFFF);
    }
    if (reg_offset == offsetof(SPU_RXX, spustat)) {
        // lower 6 bits mirror SPUCNT's low bits
        // bit 11, capture-buffer half-pointer, flipped by spu_tick.
        return (_spu_RXX->rxx.spucnt & 0x3F) |
               (_spu_RXX->rxx.spustat & (1u << 11));
    }
    return _spu_RXX->raw[reg_offset >> 1];
}

static void write_capture(unsigned int idx, short val) {
    unsigned int addr = (idx * 0x400) | spu.capture_pos;
    if (addr < PSYZ_SPU_RAM_SIZE) {
        *(short*)&spu.ram[addr] = val;
    }
}

// The SPU IRQ (SPUCNT bit 6, IRQ address register): raised when a voice
// fetches the block at the IRQ address, acknowledged by clearing the enable
// bit. Games time their sound drivers with it (MGS loops a silent sample at
// the address and runs its driver tick off the interrupt). The callback is
// NOT run here: on the ESP32 the samples are pulled on another core than the
// game, so the interrupts are only counted and the platform delivers them
// from the game's side (see psyz_pending_spu_irq).
volatile int psyz_pending_spu_irq;
// statistics for the platform's audio diagnostics
volatile unsigned psyz_spu_irq_raised, psyz_spu_voices_active, psyz_spu_irq_caught_up;

static inline void spu_irq_check(unsigned addr) {
    SPU_RXX* rxx = (SPU_RXX*)&_spu_RXX->rxx;
    if ((rxx->spucnt & SPU_CTRL_MASK_IRQ9_ENABLE) &&
        (addr >> 3) == rxx->irq_addr) {
        // one interrupt per pass over the address: a counter, since the
        // platform delivers them later and must not lose any
        rxx->spustat |= 1u << 6;
        __atomic_fetch_add(&psyz_pending_spu_irq, 1, __ATOMIC_RELAXED);
        psyz_spu_irq_raised++;
    }
}

// the voice's next decoded sample (0 when it stops: then returns 0)
static inline __attribute__((always_inline)) int voice_fetch(VoiceState* vs, short* out) {
    if (vs->sample_idx >= ADPCM_BLOCK_SAMPLES) {
        // end of block, and handle loop/end flags
        if (!vs->needs_decode && (vs->block_flags & 0x01)) {
            if (!(vs->block_flags & 0x02)) {
                vs->active = 0;
                spu.active_mask &= ~(1u << (vs - spu.voice));
                *out = 0;
                return 0;
            }
            vs->cur_addr = vs->repeat_addr;
        }
        vs->needs_decode = 1;
        vs->sample_idx = 0;
    }
    if (vs->needs_decode) {
        unsigned char block[ADPCM_BLOCK_BYTES];
        const unsigned char* src = &spu.ram[vs->cur_addr];
        spu_irq_check(vs->cur_addr);
        if (vs->cur_addr > PSYZ_SPU_RAM_SIZE - ADPCM_BLOCK_BYTES) {
            Psyz_SpuMemRead(vs->cur_addr, block, ADPCM_BLOCK_BYTES);
            src = block;
        }
        spu_adpcm_decode_block(
            src, &vs->hist1, &vs->hist2, vs->samples, &vs->block_flags);
        if (vs->block_flags & 0x04) {
            vs->repeat_addr = vs->cur_addr;
        }
        vs->cur_addr =
            (vs->cur_addr + ADPCM_BLOCK_BYTES) & (PSYZ_SPU_RAM_SIZE - 1);
        vs->needs_decode = 0;
    }
    *out = vs->samples[vs->sample_idx++];
    return 1;
}

static inline __attribute__((always_inline)) int voice_decode_one_sample(VoiceState* vs) {
    short s;
    const int ok = voice_fetch(vs, &s);
    vs->gwin[vs->gpos] = s;
    vs->gpos = (vs->gpos + 1) & 3;
    return ok;
}

static unsigned adsr_denominator(int rate) {
    return rate < 48 ? 1u : (1u << ((rate >> 2) - 11));
}

static int adsr_num_increase(int rate) {
    return rate < 48 ? (7 - (rate & 3)) << (11 - (rate >> 2))
                     : (7 - (rate & 3));
}

static int adsr_num_decrease(int rate) {
    return rate < 48 ? (-8 + (rate & 3)) << (11 - (rate >> 2))
                     : (-8 + (rate & 3));
}

// the envelope's state, for a voice's run through a block in locals
typedef struct {
    int vol;
    unsigned counter;
    AdsrState state;
    u8 delay;
} EnvState;

// the envelope's counter mask in its current phase: it changes when the
// counter & mask is 0. It depends only on the phase and (attack, sustain
// increase) on the volume crossing 0x6000, so it is recomputed only when one
// of those changes.
static inline __attribute__((always_inline)) unsigned env_mask(
    const EnvState* e, u16 adsr_lo, u16 adsr_hi) {
    int rate;
    switch (e->state) {
    case ADSR_ATTACK:
        rate = (adsr_lo >> 8) & 0x7F;
        if (((adsr_lo >> 15) & 1) && e->vol >= 0x6000) {
            rate += 8;
        }
        break;
    case ADSR_DECAY:
        rate = ((adsr_lo >> 4) & 0x0F) * 4; // decay is always exponential decrease
        break;
    case ADSR_SUSTAIN:
        // held at the top (increase) or at 0 (decrease) firing changes
        // nothing: the commonest case, often at a rate firing every sample
        if (!((adsr_hi >> 14) & 1) ? e->vol == 0x7FFF : e->vol == 0) {
            return ~0u;
        }
        rate = (adsr_hi >> 6) & 0x7F;
        if (!((adsr_hi >> 14) & 1) && ((adsr_hi >> 15) & 1) && e->vol >= 0x6000) {
            rate += 8;
        }
        break;
    case ADSR_RELEASE:
        rate = (adsr_hi & 0x1F) * 4;
        break;
    default:
        return ~0u; // off: nothing changes (the counter wrapping fires nothing)
    }
    return adsr_denominator(rate) - 1;
}

// the envelope's change when its counter fires; 1 when the release ends
static inline __attribute__((always_inline)) int env_fire(
    EnvState* e, u16 adsr_lo, u16 adsr_hi) {
    switch (e->state) {
    case ADSR_ATTACK: {
        int rate = (adsr_lo >> 8) & 0x7F;
        if (((adsr_lo >> 15) & 1) && e->vol >= 0x6000) {
            rate += 8;
        }
        e->vol += adsr_num_increase(rate);
        if (e->vol >= 0x7FFF) {
            e->vol = 0x7FFF;
            e->state = ADSR_DECAY;
        }
        break;
    }
    case ADSR_DECAY: {
        int rate = ((adsr_lo >> 4) & 0x0F) * 4;
        e->vol += (adsr_num_decrease(rate) * e->vol) >> 15;
        if (e->vol < 0) {
            e->vol = 0;
        }
        int sustain_level = adsr_lo & 0x0F;
        if (((e->vol >> 11) & 0xF) <= sustain_level) {
            e->state = ADSR_SUSTAIN;
        }
        break;
    }
    case ADSR_SUSTAIN: {
        int rate = (adsr_hi >> 6) & 0x7F;
        int sustain_dec = (adsr_hi >> 14) & 1;
        int sustain_exp = (adsr_hi >> 15) & 1;
        if (!sustain_dec) {
            if (sustain_exp && e->vol >= 0x6000) {
                rate += 8;
            }
            e->vol += adsr_num_increase(rate);
            if (e->vol > 0x7FFF) {
                e->vol = 0x7FFF;
            }
        } else {
            if (sustain_exp) {
                e->vol += (adsr_num_decrease(rate) * e->vol) >> 15;
            } else {
                e->vol += adsr_num_decrease(rate);
            }
            if (e->vol < 0) {
                e->vol = 0;
            }
        }
        break;
    }
    case ADSR_RELEASE: {
        int rate = (adsr_hi & 0x1F) * 4;
        if ((adsr_hi >> 5) & 1) {
            e->vol += (adsr_num_decrease(rate) * e->vol) >> 15;
        } else {
            e->vol += adsr_num_decrease(rate);
        }
        if (e->vol <= 0) {
            e->vol = 0;
            e->state = ADSR_OFF;
            return 1;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

// process voice ADSR envelope by one sample, calculate ADSR envelope volume;
// 1 when the release ends (the voice stops)
static inline __attribute__((always_inline)) int env_step(
    EnvState* e, u16 adsr_lo, u16 adsr_hi, u8 key_off) {
    if (e->delay) { // simulate PS1 SPU key on latency
        e->delay--;
        return 0;
    }
    if (key_off && e->state != ADSR_OFF) {
        e->state = ADSR_RELEASE;
    }

    // the PS1 SPU has an internal counter, shared across all states, that
    // allows to trigger a change based on the selected ADSR rate
    // (the denominator is a power of two: a mask, not a divide)
    if ((++e->counter & env_mask(e, adsr_lo, adsr_hi)) == 0) {
        return env_fire(e, adsr_lo, adsr_hi);
    }
    return 0;
}

static inline __attribute__((always_inline)) void voice_envelope_step(VoiceState* vs) {
    EnvState e = {vs->env_vol, vs->env_counter, vs->env_state, vs->delay_ticks};
    const int off = env_step(&e, vs->adsr_lo, vs->adsr_hi, vs->key_off);
    vs->env_vol = e.vol;
    vs->env_counter = e.counter;
    vs->env_state = e.state;
    vs->delay_ticks = e.delay;
    if (off) {
        vs->active = 0;
        spu.active_mask &= ~(1u << (vs - spu.voice));
    }
}


// Advance voice `v` by one output-rate tick (44.1 kHz) and return its
// pitch-resampled, gauss-interpolated short sample.
static inline __attribute__((always_inline)) short voice_step(int v) {
    VoiceState* vs = &spu.voice[v];

    // for pitch changes during voice on, enable vibrato or bends
    unsigned pitch = _spu_RXX->rxx.voice[v].pitch & 0x3FFF;
    vs->sinc = pitch ? pitch << 4 : 1;

    // consume decoded samples until the pitch counter is below 1.0
    while (vs->spos >= 0x10000) {
        if (!voice_decode_one_sample(vs)) {
            // Voice stopped mid-decode; flush remaining ticks as zero.
            vs->spos = 0;
            break;
        }
        vs->spos -= 0x10000;
    }

    // GAUSS interpolation
    int vl = (vs->spos >> 6) & ~3;
    int g0 = vs->gwin[vs->gpos & 3];
    int g1 = vs->gwin[(vs->gpos + 1) & 3];
    int g2 = vs->gwin[(vs->gpos + 2) & 3];
    int g3 = vs->gwin[(vs->gpos + 3) & 3];
    int acc = (spu_gauss_tbl[vl + 0] * g0) & ~2047;
    acc += (spu_gauss_tbl[vl + 1] * g1) & ~2047;
    acc += (spu_gauss_tbl[vl + 2] * g2) & ~2047;
    acc += (spu_gauss_tbl[vl + 3] * g3) & ~2047;
    vs->spos += vs->sinc;
    return clamp16(acc >> 12);
}

static inline int voice_vol(unsigned short reg) {
    if (reg & 0x8000) {
        // https://problemkaputt.de/psxspx-spu-volume-and-adsr-generator.htm
        LOG_ONCE("voice volume bit15 not implemented");
        return 0;
    }
    int v = reg & 0x7FFF;
    if (v & 0x4000) { // bit14 is sign for values -0x4000 to 0x3FFF
        v -= 0x8000;
    }
    return v;
}

// generate one frame at 44100hz with voices mix, cd playback and volume control
static void spu_tick(short* out) {
    SPU_RXX* rxx = (SPU_RXX*)&_spu_RXX->rxx;
    unsigned short spucnt = rxx->spucnt;

    // accumulate frame to each separate channels
    int left_sum = 0, right_sum = 0;

    // Pre-fill CD ring buffer if running low
    if (spu.cd_ring_count < CD_RING_LOW_WATER) {
        size_t space = CD_RING_FRAMES - spu.cd_ring_count;
        size_t write_pos =
            (spu.cd_ring_read + spu.cd_ring_count) & CD_RING_MASK;
        // Fill into contiguous chunk up to end of array
        size_t chunk = CD_RING_FRAMES - write_pos;
        if (chunk > space)
            chunk = space;
        size_t got = Psyz_CdPullSamples(&spu.cd_ring[write_pos * 2], chunk);
        spu.cd_ring_count += got;
    }

    // Read one stereo frame from CD ring buffer
    int cd_left = 0, cd_right = 0;
    if (spu.cd_ring_count > 0) {
        cd_left = spu.cd_ring[spu.cd_ring_read * 2];
        cd_right = spu.cd_ring[spu.cd_ring_read * 2 + 1];
        spu.cd_ring_read = (spu.cd_ring_read + 1) & CD_RING_MASK;
        spu.cd_ring_count--;
    }

    // decode+resample, scale volume by ADSR envelope, then mix voices
    short v1_sample = 0, v3_sample = 0;
    unsigned active = 0;
    for (unsigned m = spu.active_mask; m; m &= m - 1) {
        const int v = __builtin_ctz(m);
        if (!spu.voice[v].active)
            continue;
        active++;
        short s = voice_step(v);
        voice_envelope_step(&spu.voice[v]);
        if (spu.voice[v].env_state == ADSR_OFF) {
            s = 0;
        }
        // capture buffer stores sample pre-envelope, weirdly only after
        // processing the ADSR envelope -- this might need a re-test on real HW
        if (v == 1) {
            v1_sample = s;
        } else if (v == 3) {
            v3_sample = s;
        }
        rxx->voice[v].volumex = (unsigned short)spu.voice[v].env_vol;
        s = (short)(((int)s * spu.voice[v].env_vol) >> 15); // apply ADSR vol
        left_sum += (s * voice_vol(rxx->voice[v].volume.left)) >> 15;
        right_sum += (s * voice_vol(rxx->voice[v].volume.right)) >> 15;
    }

    // Mute all voices. CD audio is mixed after, and not affected by mute.
    if (!(spucnt & SPU_CTRL_MASK_MUTE_SPU)) {
        left_sum = right_sum = 0;
    }

    // Mix CD audio per SPUCNT and cd_vol registers.
    if (spucnt & SPU_CTRL_MASK_CD_AUDIO_ENABLE) {
        left_sum += (cd_left * rxx->cd_vol.left) >> 15;
        right_sum += (cd_right * rxx->cd_vol.right) >> 15;
    }

    psyz_spu_voices_active = active;
    out[0] = clamp16((left_sum * clamp15(rxx->main_vol.left)) >> 14);
    out[1] = clamp16((right_sum * clamp15(rxx->main_vol.right)) >> 14);

    // Capture buffers back to SPU RAM, as per real hardware
    write_capture(0, (short)cd_left);
    write_capture(1, (short)cd_right);
    write_capture(2, v1_sample);
    write_capture(3, v3_sample);

    // SPUSTAT bit 11 flips when capture_pos crosses 0x200
    unsigned prev_pos = spu.capture_pos;
    spu.capture_pos = (prev_pos + 2) & 0x3FF;
    if ((prev_pos ^ spu.capture_pos) & 0x200) {
        rxx->spustat ^= 1u << 11;
    }
}

#ifdef ESP_PLATFORM
// On PSX the RCnt handlers were IRQs on the ONE cpu; on the ESP32 the audio
// pull runs on core 1 and calling the handlers from there races the game
// (VSyncHandler/seq tickers touch GPU queues and prim chains). Samples are
// accumulated here and ticked from the game thread (Psyz_VideoVSync).
volatile int psyz_pending_rcnt;
#endif

/* spu_tick's frames, a block at a time and a voice at a time: within one pull
 * no register changes (the platform pulls up to the time of each register
 * write before making it), so each voice runs through the block with its
 * pitch and volumes read once, and the frames are mixed after. The sums are
 * the same integers in another order: the output is spu_tick's. (One
 * difference: a voice reading the capture area sees the block's captures
 * only after the block; nothing plays from there.) */
/* one voice through the block: voice_step + voice_envelope_step + the mix
 * of spu_block's loop, with the voice's state in locals */
static inline __attribute__((always_inline)) int voice_run(
    int v, int n, int* left, int* right, short* cap, int vol_l, int vol_r) {
    VoiceState* vs = &spu.voice[v];
    const unsigned pitch = _spu_RXX->rxx.voice[v].pitch & 0x3FFF;
    const unsigned sinc = pitch ? pitch << 4 : 1;
    vs->sinc = sinc;
    unsigned spos = vs->spos;
    // the gauss window oldest to newest, in registers: no ring index
    const unsigned gpos = vs->gpos;
    int g0 = vs->gwin[gpos & 3], g1 = vs->gwin[(gpos + 1) & 3];
    int g2 = vs->gwin[(gpos + 2) & 3], g3 = vs->gwin[(gpos + 3) & 3];
    // the decoded block's samples are read here until it runs out; then
    // voice_fetch (block ends, loops, decoding) takes over
    const short* const samples = vs->samples;
    unsigned idx = vs->sample_idx;
    unsigned lim = vs->needs_decode ? 0 : ADPCM_BLOCK_SAMPLES;
    EnvState e = {vs->env_vol, vs->env_counter, vs->env_state, vs->delay_ticks};
    const u16 adsr_lo = vs->adsr_lo, adsr_hi = vs->adsr_hi;
    const u8 key_off = vs->key_off;
    unsigned mask = env_mask(&e, adsr_lo, adsr_hi);
    int active = 1, i = 0;
// the voice's next pitch-resampled, gauss-interpolated sample into s,
// fetching as the pitch counter passes whole samples (at the voice's end,
// active goes 0)
#define VOICE_GAUSS(s)                                                         \
    do {                                                                       \
        while (spos >= 0x10000) {                                              \
            int d;                                                             \
            if (idx < lim) {                                                   \
                d = samples[idx++];                                            \
            } else {                                                           \
                short ds;                                                      \
                vs->sample_idx = (u8)idx;                                      \
                const int ok = voice_fetch(vs, &ds);                           \
                idx = vs->sample_idx;                                          \
                lim = ADPCM_BLOCK_SAMPLES;                                     \
                d = ds;                                                        \
                if (!ok) {                                                     \
                    g0 = g1, g1 = g2, g2 = g3, g3 = 0;                         \
                    active = 0;                                                \
                    spos = 0;                                                  \
                    break;                                                     \
                }                                                              \
            }                                                                  \
            g0 = g1, g1 = g2, g2 = g3, g3 = d;                                 \
            spos -= 0x10000;                                                   \
        }                                                                      \
        const short* const t = &spu_gauss_tbl[(spos >> 6) & ~3];               \
        int acc = (t[0] * g0) & ~2047;                                         \
        acc += (t[1] * g1) & ~2047;                                            \
        acc += (t[2] * g2) & ~2047;                                            \
        acc += (t[3] * g3) & ~2047;                                            \
        spos += sinc;                                                          \
        s = clamp16(acc >> 12);                                                \
    } while (0)
    while (i < n && active) {
        // a stretch where the envelope only counts: past the key-on delay,
        // no key-off to take, and the steps until its counter fires
        if (!e.delay && !(key_off && e.state < ADSR_RELEASE)) {
            const unsigned k = ~e.counter & mask;
            const int m = k < (unsigned)(n - i) ? (int)k : n - i;
            if (m > 0) {
                const int off = e.state == ADSR_OFF, vol = e.vol;
                const int start = i, end = i + m;
                for (; i < end && active; i++) {
                    short s;
                    VOICE_GAUSS(s);
                    if (off) {
                        s = 0;
                    }
                    if (cap) {
                        cap[i] = s;
                    }
                    s = (short)(((int)s * vol) >> 15);
                    left[i] += (s * vol_l) >> 15;
                    right[i] += (s * vol_r) >> 15;
                }
                e.counter += i - start;
                continue;
            }
        }
        // one step with env_step's work, the counter mask kept until the
        // phase or the volume changes
        short s;
        VOICE_GAUSS(s);
        if (e.delay) {
            e.delay--;
        } else {
            if (key_off && e.state < ADSR_RELEASE) {
                e.state = ADSR_RELEASE;
                mask = env_mask(&e, adsr_lo, adsr_hi);
            }
            if ((++e.counter & mask) == 0) {
                if (env_fire(&e, adsr_lo, adsr_hi)) {
                    active = 0;
                    vs->active = 0;
                    spu.active_mask &= ~(1u << v);
                }
                mask = env_mask(&e, adsr_lo, adsr_hi);
            }
        }
        if (e.state == ADSR_OFF) {
            s = 0;
        }
        if (cap) {
            cap[i] = s;
        }
        s = (short)(((int)s * e.vol) >> 15);
        left[i] += (s * vol_l) >> 15;
        right[i] += (s * vol_r) >> 15;
        i++;
    }
#undef VOICE_GAUSS
    vs->sample_idx = (u8)idx;
    vs->spos = spos;
    vs->gpos = 0;
    vs->gwin[0] = (short)g0;
    vs->gwin[1] = (short)g1;
    vs->gwin[2] = (short)g2;
    vs->gwin[3] = (short)g3;
    vs->env_vol = e.vol;
    vs->env_counter = e.counter;
    vs->env_state = e.state;
    vs->delay_ticks = e.delay;
    return i;
}

#define SPU_BLOCK 128
static void spu_block(short* out, int n) {
    SPU_RXX* rxx = (SPU_RXX*)&_spu_RXX->rxx;
    int left[SPU_BLOCK], right[SPU_BLOCK];
    short cap1[SPU_BLOCK], cap3[SPU_BLOCK];
    memset(left, 0, sizeof(int) * n);
    memset(right, 0, sizeof(int) * n);
    memset(cap1, 0, sizeof(short) * n);
    memset(cap3, 0, sizeof(short) * n);
    unsigned active_last = 0;
    for (unsigned m = spu.active_mask; m; m &= m - 1) {
        const int v = __builtin_ctz(m);
        VoiceState* vs = &spu.voice[v];
        if (!vs->active)
            continue;
        const int vol_l = voice_vol(rxx->voice[v].volume.left);
        const int vol_r = voice_vol(rxx->voice[v].volume.right);
        short* const cap = v == 1 ? cap1 : v == 3 ? cap3 : NULL;
        const int i = voice_run(v, n, left, right, cap, vol_l, vol_r);
        if (i > 0) {
            rxx->voice[v].volumex = (unsigned short)vs->env_vol;
        }
        active_last += i == n;
    }
    psyz_spu_voices_active = active_last;

    const unsigned short spucnt = rxx->spucnt;
    const int mute = !(spucnt & SPU_CTRL_MASK_MUTE_SPU);
    const int cd_on = (spucnt & SPU_CTRL_MASK_CD_AUDIO_ENABLE) != 0;
    const int main_l = clamp15(rxx->main_vol.left), main_r = clamp15(rxx->main_vol.right);
    for (int i = 0; i < n; i++) {
        if (spu.cd_ring_count < CD_RING_LOW_WATER) {
            size_t space = CD_RING_FRAMES - spu.cd_ring_count;
            size_t write_pos = (spu.cd_ring_read + spu.cd_ring_count) & CD_RING_MASK;
            size_t chunk = CD_RING_FRAMES - write_pos;
            if (chunk > space)
                chunk = space;
            size_t got = Psyz_CdPullSamples(&spu.cd_ring[write_pos * 2], chunk);
            spu.cd_ring_count += got;
        }
        int cd_left = 0, cd_right = 0;
        if (spu.cd_ring_count > 0) {
            cd_left = spu.cd_ring[spu.cd_ring_read * 2];
            cd_right = spu.cd_ring[spu.cd_ring_read * 2 + 1];
            spu.cd_ring_read = (spu.cd_ring_read + 1) & CD_RING_MASK;
            spu.cd_ring_count--;
        }
        int left_sum = mute ? 0 : left[i], right_sum = mute ? 0 : right[i];
        if (cd_on) {
            left_sum += (cd_left * rxx->cd_vol.left) >> 15;
            right_sum += (cd_right * rxx->cd_vol.right) >> 15;
        }
        out[i * 2] = clamp16((left_sum * main_l) >> 14);
        out[i * 2 + 1] = clamp16((right_sum * main_r) >> 14);
        write_capture(0, (short)cd_left);
        write_capture(1, (short)cd_right);
        write_capture(2, cap1[i]);
        write_capture(3, cap3[i]);
        unsigned prev_pos = spu.capture_pos;
        spu.capture_pos = (prev_pos + 2) & 0x3FF;
        if ((prev_pos ^ spu.capture_pos) & 0x200) {
            rxx->spustat ^= 1u << 11;
        }
    }
}

int psyz_spu_by_voice = 1; /* 0: spu_tick per frame (the reference) */

void Psyz_SpuPullSamples(short* out, int num_frames) {
    if (!spu.initialized) {
        memset(out, 0, num_frames * 2 * sizeof(short));
        return;
    }
#ifdef ESP_PLATFORM
    __atomic_fetch_add(&psyz_pending_rcnt, num_frames, __ATOMIC_RELAXED);
#else
    Psyz_RcntAdd(num_frames);
#endif
    if (psyz_spu_by_voice) {
        for (int i = 0; i < num_frames; i += SPU_BLOCK) {
            const int n = num_frames - i < SPU_BLOCK ? num_frames - i : SPU_BLOCK;
            spu_block(&out[i * 2], n);
        }
        return;
    }
    for (int i = 0; i < num_frames; i++) {
        spu_tick(&out[i * 2]);
    }
}

// platform diagnostics: the registers the IRQ comparison reads, and where a
// voice is fetching from
void Psyz_SpuDebug(unsigned* spucnt, unsigned* irq_addr, int v, unsigned* cur_addr,
                   unsigned* active) {
    *spucnt = _spu_RXX->rxx.spucnt;
    *irq_addr = _spu_RXX->rxx.irq_addr;
    *cur_addr = spu.voice[v].cur_addr;
    *active = spu.voice[v].active;
}
