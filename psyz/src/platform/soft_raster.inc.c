// PSY-Z software rasterizer - portable core.
//
// Plain C over a u16[512][1024] VRAM in native PSX RGB5551. No SDL, no OS, no
// allocations: this translation unit is compiled unchanged into both the PC
// harness (soft_gpu.c includes it) and the ESP32-S3 firmware. Anything
// platform-shaped belongs in the including shell, not here.
//
// It is written as an includable .inc.c on purpose: the shell provides `vram`
// resolution and the packet parser, the core provides state + rasterization +
// VRAM block ops, and both compile as ONE unit so the hot paths inline.

// ---- shared-macro shims -----------------------------------------------------
// When the PC shell includes sdl3_common.h first it already provides these;
// the ESP32 build defines SOFT_RASTER_STANDALONE and gets them here.
#ifdef SOFT_RASTER_STANDALONE
#include <stdbool.h>
// PS1_RECT is sdl3_common's rename of libgpu's RECT (dodging windef's on
// Windows); standalone builds have no windef to dodge.
#define PS1_RECT RECT
#define SEMITRANSP 0x02
#define TEXTURED 0x04
#define EXTRA_VERTEX 0x08
#define GOURAUD 0x10
#define TRIANGLE 0x20
/* Two rasterizer workers (one per core) execute the same command stream over
 * different bands of rows, so every piece of drawing state is per core:
 * the draw area and offset, the mask, the texpage mode, the CLUT cache. */
#ifdef ESP_PLATFORM
#include "esp_cpu.h"
static inline int soft_core(void) { return (int)esp_cpu_get_core_id(); }
#elif defined(SOFT_RASTER_HOST_CORES)
/* a desktop shell running two workers says which one each thread is */
extern __thread int psyz_host_core;
static inline int soft_core(void) { return psyz_host_core; }
#else
static inline int soft_core(void) { return 0; }
#endif
static unsigned short cur_tpage_c[2];
#define cur_tpage cur_tpage_c[soft_core()]
#ifdef ESP_PLATFORM
#include "psyz_prof.h"
PsyzProf psyz_prof_c[2];
#define PROFC (psyz_prof_c[soft_core()])
#endif
static short s11(short v) { return (short)(((v & 0x7FF) ^ 1024) - 1024); }
#ifndef WARNF
#define WARNF(...)
#endif
void Draw_SetTexpageMode(ParamDrawTexpageMode* p) {
    cur_tpage = (*(unsigned short*)p) & 0x1FF;
}
void Draw_SetTextureWindow(unsigned int mask_x, unsigned int mask_y,
                           unsigned int off_x, unsigned int off_y) {
    (void)mask_x; (void)mask_y; (void)off_x; (void)off_y;
}
#endif // SOFT_RASTER_STANDALONE

// ==================== core (SDL-free, ESP32-ready) ====================

// The game's VRAM when linked into SOTN; local fallback otherwise. Declared
// weak so psyz still links standalone.
#ifdef SOFT_RASTER_NO_FALLBACK
// The embedding shell owns VRAM placement outright (the ESP32 build puts it in
// PSRAM with an explicit attribute); a megabyte of fallback .bss would not
// even link against 512KB of internal RAM.
#elif defined(_MSC_VER)
static u16 soft_vram_fallback[VRAM_H * VRAM_W];
#define SOFT_VRAM soft_vram_fallback
#else
#ifdef ESP_PLATFORM
extern u16* g_RawVram; /* port/esp32_arena.c */
#else
extern u16 g_RawVram[VRAM_H * VRAM_W] __attribute__((weak));
#endif
static u16 soft_vram_fallback[VRAM_H * VRAM_W];
#define SOFT_VRAM (g_RawVram ? g_RawVram : soft_vram_fallback)
#endif

static u16* vram; // resolved once in InitPlatform

#ifdef SOFT_TILED_TEX
/* Texture fetches read a tiled shadow of VRAM instead of VRAM itself.
 * VRAM is linear, 2 KB per row: a texture walked along v touches a new cache
 * line for every texel, and on this board that line change costs ~125
 * cycles whether the page is warm or cold (measured: 60 cy/px along u,
 * 185 along v). The shadow stores 8x8-halfword tiles (128 B, one L2 line),
 * so a line holds an 8-row block of texels and the v walk changes lines
 * once per 8 texels. The shadow is rebuilt lazily: every VRAM write marks
 * its 64-halfword x 256-row regions dirty (one bit each, 32 regions), and
 * tex_from re-tiles the dirty regions a page covers before the primitive is
 * drawn. Both rasterizer workers share the dirty word atomically; two
 * workers re-tiling the same region write identical bytes. */
#include <stdlib.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
static u16* vram_tiled;
/* dirty regions: 64 halfwords x 32 rows, bit (row * 16 + col), row = y >> 5,
 * col = x >> 6; 256 bits in 8 words, a word per pair of rows. Small, because
 * the game redraws parts of the texture area every frame (the radar, the
 * movie strips) and whatever is dirty is re-copied by the next primitive
 * that samples it. */
static volatile uint32_t tex_dirty[8] = {~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u};
#define TEX_HW(tbase, page, hx, v) \
    vram_tiled[(tbase) + (((v) >> 3) << 13) + (((hx) >> 3) << 6) + (((v) & 7) << 3) + ((hx) & 7)]
static inline void tex_mark_dirty(int x0, int y0, int x1, int y1) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > VRAM_W - 1) x1 = VRAM_W - 1;
    if (y1 > VRAM_H - 1) y1 = VRAM_H - 1;
    if (x0 > x1 || y0 > y1) {
        return;
    }
    const uint32_t m16 = (2u << (x1 >> 6)) - (1u << (x0 >> 6));
    for (int r = y0 >> 5; r <= (y1 >> 5); r++) {
        const uint32_t m = m16 << ((r & 1) * 16);
        if ((tex_dirty[r >> 1] & m) != m) {
            __atomic_fetch_or(&tex_dirty[r >> 1], m, __ATOMIC_RELAXED);
        }
    }
}
static void tex_retile_region(int col, int row) {
#ifdef ESP_PLATFORM
    const uint32_t t0 = esp_cpu_get_cycle_count();
#endif
    for (int y = row * 32; y < row * 32 + 32; y++) {
        const uint32_t* src = (const uint32_t*)(vram + y * VRAM_W + col * 64);
        uint32_t* dst = (uint32_t*)(vram_tiled + ((y >> 3) << 13) + ((col * 8) << 6) + ((y & 7) << 3));
        for (int tx = 0; tx < 8; tx++, src += 4, dst += 32) {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
        }
    }
#ifdef ESP_PLATFORM
    PROFC.retile_cy += esp_cpu_get_cycle_count() - t0;
    PROFC.retile_n++;
#endif
}
/* re-tile the dirty regions a page (ncols columns, 256 rows from py) covers.
 * The bits are cleared first, then copied: a write that lands during the
 * copy re-marks the region and it is re-tiled next time, never missed. */
static inline void tex_sync_page(int px, int py, int ncols) {
    uint32_t cm = ((1u << ncols) - 1) << (px >> 6);
    const uint32_t m16 = (cm | (cm >> 16)) & 0xFFFFu;
    const uint32_t need = m16 | (m16 << 16);
    const int w0 = py >> 6; /* 2 rows of 32 per word: 4 words per 256 rows */
    for (int w = w0; w < w0 + 4; w++) {
        if (tex_dirty[w] & need) {
            uint32_t bits = __atomic_fetch_and(&tex_dirty[w], ~need, __ATOMIC_ACQ_REL) & need;
            while (bits) {
                const int b = __builtin_ctz(bits);
                bits &= bits - 1;
                tex_retile_region(b & 15, w * 2 + (b >> 4));
            }
        }
    }
}
static inline void tex_dirty_all(void) {
    for (int w = 0; w < 8; w++) {
        tex_dirty[w] = ~0u;
    }
}
void Draw_TiledInit(void) {
    if (!vram_tiled) {
        vram_tiled = heap_caps_malloc((size_t)VRAM_W * VRAM_H * sizeof(u16), MALLOC_CAP_SPIRAM);
        if (!vram_tiled) {
            printf("[soft] no PSRAM for the tiled texture shadow\n");
            abort();
        }
    }
    tex_dirty_all();
}
void** Draw_TiledSlot(void) { return (void**)&vram_tiled; } /* for save states */
#else
#define TEX_HW(tbase, page, hx, v) (page)[((v) << vshift) + (hx)]
void Draw_TiledInit(void) {}
static void* tiled_none;
void** Draw_TiledSlot(void) { return &tiled_none; }
#ifdef SOFT_MIP_TEX
/* Decimated copies of VRAM for minified textures. A triangle far from the
 * camera steps several texels per pixel, so every pixel's texel sits on a
 * fresh cache line (~125 cycles on this board) and the fetch dominates the
 * span. Level k holds every 2^k-th texel in both directions, in the same
 * packed format as VRAM but at 1/2^k the width and height, so a minified
 * triangle sampled at level k walks the shadow at about one texel per pixel.
 * The format depends on how a page is read (4/8/15 bpp), which VRAM does
 * not record: there is a shadow per (bpp, level), each built lazily.
 * Staleness: every VRAM write bumps a generation per 64x32 region; a page
 * sampled at level k rebuilds the regions whose generation moved since the
 * shadow was built. The frame buffer and the radar are sampled at level 0,
 * which always reads VRAM itself. */
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
#define MIP_LEVELS 3
int psyz_mip_enable = 0; /* measured no gain on the dock replay frame; kept for experiments */
static u16* mip_shadow[3][MIP_LEVELS];       /* [bpp][level-1] */
static uint32_t* region_gen;                 /* [256], internal RAM */
static uint32_t mip_built[3][MIP_LEVELS][256]; /* generation each region was built at */
static inline void tex_mark_dirty(int x0, int y0, int x1, int y1) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > VRAM_W - 1) x1 = VRAM_W - 1;
    if (y1 > VRAM_H - 1) y1 = VRAM_H - 1;
    if (x0 > x1 || y0 > y1 || !region_gen) {
        return;
    }
    for (int r = y0 >> 5; r <= (y1 >> 5); r++) {
        for (int c = x0 >> 6; c <= (x1 >> 6); c++) {
            __atomic_fetch_add(&region_gen[r * 16 + c], 1u, __ATOMIC_RELAXED);
        }
    }
}
static void mip_build_region(int bpp, int k, int col, int row) {
#ifdef ESP_PLATFORM
    const uint32_t t0 = esp_cpu_get_cycle_count();
#endif
    u16* const out = mip_shadow[bpp][k - 1];
    const int ow = VRAM_W >> k;
    for (int yo = (row * 32) >> k; yo < ((row + 1) * 32) >> k; yo++) {
        const u16* src = vram + (yo << k) * VRAM_W;
        u16* dst = out + yo * ow;
        const int hx0 = (col * 64) >> k, hx1 = ((col + 1) * 64) >> k;
        if (bpp == 2) {
            for (int hx = hx0; hx < hx1; hx++) {
                dst[hx] = src[hx << k];
            }
        } else if (bpp == 1) {
            for (int hx = hx0; hx < hx1; hx++) {
                const int t0x = hx << (k + 1), t1x = t0x + (1 << k);
                const unsigned b0 = (src[t0x >> 1] >> ((t0x & 1) * 8)) & 0xFF;
                const unsigned b1 = (src[t1x >> 1] >> ((t1x & 1) * 8)) & 0xFF;
                dst[hx] = (u16)(b0 | (b1 << 8));
            }
        } else {
            for (int hx = hx0; hx < hx1; hx++) {
                unsigned o = 0;
                for (int i = 0; i < 4; i++) {
                    const int tx = (hx << (k + 2)) + (i << k);
                    o |= ((src[tx >> 2] >> ((tx & 3) * 4)) & 0xF) << (i * 4);
                }
                dst[hx] = (u16)o;
            }
        }
    }
#ifdef ESP_PLATFORM
    PROFC.retile_cy += esp_cpu_get_cycle_count() - t0;
    PROFC.retile_n++;
#endif
}
void Draw_MipInit(void) {
    if (!region_gen) {
        region_gen = heap_caps_calloc(256, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
    }
    memset(mip_built, 0xFF, sizeof mip_built);
}
/* save states: the shadow pointers and the generation table are this run's */
void** Draw_MipSlot(int* count) { *count = 3 * MIP_LEVELS; return (void**)mip_shadow; }
void** Draw_MipGenSlot(void) { return (void**)&region_gen; }
static inline void mip_flush(void) { memset(mip_built, 0xFF, sizeof mip_built); }
#else
#define tex_mark_dirty(x0, y0, x1, y1) ((void)0)
void Draw_MipInit(void) {}
static void* mip_none[9];
static void* mip_gen_none;
void** Draw_MipSlot(int* count) { *count = 9; return mip_none; }
void** Draw_MipGenSlot(void) { return &mip_gen_none; }
#define mip_flush() ((void)0)
#endif
#endif

typedef struct {
    int x0, y0, x1, y1; // effective clip, inclusive: draw area within the band
    int ox, oy;         // draw offset
    // GP0 E6, the mask bit: mask_or is 0x8000 while "set mask bit on drawn
    // pixels" is on; mask_check skips pixels whose bit 15 is set.
    u16 mask_or;
    int mask_check;
    int area_y0, area_y1; // the draw area as set by E3/E4
    int band_y0, band_y1; // this worker's rows
} SoftState;
static SoftState st_c[2] = {
    {0, 0, VRAM_W - 1, VRAM_H - 1, 0, 0, 0, 0, 0, VRAM_H - 1, 0, VRAM_H - 1},
    {0, 0, VRAM_W - 1, VRAM_H - 1, 0, 0, 0, 0, 0, VRAM_H - 1, 0, VRAM_H - 1},
};
#define st st_c[soft_core()]
/* a frame that samples the frame buffer as a texture (the zoom blur, the
 * water refraction) must be drawn by one worker: the other band's rows may
 * not be there yet */
int psyz_fb_texture_seen;

// every pixel write goes through this: honours both mask modes
#define MASK_WRITE(dstp, val)                                                  \
    do {                                                                       \
        u16* mw_d_ = (dstp);                                                   \
        if (!(st.mask_check && (*mw_d_ & 0x8000))) {                           \
            *mw_d_ = (u16)((val) | st.mask_or);                                \
        }                                                                      \
    } while (0)
// true while either mode is on: the paired-store fast paths step aside
#define MASK_ACTIVE() (st.mask_or | st.mask_check)

#ifdef SOFT_RASTER_STANDALONE
void Draw_SetMask(int bit0, int bit1) {
    st.mask_or = bit0 ? 0x8000 : 0;
    st.mask_check = bit1;
}
#endif


// One vertex after packet parsing. Colors stay 8-bit until the pixel write.
typedef struct {
    int x, y;
    int u, v;
    int r, g, b;
} SV;

// The embedded build attributes the rasterizer loops into IRAM and swaps the
// PSRAM CLUT pointer for a small internal-SRAM cache; on PC both default off.
#ifndef SOFT_RASTER_IRAM
#define SOFT_RASTER_IRAM
#endif

#ifdef SOFT_CLUT_CACHE
// 8-way store of recently used CLUTs (4KB DRAM). Texel fetches then cost one
// PSRAM read instead of two. Any VRAM image op invalidates everything - CLUT
// uploads and palette animation go through LoadImage.
#ifdef ESP_PLATFORM
/* a platform may place this file's statics in PSRAM wholesale (a linker
 * fragment covering the archive); the cache is the one thing that must be
 * internal, so it is taken from the internal heap on first use */
#include "esp_heap_caps.h"
static u16 (*clut_cache_c[2])[256];
#define clut_cache clut_cache_c[soft_core()]
#else
static u16 clut_cache_storage[8][256];
#define clut_cache clut_cache_storage
#endif
static u16 clut_cache_key_c[2][8] = {{0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                     {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}};
#define clut_cache_key clut_cache_key_c[soft_core()]
static u8 clut_cache_next_c[2];
#define clut_cache_next clut_cache_next_c[soft_core()]
static void clut_cache_flush(void) {
    /* both cores': a VRAM block operation executed on one worker (MoveImage
     * between barriers) invalidates the other's cached palettes too */
    for (int c = 0; c < 2; c++) {
        for (int i = 0; i < 8; i++) {
            clut_cache_key_c[c][i] = 0xFFFF;
        }
    }
}
#ifdef ESP_PLATFORM
/* for save states: the cache memory is this run's, its contents stale */
void** Draw_ClutCacheSlot(void) { return (void**)clut_cache_c; } /* both cores' */
/* at cart shutdown: every heap allocation of the rasterizer (the statics
 * holding the pointers are reset before the next launch) */
void Draw_FreeCaches(void) {
    for (int c = 0; c < 2; c++) {
        free(clut_cache_c[c]);
        clut_cache_c[c] = NULL;
        for (int i = 0; i < 8; i++) {
            clut_cache_key_c[c][i] = 0xFFFF;
        }
    }
#ifdef SOFT_TILED_TEX
    free(vram_tiled);
    vram_tiled = NULL;
#endif
#ifdef SOFT_MIP_TEX
    free(region_gen);
    region_gen = NULL;
    for (int b = 0; b < 3; b++) {
        for (int k = 0; k < MIP_LEVELS; k++) {
            free(mip_shadow[b][k]);
            mip_shadow[b][k] = NULL;
        }
    }
#endif
}
void Draw_FlushCaches(void) {
#ifdef SOFT_TILED_TEX
    tex_dirty_all();
#else
    mip_flush();
#endif
    for (int c = 0; c < 2; c++) {
        for (int i = 0; i < 8; i++) {
            clut_cache_key_c[c][i] = 0xFFFF;
        }
    }
}
#endif
static inline const u16* clut_cached(u16 id, const u16* src, int n) {
#ifdef ESP_PLATFORM
    if (!clut_cache) {
        clut_cache = heap_caps_malloc(8 * 256 * sizeof(u16), MALLOC_CAP_INTERNAL);
        if (!clut_cache) {
            return src;
        }
        clut_cache_flush();
    }
#endif
    for (int i = 0; i < 8; i++) {
        if (clut_cache_key[i] == id) {
            return clut_cache[i];
        }
    }
    int slot = clut_cache_next++ & 7;
    memcpy(clut_cache[slot], src, n * sizeof(u16));
    clut_cache_key[slot] = id;
    return clut_cache[slot];
}
#else
#define clut_cache_flush()
#endif

// ---- pixel write -----------------------------------------------------------

// Semi-transparency modes, tpage bits 5-6. B = background (dest), F = front.
//   0: B/2 + F/2    1: B + F    2: B - F    3: B + F/4
static inline u16 blend5551(u16 back, int fr5, int fg5, int fb5, int abr) {
    int br = back & 0x1F, bg = (back >> 5) & 0x1F, bb = (back >> 10) & 0x1F;
    int r, g, b;
    switch (abr) {
    case 0:
        r = (br + fr5) >> 1;
        g = (bg + fg5) >> 1;
        b = (bb + fb5) >> 1;
        break;
    case 1:
        r = br + fr5;
        g = bg + fg5;
        b = bb + fb5;
        break;
    case 2:
        r = br - fr5;
        g = bg - fg5;
        b = bb - fb5;
        break;
    default:
        r = br + (fr5 >> 2);
        g = bg + (fg5 >> 2);
        b = bb + (fb5 >> 2);
        break;
    }
    if (r < 0) r = 0; else if (r > 31) r = 31;
    if (g < 0) g = 0; else if (g > 31) g = 31;
    if (b < 0) b = 0; else if (b > 31) b = 31;
    return (u16)(r | (g << 5) | (b << 10));
}

// PS1 texture modulation: out5 = min(31, tex5 * col8 >> 7). 0x80 is neutral.
static inline int mod5(int tex5, int col8) {
    int v = (tex5 * col8) >> 7;
    return v > 31 ? 31 : v;
}

// ---- texture fetch ---------------------------------------------------------

typedef struct {
    const u16* page;  // top-left of the texture page in VRAM
    const u16* clut;  // 16 or 256 entries
    int bpp;          // 0=4bpp, 1=8bpp, 2=15bpp
    int abr;          // semi-transparency mode from the tpage
    int tbase;        // tiled shadow: halfword index of the page's first tile
    int px, py;       // page origin in VRAM, halfwords
    int vshift;       // log2 of the page's row stride in halfwords (10, or less in a mip level)
    int umask, vmask; // texel coordinate wrap (0xFF, or less in a mip level)
    int uor, vor;     // GP0 E2 texture window: u = (u & umask) | uor
} Tex;

/* GP0 E2, the texture window, per core like the rest of the drawing state:
 * texcoord = (texcoord & ~(mask * 8)) | ((offset & mask) * 8). All zero
 * (the default) is no window. */
typedef struct { int uand, uor, vand, vor; } TexWindow;
static TexWindow tw_c[2] = {{0xFF, 0, 0xFF, 0}, {0xFF, 0, 0xFF, 0}};
#define tw tw_c[soft_core()]
void Draw_SetTextureWindowE2(unsigned int e2) {
    const int mx = e2 & 0x1F, my = (e2 >> 5) & 0x1F;
    const int ox = (e2 >> 10) & 0x1F, oy = (e2 >> 15) & 0x1F;
    tw.uand = 0xFF & ~(mx << 3);
    tw.uor = (ox & mx) << 3;
    tw.vand = 0xFF & ~(my << 3);
    tw.vor = (oy & my) << 3;
}

static inline Tex tex_from(u16 tpage, u16 clut) {
    Tex t;
    int px = (tpage & 0xF) * 64;         // halfword units
    int py = ((tpage >> 4) & 1) * 256;
    t.page = vram + py * VRAM_W + px;
    t.bpp = (tpage >> 7) & 3;
    if (t.bpp >= 2 && py == 0 && px < 640) {
        psyz_fb_texture_seen = 1;
    }
    t.tbase = ((py >> 3) << 13) + ((px >> 3) << 6);
    t.px = px;
    t.py = py;
    t.vshift = 10;
    t.umask = tw.uand;
    t.vmask = tw.vand;
    t.uor = tw.uor;
    t.vor = tw.vor;
#ifdef SOFT_TILED_TEX
    /* the page covers 1, 2 or 4 columns of 64 halfwords */
    tex_sync_page(px, py, 1 << (t.bpp > 2 ? 2 : t.bpp));
#endif
    t.abr = (tpage >> 5) & 3;
    t.clut = vram + ((clut >> 6) & 0x1FF) * VRAM_W + (clut & 0x3F) * 16;
#ifdef SOFT_CLUT_CACHE
    if (t.bpp <= 1) {
        t.clut = (u16*)clut_cached(clut, t.clut, t.bpp == 0 ? 16 : 256);
    }
#endif
    return t;
}

#ifdef SOFT_MIP_TEX
/* point t at level k of its page, rebuilding stale regions first */
static void tex_mip(Tex* t, int k) {
    const int bpp = t->bpp > 2 ? 2 : t->bpp;
    if (!mip_shadow[bpp][k - 1]) {
        mip_shadow[bpp][k - 1] = heap_caps_malloc(((size_t)VRAM_W >> k) * (VRAM_H >> k) * sizeof(u16),
                                                  MALLOC_CAP_SPIRAM);
        if (!mip_shadow[bpp][k - 1]) {
            return; /* stay at level 0 */
        }
        memset(mip_built[bpp][k - 1], 0xFF, sizeof mip_built[bpp][k - 1]);
    }
    const int ncols = 1 << bpp, col0 = t->px >> 6, row0 = t->py >> 5;
    for (int r = row0; r < row0 + 8; r++) {
        for (int c = 0; c < ncols; c++) {
            const int reg = r * 16 + ((col0 + c) & 15);
            const uint32_t gen = region_gen[reg];
            if (mip_built[bpp][k - 1][reg] != gen) {
                mip_built[bpp][k - 1][reg] = gen; /* before the copy: a write during it re-marks */
                mip_build_region(bpp, k, (col0 + c) & 15, r);
            }
        }
    }
    t->page = mip_shadow[bpp][k - 1] + (t->py >> k) * (VRAM_W >> k) + (t->px >> k);
    t->vshift = 10 - k;
    t->umask = 0xFF >> k;
    t->vmask = 0xFF >> k;
    t->uor = t->vor = 0; /* (minified triangles ignore a texture window) */
}
#endif

static inline u16 tex_fetch(const Tex* t, int u, int v) {
    const int vshift = t->vshift;
    (void)vshift;
    u = (u & t->umask) | t->uor;
    v = (v & t->vmask) | t->vor;
    switch (t->bpp) {
    case 0: { // 4bpp: 4 texels per halfword
        u16 hw = TEX_HW(t->tbase, t->page, u >> 2, v);
        return t->clut[(hw >> ((u & 3) * 4)) & 0xF];
    }
    case 1: { // 8bpp: 2 texels per halfword
        u16 hw = TEX_HW(t->tbase, t->page, u >> 1, v);
        return t->clut[(hw >> ((u & 1) * 8)) & 0xFF];
    }
    default: // 15bpp direct
        return TEX_HW(t->tbase, t->page, u, v);
    }
}

// ---- primitive rasterizers -------------------------------------------------

// Shared per-pixel tail for textured pixels. Returns without writing on the
// PSX "texel 0000 is never drawn" rule; blends only texels with the STP bit
// when the primitive is semi-transparent.
#ifdef ESP_PLATFORM
unsigned psyz_texel_zero, psyz_texel_ok, psyz_flat_px;
/* per-pixel statistics are read-modify-writes in the inner loops: opt-in */
#ifdef SOFT_RASTER_STATS
#define STAT_INC(c) ((c)++)
#else
#define STAT_INC(c) ((void)0)
#endif
/* where the raster time goes, by primitive class: cycles and calls for
 * textured triangles, generic (semi-transparent) flat triangles, fast-path
 * flat triangles, sprites, tiles, lines */
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#define psyz_prof_cy PROFC.cy
#define psyz_prof_n PROFC.n
/* finer: textured rows, textured pixels, cycles inside the span loops, cycles
 * in tex_from (CLUT cache) */
#define psyz_prof_rows PROFC.rows
#define psyz_prof_px PROFC.px
#define psyz_prof_span_cy PROFC.span_cy
#define psyz_prof_texfrom_cy PROFC.texfrom_cy
#define psyz_tri_off PROFC.tri_off
#define psyz_tri_tiny PROFC.tri_tiny
#define psyz_tri_ok PROFC.tri_ok
#define psyz_tri_huge PROFC.tri_huge

#define PROF_BEGIN() uint32_t prof_t0_ = esp_cpu_get_cycle_count()
#define PROF_END(k) (psyz_prof_cy[k] += esp_cpu_get_cycle_count() - prof_t0_, psyz_prof_n[k]++)
/* tri_huge: polygons the PSX GPU would have refused for exceeding 1023x511 */
#endif
static inline void put_texel(u16* dst, u16 texel, int r8, int g8, int b8,
                             int semi, int abr) {
    if (texel == 0) {
#ifdef ESP_PLATFORM
        STAT_INC(psyz_texel_zero);
#endif
        return;
    }
#ifdef ESP_PLATFORM
    STAT_INC(psyz_texel_ok);
#endif
    // 0x80 modulation is neutral: mod5(t,128)==t, so the whole mul/clamp
    // chain collapses to a passthrough (bit-exact with the general form)
    if (r8 == 0x80 && g8 == 0x80 && b8 == 0x80) {
        if (semi && (texel & 0x8000)) {
            // A textured pixel keeps the texel's own bit 15 when written
            // (the GPU ORs the texel's STP bit into the result). Dropping it
            // here turned every frame-buffer feedback effect -- the zoom
            // blur, the underwater refraction -- into a one-way copy: the
            // next frame found no STP bit, drew its copy opaque, and the
            // picture froze.
            MASK_WRITE(dst, blend5551(*dst, texel & 0x1F, (texel >> 5) & 0x1F,
                                      (texel >> 10) & 0x1F, abr) | 0x8000);
        } else {
            MASK_WRITE(dst, texel);
        }
        return;
    }
    int r = mod5(texel & 0x1F, r8);
    int g = mod5((texel >> 5) & 0x1F, g8);
    int b = mod5((texel >> 10) & 0x1F, b8);
    if (semi && (texel & 0x8000)) {
        MASK_WRITE(dst, blend5551(*dst, r, g, b, abr) | 0x8000);
    } else {
        MASK_WRITE(dst, (u16)(r | (g << 5) | (b << 10) | (texel & 0x8000)));
    }
}

static inline void put_flat(u16* dst, int r8, int g8, int b8, int semi,
                            int abr) {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
    psyz_flat_px++;
#endif
    int r = r8 >> 3, g = g8 >> 3, b = b8 >> 3;
    if (semi) {
        MASK_WRITE(dst, blend5551(*dst, r, g, b, abr));
    } else {
        MASK_WRITE(dst, (u16)(r | (g << 5) | (b << 10)));
    }
}

// Axis-aligned textured sprite (SPRT/SPRT_8/SPRT_16). PSX-exact sampling:
// u advances one texel per pixel, no interpolation.
SOFT_RASTER_IRAM static void raster_sprite(const SV* v, int w, int h, u16 tpage, u16 clut,
                          int semi) {
    Tex t = tex_from(tpage, clut);
    const int vshift = t.vshift;
    (void)vshift;
    int x0 = v->x + st.ox, y0 = v->y + st.oy;
    // clip once instead of testing every pixel
    int i0 = 0, i1 = w - 1, j0 = 0, j1 = h - 1;
    if (x0 + i0 < st.x0) i0 = st.x0 - x0;
    if (x0 + i1 > st.x1) i1 = st.x1 - x0;
    if (y0 + j0 < st.y0) j0 = st.y0 - y0;
    if (y0 + j1 > st.y1) j1 = st.y1 - y0;
    if (i0 > i1 || j0 > j1) {
        return;
    }
    tex_mark_dirty(x0 + i0, y0 + j0, x0 + i1, y0 + j1);
    for (int j = j0; j <= j1; j++) {
        u16* row = vram + (y0 + j) * VRAM_W;
        int tv = ((v->v + j) & t.vmask) | t.vor;
        int i = i0;
        if (t.bpp == 0) {
            // 4bpp: one halfword read serves four texels
            // Tilemap fast path: opaque, neutral modulation, u aligned to a
            // halfword. Four texels per source read, pixel pairs written as
            // one 32-bit store. Per-texel semantics are identical to the
            // general path (texel 0 stays transparent, neutral mod is a
            // passthrough), so output is bit-for-bit the same.
            if (!semi && !MASK_ACTIVE() && v->r == 0x80 && v->g == 0x80 && v->b == 0x80 &&
                ((v->u + i0) & 3) == 0 && ((i1 - i0) & 3) == 3) {
                u16* dst = &row[x0 + i0];
                for (int n = i0; n <= i1; n += 4) {
                    u16 hw = TEX_HW(t.tbase, t.page, (((v->u + n) & t.umask) | t.uor) >> 2, tv);
                    u16 a = t.clut[hw & 0xF];
                    u16 b = t.clut[(hw >> 4) & 0xF];
                    u16 c = t.clut[(hw >> 8) & 0xF];
                    u16 d = t.clut[(hw >> 12) & 0xF];
                    if (a && b) {
                        *(u32*)dst = (u32)a | ((u32)b << 16);
                    } else {
                        if (a) dst[0] = a;
                        if (b) dst[1] = b;
                    }
                    if (c && d) {
                        *(u32*)(dst + 2) = (u32)c | ((u32)d << 16);
                    } else {
                        if (c) dst[2] = c;
                        if (d) dst[3] = d;
                    }
                    dst += 4;
                }
                continue;
            }
            while (i <= i1) {
                /* (a window is a multiple of 8 texels wide: the run of up to
                 * four texels in one halfword never crosses its edge) */
                int u = ((v->u + i) & t.umask) | t.uor;
                u16 hw = TEX_HW(t.tbase, t.page, u >> 2, tv);
                int nib = u & 3;
                do {
                    put_texel(&row[x0 + i], t.clut[(hw >> (nib * 4)) & 0xF],
                              v->r, v->g, v->b, semi, t.abr);
                    i++;
                    nib++;
                } while (nib < 4 && i <= i1);
            }
        } else {
            for (; i <= i1; i++) {
                put_texel(&row[x0 + i], tex_fetch(&t, v->u + i, tv), v->r,
                          v->g, v->b, semi, t.abr);
            }
        }
    }
}

// Flat rectangle (TILE family).
SOFT_RASTER_IRAM static void raster_tile(const SV* v, int w, int h, int semi, int abr) {
    int x0 = v->x + st.ox, y0 = v->y + st.oy;
    int xa = x0 < st.x0 ? st.x0 : x0;
    int xb = x0 + w - 1 > st.x1 ? st.x1 : x0 + w - 1;
    int ya = y0 < st.y0 ? st.y0 : y0;
    int yb = y0 + h - 1 > st.y1 ? st.y1 : y0 + h - 1;
    tex_mark_dirty(xa, ya, xb, yb);

    /* Opaque fill: nothing to read, nothing to interpolate, one colour for
     * every pixel -- so the general per-pixel path is all overhead. It matters
     * far more than a tile normally would, because MGS clears the frame with
     * ONE 320x224 tile: 71,680 pixels, more than half of everything the
     * rasterizer touches in a frame, each costing a branch and a single 16-bit
     * store into PSRAM. Writing pairs as one 32-bit store halves the store
     * count and lets the memory bus move a full word per transaction. Output
     * is bit-for-bit what put_flat produced. */
    if (!semi && !MASK_ACTIVE()) {
        u16 c = (u16)((v->r >> 3) | ((v->g >> 3) << 5) | ((v->b >> 3) << 10));
        u32 cc = (u32)c | ((u32)c << 16);
        for (int y = ya; y <= yb; y++) {
            u16* p = vram + y * VRAM_W + xa;
            int n = xb - xa + 1;
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            psyz_flat_px += (unsigned)(n > 0 ? n : 0);
#endif
            if ((xa & 1) && n > 0) {   /* odd start: one halfword to align */
                *p++ = c;
                n--;
            }
            {
                u32* q = (u32*)p;
                for (int k = n >> 1; k > 0; k--) {
                    *q++ = cc;
                }
                if (n & 1) {
                    *(u16*)q = c;
                }
            }
        }
        return;
    }

    for (int y = ya; y <= yb; y++) {
        u16* row = vram + y * VRAM_W;
        for (int x = xa; x <= xb; x++) {
            put_flat(&row[x], v->r, v->g, v->b, semi, abr);
        }
    }
}

// Triangle with gouraud + affine UV, integer edge functions, top-left rule.
// 2D game, orthographic: affine interpolation is exact.
// ---- specialised textured spans --------------------------------------------
//
// The generic per-pixel loop decided bpp, shading, blending and modulation
// anew for every texel, and on an in-order core those branches plus the
// reloads they force cost more than the texel itself. Here the decisions are
// template parameters of an always-inline body: each call site below with
// literal arguments becomes its own straight-line loop. Pixel semantics are
// those of put_texel (texel 0 never drawn, STP-gated blending, 0x80 neutral
// modulation), so output is bit-exact with the generic loop.
typedef struct {
    u16* dst;
    int n;
    int32_t ui, vi, udx, vdx;
    int32_t ri, gi, bi, rdx, gdx, bdx;
    int r8, g8, b8;
    const u16* lut; // flat modulation: [0..31] r, [32..63] g<<5, [64..95] b<<10
} TexSpan;

static inline __attribute__((always_inline)) void tex_span_body(
    const Tex* t, const TexSpan* sp, const int bpp, const int gouraud,
    const int semi, const int modulate, const int clamp) {
    u16* __restrict dst = sp->dst;
    const u16* __restrict page = t->page;
    const int tbase = t->tbase;
    const int vshift = t->vshift, umask = t->umask, vmask = t->vmask;
    const int uor = t->uor, vor = t->vor;
    (void)page;
    (void)tbase;
    (void)vshift;
    const u16* __restrict clut = t->clut;
    const int abr = t->abr;
    int32_t ui = sp->ui, vi = sp->vi;
    const int32_t udx = sp->udx, vdx = sp->vdx;
    int32_t ri = sp->ri, gi = sp->gi, bi = sp->bi;
    const int32_t rdx = sp->rdx, gdx = sp->gdx, bdx = sp->bdx;
    const u16* __restrict lut = sp->lut;
    const int mchk = st.mask_check;
    const u16 mor = st.mask_or;
    for (int n = sp->n; n > 0; n--, dst++, ui += udx, vi += vdx) {
        const int u = ((ui >> 16) & umask) | uor;
        const int v = ((vi >> 16) & vmask) | vor;
        u16 texel;
        if (bpp == 0) {
            const u16 hw = TEX_HW(tbase, page, u >> 2, v);
            texel = clut[(hw >> ((u & 3) << 2)) & 0xF];
        } else if (bpp == 1) {
            const u16 hw = TEX_HW(tbase, page, u >> 1, v);
            texel = clut[(hw >> ((u & 1) << 3)) & 0xFF];
        } else {
            texel = TEX_HW(tbase, page, u, v);
        }
        int r8 = 0, g8 = 0, b8 = 0;
        if (gouraud) {
            r8 = (int)(ri >> 16);
            g8 = (int)(gi >> 16);
            b8 = (int)(bi >> 16);
            ri += rdx; gi += gdx; bi += bdx;
        }
        if (texel == 0) {
            continue;
        }
        if (mchk && (*dst & 0x8000)) {
            continue;
        }
        if (!modulate && !gouraud) {
            if (semi && (texel & 0x8000)) {
                *dst = blend5551(*dst, texel & 0x1F, (texel >> 5) & 0x1F,
                                 (texel >> 10) & 0x1F, abr) | 0x8000 | mor;
            } else {
                *dst = texel | mor;
            }
            continue;
        }
        if (!gouraud) {
            /* flat modulation: the colour is the primitive's, so the three
             * channel products were tabled once per triangle */
            const u16 m = lut[texel & 0x1F] | lut[32 + ((texel >> 5) & 0x1F)] |
                          lut[64 + ((texel >> 10) & 0x1F)];
            if (semi && (texel & 0x8000)) {
                *dst = blend5551(*dst, m & 0x1F, (m >> 5) & 0x1F, (m >> 10) & 0x1F, abr) |
                       0x8000 | mor;
            } else {
                *dst = (u16)(m | (texel & 0x8000)) | mor;
            }
            continue;
        }
        /* gouraud: (tex5 * col8) >> 7, clamped to 31 only when some colour on
         * this span exceeds the neutral 0x80 (the dispatcher checked) */
        int r = ((texel & 0x1F) * r8) >> 7;
        int g = (((texel >> 5) & 0x1F) * g8) >> 7;
        int b = (((texel >> 10) & 0x1F) * b8) >> 7;
        if (clamp) {
            if (r > 31) r = 31;
            if (g > 31) g = 31;
            if (b > 31) b = 31;
        }
        if (semi && (texel & 0x8000)) {
            *dst = blend5551(*dst, r, g, b, abr) | 0x8000 | mor;
        } else {
            *dst = (u16)(r | (g << 5) | (b << 10) | (texel & 0x8000)) | mor;
        }
    }
}

#define TEX_SPAN_CASE(bpp, g, s, m, c)                                         \
    case ((c) << 5) | ((bpp) << 3) | ((g) << 2) | ((s) << 1) | (m):            \
        tex_span_body(t, sp, bpp, g, s, m, c);                                 \
        break;
#define TEX_SPAN_CASES_BPP(bpp)                                                \
    TEX_SPAN_CASE(bpp, 0, 0, 0, 0) TEX_SPAN_CASE(bpp, 0, 0, 1, 0)              \
    TEX_SPAN_CASE(bpp, 0, 1, 0, 0) TEX_SPAN_CASE(bpp, 0, 1, 1, 0)              \
    TEX_SPAN_CASE(bpp, 1, 0, 1, 0) TEX_SPAN_CASE(bpp, 1, 1, 1, 0)              \
    TEX_SPAN_CASE(bpp, 1, 0, 1, 1) TEX_SPAN_CASE(bpp, 1, 1, 1, 1)

SOFT_RASTER_IRAM static void tex_span_dispatch(const Tex* t, const TexSpan* sp,
                                               int gouraud, int semi, int modulate) {
    int bpp = t->bpp > 2 ? 2 : t->bpp;
    int clamp = 0;
    if (gouraud) {
        /* the colour is linear along the span: it exceeds neutral somewhere
         * only if it does at an end */
        const int32_t n1 = sp->n - 1;
        const int32_t lim = 0x81 << 16;
        modulate = 1;
        if (sp->ri >= lim || sp->gi >= lim || sp->bi >= lim ||
            sp->ri + sp->rdx * n1 >= lim || sp->gi + sp->gdx * n1 >= lim ||
            sp->bi + sp->bdx * n1 >= lim) {
            clamp = 1;
        }
    }
    const int idx = (clamp << 5) | (bpp << 3) | ((gouraud ? 1 : 0) << 2) |
                    ((semi ? 1 : 0) << 1) | (modulate ? 1 : 0);
#ifdef SOFT_RASTER_PROF_SPANS
    const uint32_t c0 = esp_cpu_get_cycle_count();
#endif
    switch (idx) {
        TEX_SPAN_CASES_BPP(0)
        TEX_SPAN_CASES_BPP(1)
        TEX_SPAN_CASES_BPP(2)
    default:
        break;
    }
#ifdef SOFT_RASTER_PROF_SPANS
    PROFC.case_cy[idx] += esp_cpu_get_cycle_count() - c0;
    PROFC.case_px[idx] += (unsigned)sp->n;
    PROFC.case_rows[idx]++;
#endif
}

/* The span's [xl, xr] per row come from the three edge functions. Each edge
 * bound moves by a fixed rational amount per row, so instead of a hardware
 * divide per edge per row (three per row, on spans averaging 15 pixels) the
 * bound is stepped with an integer DDA: quotient and remainder of the same
 * division, advanced exactly. Output is bit-identical to the divide form. */
typedef struct {
    int q, r, dq, dr, A, kind; /* kind 0: left bound, 1: right bound, 2: horizontal edge */
    int Ao, B, w0, k;          /* for the divide-per-row reference path */
} EdgeDDA;
int psyz_edge_dda = 1;   /* 0: the divide-per-row form (reference) */
int psyz_grad_float = 1; /* 0: exact reciprocal gradients (reference) */
/* (num << 16 +- area/2) / area, rounded half away from zero, exactly as the
 * integer form -- but from a float estimate: the FPU gives a quotient within
 * one of the truth, and one 64-bit multiply-and-compare corrects it. The
 * game's texture mappings land texel coordinates exactly on integers at
 * pixel centres, so a quotient one unit short picks the neighbouring texel
 * row for a whole triangle (the opening's dark water wedges). Numerators
 * the float path cannot hold take the division itself. */
static inline long long div_area_16_fast(long long num, int area, float inv_area_f) {
    const int neg = num < 0;
    const unsigned long long mag = (unsigned long long)(neg ? -num : num);
    const unsigned long long m = (mag << 16) + (unsigned long long)area / 2;
    unsigned long long q;
    if (mag < (1ull << 31)) {
        const float gf = (float)(uint32_t)mag * inv_area_f;
        if (gf < 2147483648.0f) {
            long long r;
            q = (unsigned long long)(uint32_t)__builtin_lrintf(gf);
            r = (long long)(m - q * (unsigned long long)area);
            while (r < 0) {
                q--;
                r += area;
            }
            while (r >= area) {
                q++;
                r -= area;
            }
            return neg ? -(long long)q : (long long)q;
        }
    }
    q = m / (unsigned long long)area;
    return neg ? -(long long)q : (long long)q;
}
int psyz_grad_check;     /* >0: compare float and exact per triangle, print this many */
static inline int floordiv_i(int n, int d) {
    return n >= 0 ? n / d : -((-n + d - 1) / d);
}
static inline void edge_dda_init(EdgeDDA* e, int A, int B, int w) {
    e->Ao = A;
    e->B = B;
    e->w0 = w;
    e->k = 0;
    if (A == 0) {
        e->kind = 2; /* inside iff w >= 0; w moves by B per row */
        e->q = w;
        e->dq = B;
        e->r = e->dr = e->A = 0;
    } else if (A > 0) {
        /* x >= minx + ceil(-w / A) = minx + floor((-w + A - 1) / A) */
        const int n = -w + A - 1;
        e->kind = 0;
        e->A = A;
        e->q = floordiv_i(n, A);
        e->r = n - e->q * A;
        e->dq = floordiv_i(-B, A);
        e->dr = -B - e->dq * A;
    } else {
        /* x <= minx + floor(w / -A); a negative quotient means the row is empty */
        const int Ap = -A;
        e->kind = 1;
        e->A = Ap;
        e->q = floordiv_i(w, Ap);
        e->r = w - e->q * Ap;
        e->dq = floordiv_i(B, Ap);
        e->dr = B - e->dq * Ap;
    }
}
/* the row's span, then advance all three edges to the next row; 0 if empty */
static inline int edge_row(EdgeDDA* ed, int minx, int maxx, int* pxl, int* pxr) {
    int xl = minx, xr = maxx, ok = 1;
    if (!psyz_edge_dda) {
        /* reference: the edge value at this row, one divide per edge */
        for (int i = 0; i < 3; i++) {
            EdgeDDA* e = &ed[i];
            const int w = e->w0 + e->k * e->B, A = e->Ao;
            e->k++;
            if (A == 0) {
                if (w < 0) ok = 0;
            } else if (A > 0) {
                if (w < 0) {
                    const int x = minx + (-w + A - 1) / A;
                    if (x > xl) xl = x;
                }
            } else {
                if (w < 0) ok = 0;
                const int x = minx + w / (-A);
                if (x < xr) xr = x;
            }
        }
        *pxl = xl;
        *pxr = xr;
        return ok && xl <= xr;
    }
    for (int i = 0; i < 3; i++) {
        EdgeDDA* e = &ed[i];
        if (e->kind == 2) {
            if (e->q < 0) ok = 0;
            e->q += e->dq;
            continue;
        }
        const int x = minx + e->q;
        if (e->kind == 0) {
            if (x > xl) xl = x;
        } else {
            if (x < xr) xr = x;
        }
        e->q += e->dq;
        e->r += e->dr;
        if (e->r >= e->A) {
            e->r -= e->A;
            e->q++;
        }
    }
    *pxl = xl;
    *pxr = xr;
    return ok && xl <= xr;
}

#ifdef ESP_PLATFORM
/* Startup micro-benchmark of the textured span loop: the same 4bpp modulated
 * span over texture pages in PSRAM (one page, warm in the cache; 16 pages
 * cycling, 512 KB footprint, cold) and in internal RAM, walking the texture
 * along u (one cache line per 128 texels), along v (a new 2 KB row per
 * texel) and diagonally; destination rows in PSRAM, cold or warm. Separates
 * memory stalls from arithmetic without a cycle-accurate profiler. Reads
 * whatever VRAM holds (the CLUT has no zero entry so every texel is drawn);
 * writes rows 256.. of VRAM, so it runs once, before the game loads anything. */
static u16 bench_clut[16] __attribute__((aligned(64)));
static unsigned bench_run(const Tex* pages, int npages, u16* dst_base, int dst_cold,
                          int32_t udx, int32_t vdx, int vmask, int gouraud, int modulate) {
    TexSpan sp;
    const int n = 32, spans = 2048;
    static u16 bench_lut[96];
    for (int i = 0; i < 32; i++) {
        bench_lut[i] = (u16)mod5(i, 0x90);
        bench_lut[32 + i] = (u16)(mod5(i, 0x90) << 5);
        bench_lut[64 + i] = (u16)(mod5(i, 0x90) << 10);
    }
    memset(&sp, 0, sizeof sp);
    sp.lut = bench_lut;
    sp.udx = udx;
    sp.vdx = vdx;
    sp.n = n;
    sp.r8 = sp.g8 = sp.b8 = modulate ? 0x90 : 0x80;
    sp.ri = sp.gi = sp.bi = 0x90 << 16;
    sp.rdx = sp.gdx = sp.bdx = -(1 << 12);
    const uint32_t t0 = esp_cpu_get_cycle_count();
    for (int s = 0; s < spans; s++) {
        sp.dst = dst_cold ? dst_base + (s & 255) * VRAM_W + ((s >> 8) & 7) * 64 : dst_base;
        sp.ui = ((s * 37) & 0xFF) << 16;
        sp.vi = ((s * 11) & vmask) << 16;
        tex_span_dispatch(&pages[s % npages], &sp, gouraud, 0, modulate);
    }
    return (esp_cpu_get_cycle_count() - t0) / (unsigned)(n * spans);
}
void Draw_Benchmark(void) {
    static int done;
    if (done || !vram) {
        return;
    }
    done = 1;
    for (int i = 0; i < 16; i++) {
        bench_clut[i] = (u16)(0x4210 + i * 0x421);
    }
    Tex pages[16];
    for (int i = 0; i < 16; i++) {
        pages[i].page = vram + (i & 15) * 64;
        pages[i].clut = bench_clut;
        pages[i].bpp = 0;
        pages[i].abr = 0;
        pages[i].tbase = ((i & 15) * 64 >> 3) << 6;
        pages[i].px = (i & 15) * 64;
        pages[i].py = 0;
        pages[i].vshift = 10;
        pages[i].umask = pages[i].vmask = 0xFF;
        pages[i].uor = pages[i].vor = 0;
    }
    u16* dst = vram + 256 * VRAM_W;
    const int32_t one = 1 << 16;
    printf("[bench] 4bpp modulated span, cycles/px: psram page warm: u %u v %u diag %u\n",
           bench_run(pages, 1, dst, 0, one, 0, 0xFF, 0, 1),
           bench_run(pages, 1, dst, 0, 0, one, 0xFF, 0, 1),
           bench_run(pages, 1, dst, 0, one, one, 0xFF, 0, 1));
    printf("[bench] psram 16 pages cold, dst cold: u %u v %u diag %u | plain u %u gouraud u %u\n",
           bench_run(pages, 16, dst, 1, one, 0, 0xFF, 0, 1),
           bench_run(pages, 16, dst, 1, 0, one, 0xFF, 0, 1),
           bench_run(pages, 16, dst, 1, one, one, 0xFF, 0, 1),
           bench_run(pages, 16, dst, 1, one, 0, 0xFF, 0, 0),
           bench_run(pages, 16, dst, 1, one, 0, 0xFF, 1, 1));
    printf("[bench] psram 16 pages cold, dst warm: u %u v %u | 1 page warm, dst cold: u %u v %u\n",
           bench_run(pages, 16, dst, 0, one, 0, 0xFF, 0, 1),
           bench_run(pages, 16, dst, 0, 0, one, 0xFF, 0, 1),
           bench_run(pages, 1, dst, 1, one, 0, 0xFF, 0, 1),
           bench_run(pages, 1, dst, 1, 0, one, 0xFF, 0, 1));
}
#endif
#undef TEX_SPAN_CASE
#undef TEX_SPAN_CASES_BPP

SOFT_RASTER_IRAM static void raster_tri(const SV* a, const SV* b, const SV* c, int textured,
                       u16 tpage, u16 clut, int semi, int gouraud) {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
    const uint32_t prof_setup_t0 = esp_cpu_get_cycle_count();
#endif
    int ax = a->x + st.ox, ay = a->y + st.oy;
    int bx = b->x + st.ox, by = b->y + st.oy;
    int cx = c->x + st.ox, cy = c->y + st.oy;

    int area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (area == 0) {
        return;
    }
    if (area < 0) { // enforce CCW so the edge functions face inward
        const SV* t = b;
        b = c;
        c = t;
        int ti = bx; bx = cx; cx = ti;
        ti = by; by = cy; cy = ti;
        area = -area;
    }

#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
    {
        /* one-shot: where is the rasterizer actually writing, and what is it
         * clipping to? Compare against the framebuffer the scanout reads. */
        extern unsigned short *g_RawVram;
        static int shown = 3;
        if (shown > 0) {
            shown--;
            printf("[rast] vram %p g_RawVram %p ofs %d,%d clip %d,%d..%d,%d "
                   "tri %d,%d %d,%d %d,%d\n",
                   (void*)vram, (void*)g_RawVram, st.ox, st.oy,
                   st.x0, st.y0, st.x1, st.y1, ax, ay, bx, by, cx, cy);
        }
    }
#endif
    int minx = ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx);
    int maxx = ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx);
    int miny = ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy);
    int maxy = ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy);
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
    {
        /* classify: is the world missing because it lands outside the screen,
         * or because it projects to sub-pixel size? */
        if (maxx < st.x0 || minx > st.x1 || maxy < st.y0 || miny > st.y1) {
            psyz_tri_off++;
        } else if ((maxx - minx) < 2 && (maxy - miny) < 2) {
            psyz_tri_tiny++;
        } else {
            static int shown = 8;
            psyz_tri_ok++;
            if (shown > 0) {
                shown--;
                printf("[ok] tri (%d,%d) (%d,%d) (%d,%d) area %d tex %d\n",
                       ax, ay, bx, by, cx, cy, area, textured);
            }
        }
    }
#endif
    /* The PSX GPU refuses any polygon wider than 1023 or taller than 511, and
     * that refusal is load-bearing rather than a curiosity. When a polygon
     * crosses behind the camera its projection is meaningless and the GTE pins
     * the offending vertices at its +/-1024 clamp, producing a quad that
     * stretches from one clamp corner across the whole screen. The console
     * simply drops those. Without the rule a software rasterizer draws them at
     * full size: on this board a searchlight cone came out as a moving white
     * sheet over the dock, with the HUD and the credit text still legible on
     * top of it because they are drawn afterwards -- three full screens of
     * texels per frame and 142 ms to raster.
     *
     * Measured on the RAW extents, before clipping to the drawing area: the
     * clip would shrink every one of them to screen size and hide exactly the
     * property being tested. */
    if ((maxx - minx) > 1023 || (maxy - miny) > 511) {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
        psyz_tri_huge++;
#endif
        return;
    }

    if (minx < st.x0) minx = st.x0;
    if (maxx > st.x1) maxx = st.x1;
    /* the row a single worker starts its attribute stepping on (clipped to
     * the draw area, not to this worker's band): the band's first row steps
     * from there, so two workers' pixels are those of one */
    int step_y0 = miny < st.area_y0 ? st.area_y0 : miny;
    if (miny < st.y0) miny = st.y0;
    if (maxy > st.y1) maxy = st.y1;
    if (minx > maxx || miny > maxy) {
        return;
    }
    if (step_y0 > miny) step_y0 = miny;
    tex_mark_dirty(minx, miny, maxx, maxy);

    // edge setup: E(x,y) >= 0 inside, top-left edges get the <0 bias on the
    // others so shared edges paint exactly once
    int A0 = by - cy, B0 = cx - bx, C0 = bx * cy - by * cx;
    int A1 = cy - ay, B1 = ax - cx, C1 = cx * ay - cy * ax;
    int A2 = ay - by, B2 = bx - ax, C2 = ax * by - ay * bx;
    int bias0 = (A0 > 0 || (A0 == 0 && B0 > 0)) ? 0 : -1;
    int bias1 = (A1 > 0 || (A1 == 0 && B1 > 0)) ? 0 : -1;
    int bias2 = (A2 > 0 || (A2 == 0 && B2 > 0)) ? 0 : -1;

    Tex t;
    int tex_mode = 0;
    if (textured) {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
        const uint32_t tf0 = esp_cpu_get_cycle_count();
#endif
        t = tex_from(tpage, clut);
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
        psyz_prof_texfrom_cy += esp_cpu_get_cycle_count() - tf0;
#endif
        tex_mode = (!gouraud && a->r == 0x80 && a->g == 0x80 && a->b == 0x80) ? 0 : 1;
    }

    int w0row = A0 * minx + B0 * miny + C0 + bias0;
    int w1row = A1 * minx + B1 * miny + C1 + bias1;
    int w2row = A2 * minx + B2 * miny + C2 + bias2;
    EdgeDDA edges[3];
    edge_dda_init(&edges[0], A0, B0, w0row);
    edge_dda_init(&edges[1], A1, B1, w1row);
    edge_dda_init(&edges[2], A2, B2, w2row);
    u16 mod_lut[96];
    if (textured && tex_mode && !gouraud) {
        for (int i = 0; i < 32; i++) {
            mod_lut[i] = (u16)mod5(i, a->r);
            mod_lut[32 + i] = (u16)(mod5(i, a->g) << 5);
            mod_lut[64 + i] = (u16)(mod5(i, a->b) << 10);
        }
    }

    // Per-triangle attribute gradients, 16.16 fixed point. Xtensa has no fast
    // divider, and the first-correctness version paid 3-5 integer divisions
    // per PIXEL for barycentric attributes - measured at 100-130 cy/px on the
    // S3, it dominated every gouraud primitive. An affine attribute is a plane
    // over (x,y): compute d/dx and d/dy once with a handful of divisions, then
    // step. The +area/2 rounds the gradient to nearest, which keeps the last
    // scanline of a 255-wide gradient from drifting versus the exact form.
#if defined(SOFT_RASTER_FLOAT_GRAD) && !defined(SOFT_RASTER_RECIP)
#error "SOFT_RASTER_FLOAT_GRAD needs SOFT_RASTER_RECIP: the exact path stays as the reference"
#endif
#if defined(SOFT_RASTER_RECIP)
    // RV32 has no 64-bit divide: each of the fifteen (s64)/area below is a
    // library call of a few hundred cycles, which per triangle outweighed the
    // texels. One 64-bit divide gives 2^32/area; every quotient is then a
    // 32x32->64 multiply for a first guess and one multiply-compare that
    // corrects it to the exact rounded result the division gave (the guess
    // is within one of it). Shapes outside the range the guess is valid for
    // (a numerator beyond 2^31, or area 1) take the division itself.
    typedef long long s64;
    typedef unsigned long long u64;
#if defined(SOFT_RASTER_FLOAT_GRAD)
    const u64 inv_area = (psyz_grad_float && !psyz_grad_check) ? 0 : ((u64)1 << 32) / (u64)area;
#else
    const u64 inv_area = ((u64)1 << 32) / (u64)area;
#endif
    s64 dx_, dy_;
    s64 div_area_16(s64 num) /* (num << 16 +- area/2) / area, like the exact form */
    {
        const int neg = num < 0;
        u64 mag = (u64)(neg ? -num : num);
        u64 q;
        if (mag >= (1ull << 31) || area < 2) {
            q = ((mag << 16) + (u64)area / 2) / (u64)area;
        } else {
            const u64 m = (mag << 16) + (u64)area / 2;
            q = (mag * inv_area) >> 16; /* <= exact */
            s64 rem = (s64)(m - q * (u64)area);
            while (rem >= area) { q++; rem -= area; }
            while (rem < 0) { q--; rem += area; }
        }
        return neg ? -(s64)q : (s64)q;
    }
#define DIV_AREA_16(num) div_area_16((s64)(num))
#if defined(SOFT_RASTER_FLOAT_GRAD)
    // Single-precision FPU: gradient = numerator * (65536 / area), rounded
    // to nearest. The numerators are under 2^21 and the float mantissa has
    // 24 bits, so the result is within one 16.16 unit of the exact form --
    // for ~1200 cycles less per triangle than the reciprocal fix-ups.
    // Runtime-selectable so the replay benchmark can compare the two.
    const float inv_area_f = 65536.0f / (float)area;
#define GRAD(p0, p1, p2)                                                       \
    if (psyz_grad_float) {                                                     \
        dx_ = div_area_16_fast((A0) * (p0) + (A1) * (p1) + (A2) * (p2), area, inv_area_f); \
        dy_ = div_area_16_fast((B0) * (p0) + (B1) * (p1) + (B2) * (p2), area, inv_area_f); \
    } else {                                                                   \
        dx_ = DIV_AREA_16((s64)(A0) * (p0) + (s64)(A1) * (p1) + (s64)(A2) * (p2)); \
        dy_ = DIV_AREA_16((s64)(B0) * (p0) + (s64)(B1) * (p1) + (s64)(B2) * (p2)); \
    }
#else
#define GRAD(p0, p1, p2)                                                       \
    dx_ = DIV_AREA_16((s64)(A0) * (p0) + (s64)(A1) * (p1) + (s64)(A2) * (p2)); \
    dy_ = DIV_AREA_16((s64)(B0) * (p0) + (s64)(B1) * (p1) + (s64)(B2) * (p2));
#endif
#else
#define GRAD(p0, p1, p2)                                                           dx_ = ((s64)(A0) * (p0) + (s64)(A1) * (p1) + (s64)(A2) * (p2)) * 65536;        dy_ = ((s64)(B0) * (p0) + (s64)(B1) * (p1) + (s64)(B2) * (p2)) * 65536;        dx_ = dx_ >= 0 ? (dx_ + area / 2) / area : (dx_ - area / 2) / area;            dy_ = dy_ >= 0 ? (dy_ + area / 2) / area : (dy_ - area / 2) / area;

    typedef long long s64;
    s64 dx_, dy_;
#endif
    int e0r = w0row - bias0, e1r = w1row - bias1, e2r = w2row - bias2;

    // Accumulators are 32-bit 16.16: attributes are 8-bit, spans are under
    // 1024px, so values stay far from overflow - and on a 32-bit Xtensa a
    // 64-bit accumulator costs two registers and multi-instruction adds per
    // pixel, which measured SLOWER than the hardware-divide version it was
    // meant to replace (17.8M vs 13.1M cy on the quad battery).
    typedef int32_t fx;
    fx rrow = 0, grow = 0, brow = 0, urow = 0, vrow = 0;
    fx rdx = 0, rdy = 0, gdx = 0, gdy = 0, bdx = 0, bdy = 0;
    fx udx = 0, udy = 0, vdx = 0, vdy = 0;
#if defined(SOFT_RASTER_FLOAT_GRAD)
/* the three terms are large and cancel (the edge values at the bounding-box
 * corner can be ~2^22 while their weighted sum is small): summed exactly in
 * 64 bits, then one float multiply. In float they lost up to 64 units each,
 * which on long thin triangles (the opening's water) showed as texture
 * wobble. */
#define ROW0(p0, p1, p2)                                                       \
    (psyz_grad_float                                                           \
         ? (fx)div_area_16_fast((s64)e0r * (p0) + (s64)e1r * (p1) + (s64)e2r * (p2), area, inv_area_f) \
         : (fx)DIV_AREA_16((s64)e0r * (p0) + (s64)e1r * (p1) + (s64)e2r * (p2)))
#elif defined(SOFT_RASTER_RECIP)
#define ROW0(p0, p1, p2) ((fx)DIV_AREA_16((s64)e0r * (p0) + (s64)e1r * (p1) + (s64)e2r * (p2)))
#else
#define ROW0(p0, p1, p2) ((fx)(((s64)(e0r * (p0) + e1r * (p1) + e2r * (p2)) << 16) / area))
#endif
    const int band_k = miny - step_y0; /* rows from step_y0 to this band */
    e0r -= B0 * band_k; e1r -= B1 * band_k; e2r -= B2 * band_k;
    if (gouraud) {
        GRAD(a->r, b->r, c->r) rdx = (fx)dx_; rdy = (fx)dy_;
        GRAD(a->g, b->g, c->g) gdx = (fx)dx_; gdy = (fx)dy_;
        GRAD(a->b, b->b, c->b) bdx = (fx)dx_; bdy = (fx)dy_;
        rrow = ROW0(a->r, b->r, c->r) + rdy * band_k;
        grow = ROW0(a->g, b->g, c->g) + gdy * band_k;
        brow = ROW0(a->b, b->b, c->b) + bdy * band_k;
    }
    if (textured) {
        GRAD(a->u, b->u, c->u) udx = (fx)dx_; udy = (fx)dy_;
        GRAD(a->v, b->v, c->v) vdx = (fx)dx_; vdy = (fx)dy_;
        urow = ROW0(a->u, b->u, c->u) + udy * band_k;
        vrow = ROW0(a->v, b->v, c->v) + vdy * band_k;
#ifdef SOFT_MIP_TEX
        {
            /* texels per pixel along x and y, from the larger axis step */
            fx s = udx < 0 ? -udx : udx;
            fx s2 = vdx < 0 ? -vdx : vdx;
            if (s2 > s) s = s2;
            s2 = udy < 0 ? -udy : udy;
            if (s2 > s) s = s2;
            s2 = vdy < 0 ? -vdy : vdy;
            if (s2 > s) s = s2;
            const int level = s >= (8 << 16) ? 3 : s >= (4 << 16) ? 2 : s >= (2 << 16) ? 1 : 0;
#ifdef ESP_PLATFORM
            PROFC.mip_tris[level]++;
#endif
            if (level && t.bpp <= 1 && psyz_mip_enable) {
                /* palette textures only: a 15-bit page is a frame-buffer
                 * effect or a movie frame, sampled live */
                tex_mip(&t, level);
                if (t.vshift != 10) { /* the level is available */
                    udx >>= level; udy >>= level; vdx >>= level; vdy >>= level;
                    urow >>= level; vrow >>= level;
                }
            }
        }
#endif
    }
#if defined(SOFT_RASTER_FLOAT_GRAD) && defined(SOFT_RASTER_RECIP)
    if (psyz_grad_check > 0 && psyz_grad_float) {
        /* the exact values for the same triangle, printed when they differ
         * from what the float path produced by more than 1/16 of a level */
#define CHK(name, p0, p1, p2, fdx, fdy, frow)                                      \
        {                                                                          \
            const s64 ex = DIV_AREA_16((s64)(A0) * (p0) + (s64)(A1) * (p1) + (s64)(A2) * (p2)); \
            const s64 ey = DIV_AREA_16((s64)(B0) * (p0) + (s64)(B1) * (p1) + (s64)(B2) * (p2)); \
            const s64 er = DIV_AREA_16((s64)e0r * (p0) + (s64)e1r * (p1) + (s64)e2r * (p2)); \
            const s64 dd = (ex - (fdx)) * (maxx - minx) + (ey - (fdy)) * (maxy - miny) + (er - (frow)); \
            if ((dd > 4096 || dd < -4096) && psyz_grad_check > 0) {                \
                psyz_grad_check--;                                                 \
                printf("[gradchk] %c tri (%d,%d) (%d,%d) (%d,%d) area %d bbox %d,%d..%d,%d p %d %d %d " \
                       "exact dx %lld dy %lld r0 %lld | float dx %d dy %d r0 %d\n",      \
                       name, ax, ay, bx, by, cx, cy, area, minx, miny, maxx, maxy, (int)(p0), \
                       (int)(p1), (int)(p2), ex, ey, er, (int)(fdx), (int)(fdy), (int)(frow)); \
            }                                                                      \
        }
        if (gouraud) {
            CHK('r', a->r, b->r, c->r, rdx, rdy, rrow)
            CHK('g', a->g, b->g, c->g, gdx, gdy, grow)
            CHK('b', a->b, b->b, c->b, bdx, bdy, brow)
        }
        if (textured) {
            CHK('u', a->u, b->u, c->u, udx, udy, urow)
            CHK('v', a->v, b->v, c->v, vdx, vdy, vrow)
        }
#undef CHK
    }
#endif
    e0r += B0 * band_k; e1r += B1 * band_k; e2r += B2 * band_k;
#undef GRAD
#undef ROW0
#if defined(SOFT_RASTER_RECIP)
#undef DIV_AREA_16
#endif
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
    /* slot 5: the per-triangle setup, from entry to here */
    psyz_prof_cy[5] += esp_cpu_get_cycle_count() - prof_setup_t0;
    psyz_prof_n[5]++;
#endif

    // Fast path for the dominant WRP case: opaque untextured spans (the big
    // gouraud background quads). Span bounds come from the edge functions
    // directly (three divides per row on the LX7's hardware divider), the
    // inner loop steps only RGB and writes pixel pairs with 32-bit stores.
    // Attribute stepping is identical to the generic loop, so pixel output
    // is bit-exact with it.
    if (!textured && !semi && !MASK_ACTIVE()) {
        fx r0 = gouraud ? rrow : (fx)(a->r << 16);
        fx g0 = gouraud ? grow : (fx)(a->g << 16);
        fx b0 = gouraud ? brow : (fx)(a->b << 16);
        fx frdx = gouraud ? rdx : 0, frdy = gouraud ? rdy : 0;
        fx fgdx = gouraud ? gdx : 0, fgdy = gouraud ? gdy : 0;
        fx fbdx = gouraud ? bdx : 0, fbdy = gouraud ? bdy : 0;
        for (int y = miny; y <= maxy; y++, r0 += frdy, g0 += fgdy, b0 += fbdy) {
            int xl, xr;
            if (!edge_row(edges, minx, maxx, &xl, &xr)) {
                continue;
            }
            fx ri = r0 + (fx)(xl - minx) * frdx;
            fx gi = g0 + (fx)(xl - minx) * fgdx;
            fx bi = b0 + (fx)(xl - minx) * fbdx;
            u16* p = vram + y * VRAM_W + xl;
            int n = xr - xl + 1;
#define SPAN_PIX()                                                             \
    (u16)((((unsigned)ri >> 19) & 0x1F) | ((((unsigned)gi >> 19) & 0x1F) << 5) \
          | ((((unsigned)bi >> 19) & 0x1F) << 10))
            if (((uintptr_t)p & 2) && n) {
                *p++ = SPAN_PIX();
                ri += frdx; gi += fgdx; bi += fbdx;
                n--;
            }
            u32* p2 = (u32*)p;
            while (n >= 2) {
                u16 lo = SPAN_PIX();
                ri += frdx; gi += fgdx; bi += fbdx;
                u16 hi = SPAN_PIX();
                ri += frdx; gi += fgdx; bi += fbdx;
                *p2++ = (u32)lo | ((u32)hi << 16);
                n -= 2;
            }
            if (n) {
                *(u16*)p2 = SPAN_PIX();
            }
#undef SPAN_PIX
        }
        return;
    }

    // Generic loop, span-based: per row the edge functions give [xl, xr]
    // directly (three hardware divides), so the inner loop carries no edge
    // accumulators and no accept test - just attribute stepping + the pixel
    // write. Identical stepping start (xl offset from minx) keeps output
    // bit-exact with the per-pixel-test form.
    for (int y = miny; y <= maxy; y++, rrow += rdy, grow += gdy, brow += bdy,
             urow += udy, vrow += vdy) {
        int xl, xr;
        if (!edge_row(edges, minx, maxx, &xl, &xr)) {
            continue;
        }
        fx ri = rrow + (fx)(xl - minx) * rdx;
        fx gi = grow + (fx)(xl - minx) * gdx;
        fx bi = brow + (fx)(xl - minx) * bdx;
        fx ui = urow + (fx)(xl - minx) * udx;
        fx vi = vrow + (fx)(xl - minx) * vdx;
        u16* row = vram + y * VRAM_W;
        if (textured) {
            // one specialised loop per (bpp, shading, blending) combination:
            // every test the generic loop made per pixel is made once here
            TexSpan sp = {row + xl, xr - xl + 1, ui, vi, udx, vdx,
                          ri, gi, bi, rdx, gdx, bdx, a->r, a->g, a->b, mod_lut};
#ifdef SOFT_RASTER_PROF_SPANS
            const uint32_t sp0 = esp_cpu_get_cycle_count();
            tex_span_dispatch(&t, &sp, gouraud, semi, tex_mode);
            psyz_prof_span_cy += esp_cpu_get_cycle_count() - sp0;
            psyz_prof_rows++;
            psyz_prof_px += (unsigned)sp.n;
#else
            tex_span_dispatch(&t, &sp, gouraud, semi, tex_mode);
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            psyz_prof_rows++;
            psyz_prof_px += (unsigned)sp.n;
#endif
#endif
            continue;
        }
        for (int x = xl; x <= xr; x++, ri += rdx, gi += gdx, bi += bdx,
                 ui += udx, vi += vdx) {
            int r8 = a->r, g8 = a->g, b8 = a->b;
            if (gouraud) {
                r8 = (int)(ri >> 16);
                g8 = (int)(gi >> 16);
                b8 = (int)(bi >> 16);
            }
            if (textured) {
                put_texel(&row[x],
                          tex_fetch(&t, (int)(ui >> 16), (int)(vi >> 16)), r8,
                          g8, b8, semi, t.abr);
            } else {
                put_flat(&row[x], r8, g8, b8, semi, (cur_tpage >> 5) & 3);
            }
        }
    }
}

// Gouraud line, Bresenham over the major axis.
SOFT_RASTER_IRAM static void raster_line(const SV* a, const SV* b, int semi) {
    int x0 = a->x + st.ox, y0 = a->y + st.oy;
    int x1 = b->x + st.ox, y1 = b->y + st.oy;
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int steps = (dx > -dy ? dx : -dy);
    if (steps == 0) {
        steps = 1;
    }
    int abr = (cur_tpage >> 5) & 3;
    tex_mark_dirty(x0 < x1 ? x0 : x1, y0 < y1 ? y0 : y1, x0 > x1 ? x0 : x1, y0 > y1 ? y0 : y1);
    for (int i = 0;; i++) {
        if (x0 >= st.x0 && x0 <= st.x1 && y0 >= st.y0 && y0 <= st.y1) {
            int r = a->r + (b->r - a->r) * i / steps;
            int g = a->g + (b->g - a->g) * i / steps;
            int bl = a->b + (b->b - a->b) * i / steps;
            put_flat(&vram[y0 * VRAM_W + x0], r, g, bl, semi, abr);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// ---- VRAM block operations -------------------------------------------------

// A corrupt packet must degrade to a bounded no-op, not a multi-second (or
// unbounded) VRAM stomp - clamp every block op to physical VRAM limits.
static inline int vram_clamp_w(const PS1_RECT* rect, const char* op) {
    int w = rect->w;
    if (w < 0 || w > VRAM_W) {
        WARNF("%s rect suspicious: %d,%d %dx%d", op, rect->x, rect->y,
              rect->w, rect->h);
        w = w < 0 ? 0 : VRAM_W;
    }
    return w;
}
static inline int vram_clamp_h(const PS1_RECT* rect) {
    int h = rect->h;
    return h < 0 ? 0 : (h > VRAM_H ? VRAM_H : h);
}

static void soft_clear_image(const PS1_RECT* rect, u8 r, u8 g, u8 b) {
    clut_cache_flush();
    u16 c = (u16)((r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10));
    int w = vram_clamp_w(rect, "clear"), h = vram_clamp_h(rect);
    tex_mark_dirty(rect->x, rect->y, rect->x + w - 1, rect->y + h - 1);
    for (int y = 0; y < h; y++) {
        int yy = (rect->y + y) & (VRAM_H - 1);
        if (yy < st.band_y0 || yy > st.band_y1) {
            continue; /* the other worker's rows */
        }
        u16* row = vram + yy * VRAM_W;
        if (rect->x + w <= VRAM_W) {
            /* no wrap (the usual frame clear): words, not masked halfwords */
            u16* p = row + rect->x;
            int n = w;
            if (((uintptr_t)p & 2) && n) {
                *p++ = c;
                n--;
            }
            u32* q = (u32*)p;
            const u32 cc = (u32)c | ((u32)c << 16);
            for (; n >= 8; n -= 8, q += 4) {
                q[0] = cc; q[1] = cc; q[2] = cc; q[3] = cc;
            }
            for (; n >= 2; n -= 2) *q++ = cc;
            if (n) *(u16*)q = c;
            continue;
        }
        for (int x = 0; x < w; x++) {
            row[(rect->x + x) & (VRAM_W - 1)] = c;
        }
    }
}

static void soft_load_image(const PS1_RECT* rect, const u16* src) {
    if (!src) {
        // The PSX's DMA read open bus at address 0 and uploaded garbage
        // without complaint -- MGS's menu does exactly this on purpose
        // (menu_restore_nouse runs before menu_init_nouse fills the panel
        // in). A host dereference of NULL is fatal, so skip the upload; the
        // panel is overwritten with real pixels moments later either way.
        WARNF("load image from NULL (%d,%d %dx%d) skipped", rect->x, rect->y,
              rect->w, rect->h);
        return;
    }
    clut_cache_flush();
    int w = vram_clamp_w(rect, "load"), h = vram_clamp_h(rect);
    tex_mark_dirty(rect->x, rect->y, rect->x + w - 1, rect->y + h - 1);
    for (int y = 0; y < h; y++) {
        u16* dst = vram + ((rect->y + y) & (VRAM_H - 1)) * VRAM_W + rect->x;
        memcpy(dst, src, (size_t)rect->w * 2);
        src += rect->w;
    }
}

static void soft_store_image(const PS1_RECT* rect, u16* dst) {
    int w = vram_clamp_w(rect, "store"), h = vram_clamp_h(rect);
    (void)w;
    { static int store_log = 8; if (store_log > 0) { store_log--;
    WARNF("store %d,%d %dx%d -> %p", rect->x, rect->y, rect->w, rect->h,
          (void*)dst); } }
    for (int y = 0; y < h; y++) {
        const u16* src =
            vram + ((rect->y + y) & (VRAM_H - 1)) * VRAM_W + rect->x;
        memcpy(dst, src, (size_t)rect->w * 2);
        dst += rect->w;
    }
}

static void soft_move_image(const PS1_RECT* rect, int dx, int dy) {
    clut_cache_flush();
    int w = vram_clamp_w(rect, "move"), h = vram_clamp_h(rect);
    // clip so both source and destination stay inside VRAM
    int maxx = rect->x > dx ? rect->x : dx;
    int maxy = rect->y > dy ? rect->y : dy;
    if (w > VRAM_W - maxx) w = VRAM_W - maxx;
    if (h > VRAM_H - maxy) h = VRAM_H - maxy;
    if (w <= 0 || h <= 0 || rect->x < 0 || rect->y < 0 || dx < 0 || dy < 0) {
        return;
    }
    tex_mark_dirty(dx, dy, dx + w - 1, dy + h - 1);
    // memmove per row, walking in the safe direction for overlap
    if (dy <= rect->y) {
        for (int y = 0; y < h; y++) {
            memmove(vram + (dy + y) * VRAM_W + dx,
                    vram + (rect->y + y) * VRAM_W + rect->x, (size_t)w * 2);
        }
    } else {
        for (int y = h - 1; y >= 0; y--) {
            memmove(vram + (dy + y) * VRAM_W + dx,
                    vram + (rect->y + y) * VRAM_W + rect->x, (size_t)w * 2);
        }
    }
}



// ---- packet parsing into core calls ---------------------------------------
// Mirrors sdl3_gl.c's Draw_PushPrim contract exactly: same s11 decoding, same
// word accounting, same quad split (0,1,2)+(1,3,2).

typedef struct { int x, y; } SoftPos;
static SoftPos display_area_px = {0, 0};
static SoftPos display_size_px = {256, 240};
static int display_enabled = 1;

#ifdef ESP_PLATFORM
/* A primitive that samples the frame buffer must see every earlier
 * primitive drawn by BOTH workers: the two rendezvous before it. Decided
 * from the packet's texpage, identically on both workers, before any
 * per-band clipping can make them diverge. */
void Psyz_GpuBarrier(void);
static inline void Psyz_GpuFeedbackCheck(u16 tpage) {
    if (((tpage >> 7) & 3) >= 2 && ((tpage >> 4) & 1) == 0 && (tpage & 0xF) < 10) {
        Psyz_GpuBarrier();
    }
}
#endif
int Draw_PushPrim(u_long* packets, int max_len) {
    int len = max_len;
    int code = (int)(*packets >> 24) & 0xFF;
    bool isPoly = !(code & 0x40);
    bool isLine = (code & 0x40) && !(code & 0x20);
    bool isTile = (code & 0x40) && (code & 0x20);
    bool isTextured = (code & TEXTURED) != 0;
    bool isGouraud = (code & GOURAUD) != 0;
    bool isShadeTex = !((code & 1) && isTextured && !isLine);
    bool isSemi = (code & SEMITRANSP) != 0;
    u16 tpage = 0, clut = 0;
    SV v[4];

    int r0 = 0x80, g0 = 0x80, b0 = 0x80;
    if (isShadeTex) {
        r0 = (u8)(*packets >> 0);
        g0 = (u8)(*packets >> 8);
        b0 = (u8)(*packets >> 16);
    }
    packets++;
    len--;

    if (isPoly) {
        if (!(code & TRIANGLE)) {
            WARNF("code %02X not supported", code);
            return max_len - len;
        }
        int n = (code & EXTRA_VERTEX) ? 4 : 3;
        for (int i = 0; i < n; i++) {
            if (i == 0 || !isGouraud) {
                v[i].r = r0;
                v[i].g = g0;
                v[i].b = b0;
            }
            if (i > 0 && isGouraud && len > 0) {
                v[i].r = ((u8*)packets)[0];
                v[i].g = ((u8*)packets)[1];
                v[i].b = ((u8*)packets)[2];
                packets++;
                len--;
            }
            if (len <= 0) {
                return max_len - len;
            }
            v[i].x = s11(((short*)packets)[0]);
            v[i].y = s11(((short*)packets)[1]);
            packets++;
            len--;
            if (isTextured && len > 0) {
                v[i].u = ((u8*)packets)[0];
                v[i].v = ((u8*)packets)[1];
                if (i == 0) {
                    clut = ((u16*)packets)[1];
                } else if (i == 1) {
                    tpage = ((u16*)packets)[1];
                }
                packets++;
                len--;
            } else {
                v[i].u = v[i].v = 0;
            }
        }
        if (!isTextured) {
            tpage = cur_tpage;
        }
        if (!isShadeTex) {
            // raw-texture polys: neutral modulation
            for (int i = 0; i < n; i++) {
                v[i].r = v[i].g = v[i].b = 0x80;
            }
        }
        {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            PROF_BEGIN();
            if (isTextured) {
                Psyz_GpuFeedbackCheck(tpage);
            }
#endif
            raster_tri(&v[0], &v[1], &v[2], isTextured, tpage, clut, isSemi,
                       isGouraud);
            if (n == 4) {
                raster_tri(&v[1], &v[3], &v[2], isTextured, tpage, clut, isSemi,
                           isGouraud);
            }
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            PROF_END(isTextured ? 0 : (isSemi ? 1 : 2));
#endif
        }
    } else if (isLine) {
        bool padding = true;
        int nPoints = ((code >> 2) & 3) + 1;
        if (nPoints == 1) {
            padding = false;
            nPoints++;
        }
        SV pts[4];
        int parsed = 0;
        pts[0].r = r0;
        pts[0].g = g0;
        pts[0].b = b0;
        for (int i = 0; len > 0 && i < nPoints; i++) {
            pts[i].x = s11(((short*)packets)[0]);
            pts[i].y = s11(((short*)packets)[1]);
            packets++;
            len--;
            parsed = i + 1;
            if (len > 0 && i + 1 < nPoints && isGouraud) {
                pts[i + 1].r = ((u8*)packets)[0];
                pts[i + 1].g = ((u8*)packets)[1];
                pts[i + 1].b = ((u8*)packets)[2];
                packets++;
                len--;
            }
        }
        if (!isGouraud) {
            for (int i = 1; i < parsed; i++) {
                pts[i].r = pts[0].r;
                pts[i].g = pts[0].g;
                pts[i].b = pts[0].b;
            }
        }
        for (int i = 0; i + 1 < parsed; i++) {
            raster_line(&pts[i], &pts[i + 1], isSemi);
        }
        if (padding && len > 0) {
            packets++;
            len--;
        }
    } else if (isTile) {
        int w = 0, h = 0;
        v[0].r = r0;
        v[0].g = g0;
        v[0].b = b0;
        v[0].x = s11(((short*)packets)[0]);
        v[0].y = s11(((short*)packets)[1]);
        packets++;
        len--;
        if (isTextured) {
            v[0].u = ((u8*)packets)[0];
            v[0].v = ((u8*)packets)[1];
            clut = ((u16*)packets)[1];
            tpage = cur_tpage;
            packets++;
            len--;
        }
        switch (code & ~3) {
        case 0x60: // TILE
        case 0x64: // SPRT
            w = ((s16*)packets)[0];
            h = ((s16*)packets)[1];
            packets++;
            len--;
            break;
        case 0x68:
        case 0x6C:
            w = h = 1;
            break;
        case 0x70:
        case 0x74:
            w = h = 8;
            break;
        case 0x78:
        case 0x7C:
            w = h = 16;
            break;
        default:
            break;
        }
        {
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            PROF_BEGIN();
#endif
            if (isTextured) {
#ifdef ESP_PLATFORM
                Psyz_GpuFeedbackCheck(tpage);
#endif
                raster_sprite(&v[0], w, h, tpage, clut, isSemi);
            } else {
                raster_tile(&v[0], w, h, isSemi, (cur_tpage >> 5) & 3);
            }
#if defined(ESP_PLATFORM) && !defined(SOFT_RASTER_NO_STATS)
            PROF_END(isTextured ? 3 : 4);
#endif
        }
    }
    return max_len - len;
}

// ---- Draw_* state ----------------------------------------------------------

static inline void soft_clip_update(void) {
    st.y0 = st.area_y0 > st.band_y0 ? st.area_y0 : st.band_y0;
    st.y1 = st.area_y1 < st.band_y1 ? st.area_y1 : st.band_y1;
}
void Draw_SetAreaStart(int x, int y) {
    st.x0 = x;
    st.area_y0 = y;
    soft_clip_update();
}
void Draw_SetAreaEnd(int x, int y) {
    st.x1 = x;
    st.area_y1 = y;
    soft_clip_update();
}
/* the rows this core's worker owns for the command stream it is executing */
void Draw_SetBand(int y0, int y1) {
    st.band_y0 = y0;
    st.band_y1 = y1;
    soft_clip_update();
}
void Draw_SetOffset(int x, int y) {
    x = x % VRAM_W;
    y = y % VRAM_H;
    if (x < 0) {
        x += VRAM_W;
    }
    if (y < 0) {
        y += VRAM_H;
    }
    st.ox = x;
    st.oy = y;
}

void Draw_ClearImage(PS1_RECT* rect, u_char r, u_char g, u_char b) {
    if (rect->w == 0 || rect->h == 0) {
        return;
    }
    soft_clear_image(rect, r, g, b);
}
void Draw_LoadImage(PS1_RECT* rect, u_long* p) {
    if (rect->w == 0 || rect->h == 0) {
        return;
    }
    soft_load_image(rect, (const u16*)p);
}
void Draw_StoreImage(PS1_RECT* rect, u_long* p) {
    if (rect->w == 0 || rect->h == 0) {
        return;
    }
    soft_store_image(rect, (u16*)p);
}
void Draw_MoveImage(PS1_RECT* rect, unsigned int x, unsigned int y) {
    if (rect->w == 0 || rect->h == 0) {
        return;
    }
    soft_move_image(rect, (int)x, (int)y);
}

void Draw_DisplayEnable(unsigned int on) { display_enabled = (int)on; }
void Draw_DisplayArea(unsigned int x, unsigned int y) {
    display_area_px.x = (int)x;
    display_area_px.y = (int)y;
}
void Draw_DisplayHorizontalRange(unsigned int s, unsigned int e) {
    (void)s;
    (void)e;
}
void Draw_DisplayVerticalRange(unsigned int s, unsigned int e) {
    (void)s;
    (void)e;
}
void Draw_SetDisplayMode(DisplayMode* mode) {
    static const int hres[4] = {256, 320, 512, 640};
    display_size_px.x = mode->horizontal_resolution_368
                            ? 368
                            : hres[mode->horizontal_resolution];
    display_size_px.y = mode->vertical_resolution ? 480 : 240;
}
void Draw_PutDispEnv(DISPENV* disp) {
    display_area_px.x = disp->disp.x;
    display_area_px.y = disp->disp.y;
    if (disp->disp.w > 0) {
        display_size_px.x = disp->disp.w;
    }
    if (disp->disp.h > 0) {
        display_size_px.y = disp->disp.h;
    }
}

void Draw_ResetBuffer(void) {}
void Draw_FlushBuffer(void) {}
int Draw_ExequeSync() { return 0; }

