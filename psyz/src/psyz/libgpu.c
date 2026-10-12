#ifdef ESP_PLATFORM
#include "esp_memory_utils.h"
#endif
#include <psyz.h>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define PSYZ_ESP_XRAM EXT_RAM_BSS_ATTR
#else
#define PSYZ_ESP_XRAM
#endif
#include <libgpu.h>
#include <psyz/log.h>
#include "../draw.h"
#include "../internal.h"

static u32 GPU_STATUS = 0;

typedef enum {
    // https://psx-spx.consoledev.net/graphicsprocessingunitgpu/#gpu-versions
    GPU_V0, // 1MB VRAM, retail
    GPU_V1, // 2MB VRAM, arcade
    GPU_V2, // 2MB VRAM, PSone
} GpuVersion;

struct Gpu {
    /* 0x00 */ const char* ver;
    /* 0x04 */ int (*addque)(
        int (*exec)(u_long p1, u_long p2), u_long p1, u_long p2);
    /* 0x08 */ int (*addque2)(
        int (*exec)(u_long p1, u_long p2), u_long p1, int len, u_long p2);
    /* 0x0C */ int (*clr)(RECT* rect, u32 color);
    /* 0x10 */ void (*ctl)(unsigned int);
    /* 0x14 */ int (*cwb)();
    /* 0x18 */ int (*cwc)(u_long p1, u_long p2);
    /* 0x1C */ int (*drs)(u_long p1, u_long p2);
    /* 0x20 */ int (*dws)(u_long p1, u_long p2);
    /* 0x24 */ int (*exeque)();
    /* 0x28 */ int (*getctl)(int);
    /* 0x2C */ void (*otc)(OT_TYPE* ot, s32 n);
    /* 0x30 */ int (*param)(int);
    /* 0x34 */ int (*reset)(int);
    /* 0x38 */ u_long (*status)(void);
    /* 0x3C */ int (*sync)(int mode);
};

u32 get_vram_wh(void);

static void GPU_clear_cache() { NOT_IMPLEMENTED; }

static void GPU_write_image() { NOT_IMPLEMENTED; }

static void GPU_read_image() { NOT_IMPLEMENTED; }

#ifdef ESP_PLATFORM
// queue_len gets overwritten by a still-unlocated wild DRAM writer on the
// NZ0 boot path; these canaries give a hardware watchpoint a quiet target
// (nothing legitimate ever writes them)
int psyz_queue_canary_lo;
static int queue_len = 0;
int psyz_queue_canary_hi;
#endif
#define QUEUE_CAP 0x4000
#ifdef ESP_PLATFORM
/* The rasterizer runs on the other core (see Psyz_GpuWorkerStart): the game
 * fills one queue while the worker executes the other. Everything below that
 * touches queue_buf / queue_len is the FILL side and runs on the game core;
 * the worker only ever sees exec_buf / exec_len. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
PSYZ_ESP_XRAM static u_long queue_bufs[2][QUEUE_CAP];
static u_long* queue_buf = queue_bufs[0];
static int fill_idx;
static u_long* exec_buf;
static int exec_len;
static volatile int raster_busy; /* workers still on the current batch */
static volatile int enqueue_in_progress;
static TaskHandle_t raster_task;  /* core 1: the top band, or the whole frame */
static TaskHandle_t raster_task0; /* core 0: the bottom band, when the game is idle */
static SemaphoreHandle_t raster_done;
static int exec_workers;          /* 1 or 2 for the batch in flight */
static int queue_has_move;        /* a VRAM-to-VRAM move or read: one worker only */
extern int psyz_fb_texture_seen;  /* soft_raster: a frame-buffer texture was sampled */
static int fb_texture_frames;     /* ... keep to one worker for a few frames */
unsigned psyz_prof_single_batches, psyz_prof_dual_batches;
/* Where the bands split: adapted after every dual batch from the two workers'
 * wall times (core 0's includes the game preempting it) so the cores finish
 * together. Start biased to core 1 since core 0 runs the game too. */
static int raster_split_row = 136;
static int batch_split_row;       /* the split the batch in flight uses */
static unsigned exec_us_c[2];     /* wall time of each worker's last run */
unsigned psyz_prof_exeq0_us;      /* core 0 worker time, for the telemetry */
int psyz_raster_split_row;        /* telemetry copy */
static int psyz_exec_queue(u_long* queue_buf, int queue_len, int y0, int y1);
static void psyz_launch(u_long* buf, int len, int workers);
static void rec_append(const u_long* w, int n);
#include <stdio.h>
#include <string.h>
#else
static int queue_len = 0;
static u_long queue_buf[QUEUE_CAP];
#endif
#ifdef ESP_PLATFORM
/* rough phase timing: where does the frame actually go? */
unsigned psyz_prof_exeq_us, psyz_prof_load_us, psyz_prof_load_n;
#include "psyz_prof.h"
#endif
#ifdef ESP_PLATFORM
extern unsigned sotn_move_count, sotn_exeque_count;
#endif

#ifdef ESP_PLATFORM
/* Wait until the worker has finished the batch it holds (no-op without one). */
static void psyz_wait_raster(void) {
    while (raster_busy) {
        xSemaphoreTake(raster_done, portMAX_DELAY);
    }
}

/* Hand the fill queue to the worker. Blocks only while the worker is still
 * busy with the previous batch. Without a worker, executes synchronously. */
int Psyz_GpuExeque() {
    if (queue_len == 0) {
        return 0;
    }
    if (!raster_task) {
        psyz_exec_queue(queue_buf, queue_len, 0, VRAM_H - 1);
        queue_len = 0;
        return 0;
    }
    /* The vblank tick's poll (DrawSync(1)) may submit too, and it preempts
     * this thread on the same core: a submit it starts in the middle of ours
     * swapped the buffers twice and left the game filling the buffer being
     * drawn -- seen as the title screen's text and logo flickering. The
     * flag hands the tick off until the swap is published. */
    enqueue_in_progress = 1;
    psyz_wait_raster();
    {
        u_long* buf = queue_buf;
        const int len = queue_len;
        int workers;
        fill_idx ^= 1;
        queue_buf = queue_bufs[fill_idx];
        queue_len = 0;
        workers = raster_task0 ? 2 : 1;
        queue_has_move = 0;
        rec_append(buf, len);
        psyz_launch(buf, len, workers);
    }
    enqueue_in_progress = 0;
    return 0;
}

/* Rendezvous of the two workers before a primitive that samples the frame
 * buffer (see Psyz_GpuFeedbackCheck). Both execute the same stream, so both
 * arrive once per such primitive. A worker alone in its batch returns at
 * once. */
static volatile int bar_count;
static volatile unsigned bar_gen;
void Psyz_GpuBarrier(void) {
    if (exec_workers != 2) {
        return;
    }
    const unsigned gen = bar_gen;
    if (__atomic_add_fetch(&bar_count, 1, __ATOMIC_SEQ_CST) == 2) {
        bar_count = 0;
        __atomic_store_n(&bar_gen, gen + 1, __ATOMIC_RELEASE);
    } else {
        extern long long esp_timer_get_time(void);
        const long long t0 = esp_timer_get_time();
        while (__atomic_load_n(&bar_gen, __ATOMIC_ACQUIRE) == gen) {
            if (esp_timer_get_time() - t0 > 500000) {
                static int warned;
                if (!warned) {
                    warned = 1;
                    printf("[gpu] feedback barrier: partner never arrived\n");
                }
                bar_count = 0;
                break;
            }
        }
    }
}

/* start the workers on a batch; the caller has waited for the previous one */
static void psyz_launch(u_long* buf, int len, int workers) {
    exec_buf = buf;
    exec_len = len;
    exec_workers = workers;
    if (exec_workers == 2) {
        /* rebalance on the previous dual batch's times, a few rows at a time */
        if (exec_us_c[1] > exec_us_c[0] + 1000u && raster_split_row > 48) {
            raster_split_row -= 4;
        } else if (exec_us_c[0] > exec_us_c[1] + 1000u && raster_split_row < 208) {
            raster_split_row += 4;
        }
        exec_us_c[0] = exec_us_c[1] = 0;
    }
    batch_split_row = raster_split_row;
    psyz_raster_split_row = raster_split_row;
    if (exec_workers == 2) {
        psyz_prof_dual_batches++;
    } else {
        psyz_prof_single_batches++;
    }
    raster_busy = exec_workers;
    xTaskNotifyGive(raster_task);
    if (exec_workers == 2) {
        xTaskNotifyGive(raster_task0);
    }
}

/* --- frame recording and replay benchmark ------------------------------
 * Every batch is copied into a record of the current frame (DG_SwapFrame
 * marks the frame boundary). The pause menu writes the last complete frame
 * next to its VRAM dump; at the next cart start that frame is replayed
 * through the workers before the game runs, so the rasterizer's speed is
 * measured on an identical scene from build to build. */
#define REC_WORDS (QUEUE_CAP * 2)
#define REC_BATCHES 8
#define REC_FRAMES 6 /* a ring: the last complete frames, oldest first in the dump */
typedef struct {
    int nb, total;
    int len[REC_BATCHES];
} RecHdr;
static u_long* rec_words[REC_FRAMES];
static RecHdr rec_hdr[REC_FRAMES];
static int rec_cur;
static void rec_append(const u_long* w, int n) {
    RecHdr* h = &rec_hdr[rec_cur];
    if (!rec_words[0]) {
        for (int i = 0; i < REC_FRAMES; i++) {
            rec_words[i] = heap_caps_malloc(REC_WORDS * sizeof(u_long), MALLOC_CAP_SPIRAM);
            if (!rec_words[i]) {
                rec_words[0] = NULL;
                return;
            }
        }
    }
    if (h->nb >= REC_BATCHES || h->total + n > REC_WORDS) {
        return;
    }
    memcpy(rec_words[rec_cur] + h->total, w, (size_t)n * sizeof(u_long));
    h->len[h->nb++] = n;
    h->total += n;
}
void Psyz_GpuFrameMark(void) {
    rec_cur = (rec_cur + 1) % REC_FRAMES;
    rec_hdr[rec_cur].nb = 0;
    rec_hdr[rec_cur].total = 0;
}
/* file: int count, then per frame RecHdr + words; the newest frame last */
void Psyz_GpuDumpFrame(const char* path) {
    FILE* f;
    int count = 0;
    if (!rec_words[0]) {
        printf("[gpu] no recorded frame to dump\n");
        return;
    }
    f = fopen(path, "wb");
    if (!f) {
        printf("[gpu] cannot write %s\n", path);
        return;
    }
    for (int k = 1; k < REC_FRAMES; k++) {
        if (rec_hdr[(rec_cur + k) % REC_FRAMES].nb > 0) {
            count++;
        }
    }
    fwrite(&count, sizeof count, 1, f);
    for (int k = 1; k < REC_FRAMES; k++) {
        const int i = (rec_cur + k) % REC_FRAMES;
        if (rec_hdr[i].nb > 0) {
            fwrite(&rec_hdr[i], sizeof(RecHdr), 1, f);
            fwrite(rec_words[i], sizeof(u_long), (size_t)rec_hdr[i].total, f);
        }
    }
    fclose(f);
    printf("[gpu] frame dump: %s (%d frames, newest %d batches %d words)\n", path, count,
           rec_hdr[(rec_cur + REC_FRAMES - 1) % REC_FRAMES].nb,
           rec_hdr[(rec_cur + REC_FRAMES - 1) % REC_FRAMES].total);
}
static void psyz_replay_bench(void) {
    extern unsigned short g_RawVram[];
    extern long long esp_timer_get_time(void);
    void Draw_FlushCaches(void);
    RecHdr h;
    u_long* w = NULL;
    int count = 0;
    FILE* f = fopen("/sdcard/mgs_gpu_frame.bin", "rb");
    if (!f) {
        return;
    }
    if (fread(&count, sizeof count, 1, f) != 1 || count <= 0 || count > REC_FRAMES) {
        fclose(f);
        return;
    }
    /* every recorded frame but the last: rendered once, exact single worker,
     * on the dumped VRAM, and written to the card as a picture (the flicker
     * alternates between frames, so one of these should hold a bad one) */
    for (int k = 0; k < count; k++) {
        if (fread(&h, sizeof h, 1, f) != 1 || h.nb <= 0 || h.nb > REC_BATCHES || h.total <= 0 ||
            h.total > REC_WORDS) {
            fclose(f);
            free(w);
            return;
        }
        free(w);
        w = heap_caps_malloc((size_t)h.total * sizeof(u_long), MALLOC_CAP_SPIRAM);
        if (!w || fread(w, sizeof(u_long), (size_t)h.total, f) != (size_t)h.total) {
            fclose(f);
            free(w);
            return;
        }
        if (k == count - 1) {
            break; /* the newest: the comparison and timing below */
        }
        {
            extern int psyz_edge_dda, psyz_grad_float, psyz_mip_enable;
            FILE* v = fopen("/sdcard/mgs_vram.bin", "rb");
            char path[48];
            if (v) {
                fread(g_RawVram, 2, 1024u * 512u, v);
                fclose(v);
            }
            Draw_FlushCaches();
            psyz_grad_float = 0;
            psyz_edge_dda = 0;
            psyz_mip_enable = 0;
            {
                int off = 0;
                for (int b = 0; b < h.nb; b++) {
                    psyz_wait_raster();
                    psyz_launch(w + off, h.len[b], 1);
                    off += h.len[b];
                }
                psyz_wait_raster();
            }
            psyz_grad_float = 1;
            psyz_edge_dda = 1;
            snprintf(path, sizeof path, "/sdcard/mgs_cmp_f%d.bin", k);
            v = fopen(path, "wb");
            if (v) {
                for (unsigned y = 0; y < 256; y++) {
                    fwrite(&g_RawVram[y * 1024u], 2, 640, v);
                }
                fclose(v);
            }
            printf("[cmp] recorded frame %d of %d (%d words) rendered to %s\n", k, count, h.total, path);
        }
    }
    fclose(f);
    {
        /* Correctness first: the frame under each optimisation against the
         * exact-arithmetic single-worker rendering, as differing pixels in
         * the two display buffers (640x256). VRAM is reloaded from the dump
         * before every rendering. The display regions are written to the
         * card for inspection. */
        extern int psyz_edge_dda, psyz_grad_float, psyz_mip_enable;
        static const char* const names[5] = {"reference", "float gradients", "dda edges",
                                             "two workers", "two workers + mips"};
        u16* ref = heap_caps_malloc(640u * 256u * 2u, MALLOC_CAP_SPIRAM);
        for (int m = 0; m < 5 && ref; m++) {
            f = fopen("/sdcard/mgs_vram.bin", "rb");
            if (!f) {
                break;
            }
            fread(g_RawVram, 2, 1024u * 512u, f);
            fclose(f);
            Draw_FlushCaches();
            psyz_grad_float = (m == 1 || m >= 3);
            psyz_edge_dda = (m == 2 || m >= 3);
            psyz_mip_enable = (m == 4);
            {
                extern int psyz_grad_check;
                psyz_grad_check = (m == 1) ? 12 : 0;
            }
            {
                int off = 0;
                for (int b = 0; b < h.nb; b++) {
                    psyz_wait_raster();
                    psyz_launch(w + off, h.len[b], (m >= 3 && raster_task0) ? 2 : 1);
                    off += h.len[b];
                }
                psyz_wait_raster();
            }
            {
                unsigned diff = 0, maxd = 0;
                char path[48];
                for (unsigned y = 0; y < 256; y++) {
                    const u16* row = &g_RawVram[y * 1024u];
                    u16* rrow = &ref[y * 640u];
                    for (unsigned x = 0; x < 640; x++) {
                        if (m == 0) {
                            rrow[x] = row[x];
                        } else if (rrow[x] != row[x]) {
                            const unsigned a = rrow[x], b = row[x];
                            unsigned d = 0, c;
                            for (c = 0; c < 15; c += 5) {
                                const unsigned ca = (a >> c) & 31, cb = (b >> c) & 31;
                                const unsigned dc = ca > cb ? ca - cb : cb - ca;
                                if (dc > d) d = dc;
                            }
                            if ((a ^ b) & 0x8000) d = 32;
                            diff++;
                            if (d > maxd) maxd = d;
                        }
                    }
                }
                printf("[cmp] %s: %u pixels differ from the reference (max channel diff %u)\n",
                       names[m], diff, maxd);
                snprintf(path, sizeof path, "/sdcard/mgs_cmp_%d.bin", m);
                f = fopen(path, "wb");
                if (f) {
                    for (unsigned y = 0; y < 256; y++) {
                        fwrite(&g_RawVram[y * 1024u], 2, 640, f);
                    }
                    fclose(f);
                }
            }
        }
        free(ref);
        psyz_grad_float = 1;
        psyz_edge_dda = 1;
        psyz_mip_enable = 0;
    }
    f = fopen("/sdcard/mgs_vram.bin", "rb");
    if (f) {
        fread(g_RawVram, 2, 1024u * 512u, f);
        fclose(f);
    }
    Draw_FlushCaches();
    for (int mode = 0; mode < 3; mode++) {
        extern int psyz_mip_enable;
        const int iters = mode == 2 ? 10 : 30;
        psyz_mip_enable = mode == 1; /* mode 1 measures the mips ON */
        long long tsum = 0;
        int counted = 0;
        for (int it = 0; it < iters; it++) {
            const long long t0 = esp_timer_get_time();
            int off = 0;
            for (int b = 0; b < h.nb; b++) {
                psyz_wait_raster();
                psyz_launch(w + off, h.len[b], (mode != 2 && raster_task0) ? 2 : 1);
            if (mode == 1) { extern int psyz_mip_enable; psyz_mip_enable = 1; }
                off += h.len[b];
            }
            psyz_wait_raster();
            if (it >= iters / 2) {
                tsum += esp_timer_get_time() - t0;
                counted++;
            }
        }
        printf("[bench] frame replay, %d batches %d words: %s %lld us/frame, split %d\n", h.nb,
               h.total, mode == 0 ? "two workers" : mode == 1 ? "two workers, mips on" : "single worker",
               tsum / counted, raster_split_row);
        psyz_mip_enable = 0;
    }
    free(w);
    Draw_FlushCaches();
}

/* DrawSync(0): everything queued so far has been drawn. */
void Psyz_GpuSync(void) {
    Psyz_GpuExeque();
    psyz_wait_raster();
}

/* DrawSync(1): is the GPU still busy? Never blocks -- the vblank tick asks
 * this. Submits the fill queue when the worker is free so nothing sits
 * unexecuted waiting for a DrawOTag that may not come. */
static int Psyz_GpuPoll(void) {
    if (queue_len > 0 && !raster_busy && !enqueue_in_progress) {
        Psyz_GpuExeque();
    }
    return raster_busy ? 1 : 0;
}

static void psyz_raster_task(void* arg) {
    const int core = (int)(intptr_t)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!raster_busy) {
            continue; /* a stray wake: nothing was handed over */
        }
        if (core == 1) {
            psyz_exec_queue(exec_buf, exec_len, 0,
                            exec_workers == 2 ? batch_split_row - 1 : VRAM_H - 1);
        } else {
            psyz_exec_queue(exec_buf, exec_len, batch_split_row, VRAM_H - 1);
        }
        if (__atomic_sub_fetch(&raster_busy, 1, __ATOMIC_SEQ_CST) == 0) {
            xSemaphoreGive(raster_done);
        }
    }
}

void Psyz_GpuWorkerStart(void) {
    if (raster_task) {
        return;
    }
    if (!raster_done) {
        raster_done = xSemaphoreCreateBinary();
    }
    raster_busy = 0;
    {
        /* the span-loop micro-benchmark, once, before the game loads any
         * texture (a constructor is too early for the console) */
        void Draw_Benchmark(void);
        Draw_Benchmark();
    }
    if (xTaskCreatePinnedToCoreWithCaps(psyz_raster_task, "psyz_raster", 8192,
                                        (void*)1, 4, &raster_task, 1,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        raster_task = NULL;
        printf("[gpu] no raster worker: drawing stays on the game core\n");
        return;
    }
    /* the second worker: below the game threads (5) so it only takes the
     * time core 0 would otherwise idle while the game waits for the frame */
    if (xTaskCreatePinnedToCoreWithCaps(psyz_raster_task, "psyz_raster0", 8192,
                                        (void*)0, 4, &raster_task0, 0,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        raster_task0 = NULL;
        printf("[gpu] no second raster worker: core 1 draws everything\n");
    }
    printf("[gpu] raster workers on both cores (prio 4), split starts at row %d, %d-word queues x2\n",
           raster_split_row, QUEUE_CAP);
    psyz_replay_bench();
}

/* --- save states (ESP): the worker and the CLUT cache belong to this run -- */
void Mgs_SnapshotPreserve(void* p, size_t n);
void** Draw_ClutCacheSlot(void);
void Draw_FlushCaches(void);
void Psyz_GpuSnapshotSetup(void) {
    Mgs_SnapshotPreserve(&raster_task, sizeof raster_task);
    Mgs_SnapshotPreserve(&raster_task0, sizeof raster_task0);
    Mgs_SnapshotPreserve(&raster_done, sizeof raster_done);
    Mgs_SnapshotPreserve(Draw_ClutCacheSlot(), 2 * sizeof(void*));
    {
        void** Draw_TiledSlot(void);
        void** Draw_MipSlot(int* count);
        void** Draw_MipGenSlot(void);
        int n;
        void** mips;
        Mgs_SnapshotPreserve(Draw_TiledSlot(), sizeof(void*));
        mips = Draw_MipSlot(&n);
        Mgs_SnapshotPreserve(mips, (size_t)n * sizeof(void*));
        Mgs_SnapshotPreserve(Draw_MipGenSlot(), sizeof(void*));
    }
}
void Psyz_GpuAfterRestore(void) {
    raster_busy = 0;
    enqueue_in_progress = 0;
    queue_len = 0;
    fill_idx = 0;
    queue_buf = queue_bufs[0];
    Draw_FlushCaches();
}

void Psyz_GpuWorkerStop(void) {
    if (!raster_task) {
        return;
    }
    Psyz_GpuSync();
    vTaskDeleteWithCaps(raster_task);
    raster_task = NULL;
    if (raster_task0) {
        vTaskDeleteWithCaps(raster_task0);
        raster_task0 = NULL;
    }
    vSemaphoreDelete(raster_done);
    raster_done = NULL;
    raster_busy = 0;
}

volatile int psyz_trace_env; /* frames of E5/E6 tracing left (set by the game) */
void Draw_SetBand(int y0, int y1);
static int psyz_exec_queue(u_long* queue_buf, int queue_len, int band_y0, int band_y1) {
    RECT rect;
    unsigned int x, y;
    extern long long esp_timer_get_time(void);
    const long long exec_t0 = esp_timer_get_time();
    const int core = (int)esp_cpu_get_core_id();
    if (core == 1) {
        sotn_exeque_count++;
        if (psyz_trace_env > 0) {
            printf("[env] --- queue of %d words ---\n", queue_len);
            psyz_trace_env--;
        }
    }
    Draw_SetBand(band_y0, band_y1);
#else
int Psyz_GpuExeque() {
    RECT rect;
    unsigned int x, y;
#endif
    Draw_ResetBuffer();
    for (int i = 0; i < queue_len; i++) {
        u_long op = queue_buf[i];
        int code = (int)(queue_buf[i] >> 24) & 0xFF;
        // https://psx-spx.consoledev.net/graphicsprocessingunitgpu/#gpu-render-polygon-commands
        switch (code) {
        case 0x00:
            // empty?!
            break;
        case 0x01: // clear cache
            GPU_clear_cache();
            break;
        case 0x02: // frame buffer rectangle draw
            rect.x = (short)(queue_buf[i + 1] & 0xFFFF);
            rect.y = (short)((queue_buf[i + 1] >> 16) & 0xFFFF);
            rect.w = (short)(queue_buf[i + 2] & 0xFFFF);
            rect.h = (short)((queue_buf[i + 2] >> 16) & 0xFFFF);
            Draw_ClearImage(
                &rect, (u_char)(op & 0xFF), (u_char)((op >> 8) & 0xFF),
                (u_char)((op >> 16) & 0xFF));
            i += 2;
            break;
        case 0x80: // move image; the GPU truncates coords to 10/9 bits
            rect.x = (short)(queue_buf[i + 1] & 0x3FF);
            rect.y = (short)((queue_buf[i + 1] >> 16) & 0x1FF);
            rect.w = (short)(queue_buf[i + 3] & 0x7FF);
            rect.h = (short)((queue_buf[i + 3] >> 16) & 0x3FF);
            x = queue_buf[i + 2] & 0x3FF;
            y = (queue_buf[i + 2] >> 16) & 0x1FF;
#ifdef ESP_PLATFORM
            /* one worker moves, after both have drawn everything before it
             * and before either draws anything after it */
            Psyz_GpuBarrier();
            if (exec_workers != 2 || core == 1) {
                Draw_MoveImage(&rect, x, y);
            }
            Psyz_GpuBarrier();
            if (core == 1) {
                sotn_move_count++;
            }
#else
            Draw_MoveImage(&rect, x, y);
#endif
            i += 3; // consume the three parameter words or the parser
                    // re-reads them as opcodes and desyncs the stream
            break;
        case 0xA0: // write image: inline texture upload inside the display
                   // list. MGS's libdg streams stage textures this way, so an
                   // unhandled A0 leaves its pixel payload in the stream and
                   // the parser reads pixels as opcodes -- the screen fills
                   // with marching diagonal garbage as the queue desyncs.
        {
            RECT wrect;
            u32 words;
            wrect.x = (short)(queue_buf[i + 1] & 0x3FF);
            wrect.y = (short)((queue_buf[i + 1] >> 16) & 0x1FF);
            wrect.w = (short)(queue_buf[i + 2] & 0x7FF);
            wrect.h = (short)((queue_buf[i + 2] >> 16) & 0x3FF);
            words = ((u32)wrect.w * (u32)wrect.h + 1) / 2;
            if (wrect.w > 0 && wrect.h > 0 && i + 2 + (int)words < queue_len) {
                Draw_LoadImage(&wrect, (u_long*)&queue_buf[i + 3]);
            }
            i += 2 + (int)words; // consume header + pixel payload
            break;
        }
        case 0xC0: // read image
#ifdef ESP_PLATFORM
            Psyz_GpuBarrier();
            if (exec_workers != 2 || core == 1) {
                GPU_read_image();
            }
            Psyz_GpuBarrier();
#else
            GPU_read_image();
#endif
            break;
        case 0xE1:
            Draw_SetTexpageMode((ParamDrawTexpageMode*)&op);
            break;
        case 0xE2:
            // https://psx-spx.consoledev.net/graphicsprocessingunitgpu/#gp0e2h-texture-window-setting
            Draw_SetTextureWindow(
                (op & 0x1F) * 8 - 1, ((op >> 5) & 0x1F) * 8 - 1,
                ((op >> 10) & 0x1F) * 8, ((op >> 15) & 0x1F) * 8);
            break;
        case 0xE3:
#ifdef ESP_PLATFORM
            { static int b = 6; if (b > 0) { b--;
                printf("[env] E3 area start %d,%d (raw %08lx)\n",
                       (int)op & 0x3FF, (int)(op >> 10) & 0x3FF,
                       (unsigned long)op); } }
#endif
            Draw_SetAreaStart((int)op & 0x3FF, (int)(op >> 10) & 0x3FF);
            break;
        case 0xE4:
#ifdef ESP_PLATFORM
            { static int b = 6; if (b > 0) { b--;
                printf("[env] E4 area end %d,%d (raw %08lx)\n",
                       (int)op & 0x3FF, (int)(op >> 10) & 0x3FF,
                       (unsigned long)op); } }
#endif
            Draw_SetAreaEnd((int)op & 0x3FF, (int)(op >> 10) & 0x3FF);
            break;
        case 0xE5:
#ifdef ESP_PLATFORM
            { static int b = 6; if (b > 0) { b--;
                printf("[env] E5 offset %d,%d (raw %08lx)\n",
                       (int)op & 0x7FF, (int)(op >> 11) & 0x7FF,
                       (unsigned long)op); } }
#endif
            Draw_SetOffset((int)op & 0x7FF, (int)(op >> 11) & 0x7FF);
#ifdef ESP_PLATFORM
            if (psyz_trace_env > 0) {
                printf("[env] q%d/%d E5 offset %d,%d\n", i, queue_len, (int)op & 0x7FF,
                       (int)(op >> 11) & 0x7FF);
            }
#endif
            break;
        case 0xE6:
#ifdef ESP_PLATFORM
            if (psyz_trace_env > 0) {
                printf("[env] q%d/%d E6 mask set %d check %d\n", i, queue_len, (int)(op & 1),
                       (int)((op >> 1) & 1));
            }
#endif
            Draw_SetMask(!!(op & 1), !!(op & 2));
            break;
        default:
            if (code >= 0x20 && code < 0x80) {
#ifdef ESP_PLATFORM
                {
                    /* census of what actually reaches the rasterizer, and
                     * the screen coords of the first few polys */
                    static int shown = 8;
                    psyz_prof_c[core].prim_by_code[(code >> 4) & 7]++;
                    if (shown > 0 && code >= 0x20 && code < 0x40) {
                        shown--;
                        const short* w = (const short*)&queue_buf[i + 1];
                        printf("[prim] code %02X xy %d,%d %d,%d\n", code,
                               w[0], w[1], w[2], w[3]);
                    }
                    /* Who painted the whole screen?
                     *
                     * An untextured polygon big enough to cover most of the
                     * frame is never scenery -- it is a flash, a fade or a
                     * curtain. When one of those comes out opaque white the
                     * picture is gone and the cause is invisible from the
                     * panel, so name it here: its colour says whether the
                     * source is white to begin with, and bit 1 of the code is
                     * ABE, the semi-transparency enable. A flash meant to be
                     * blended that arrives with ABE clear is drawn solid, and
                     * that alone turns a lighting effect into a blank screen.
                     *
                     * Flat primitives only (0x20-0x2F): their layout is one
                     * colour word then plain XY pairs, so the vertices can be
                     * read without decoding the gouraud interleave. */
                    {
                        /* The vertex stride is not fixed: a polygon packet is
                         * one colour word and then, per vertex, XY plus a UV
                         * word if textured plus a colour word if gouraud. My
                         * first version assumed plain XY pairs and read the
                         * next primitive's PSRAM pointer as a coordinate --
                         * hence boxes like 24671,15472. Derive the stride from
                         * the code bits instead: 0x10 gouraud, 0x08 quad,
                         * 0x04 textured, 0x02 semi-transparent. */
                        static int big = 12;
                        const unsigned long* p = &queue_buf[i];
                        int lo_x = 32767, hi_x = -32768;
                        int lo_y = 32767, hi_y = -32768;

                        if (code < 0x40) {
                            int stride = 1 + ((code & 0x04) ? 1 : 0) +
                                         ((code & 0x10) ? 1 : 0);
                            int n = (code & 0x08) ? 4 : 3;
                            int v;
                            for (v = 0; v < n; v++) {
                                const short* xy =
                                    (const short*)&p[1 + (long)v * stride];
                                if (xy[0] < lo_x) lo_x = xy[0];
                                if (xy[0] > hi_x) hi_x = xy[0];
                                if (xy[1] < lo_y) lo_y = xy[1];
                                if (xy[1] > hi_y) hi_y = xy[1];
                            }
                        } else if (code >= 0x60) {
                            /* Rectangles are a different packet entirely --
                             * colour, top-left, then a size word only when the
                             * code says variable-size (bits 4-3 zero; 1/2/3
                             * mean 1x1, 8x8, 16x16). A textured one slips a UV
                             * word in between. Reading the size as if it were a
                             * second vertex is what produced boxes like
                             * 0,-7424. A full-screen fill is most likely to
                             * arrive as exactly this. */
                            const short* xy = (const short*)&p[1];
                            int sz = (code >> 3) & 3;
                            int w = sz == 2 ? 8 : sz == 3 ? 16 : 1;
                            int h = w;
                            if (sz == 0) {
                                const short* wh = (const short*)
                                    &p[2 + ((code & 0x04) ? 1 : 0)];
                                w = wh[0];
                                h = wh[1];
                            }
                            lo_x = xy[0];
                            lo_y = xy[1];
                            hi_x = xy[0] + w;
                            hi_y = xy[1] + h;
                        }
                        /* The opaque black full-screen rectangle is the routine
                         * background fill, once or twice per frame. Reporting
                         * it would burn the quota long before anything
                         * interesting happens, and what we are hunting only
                         * shows up minutes into a session. Stay quiet for it. */
                        /* Textured primitives are IN. They were excluded on the
                         * reasoning that a white flash carries no texture --
                         * which was wrong, and excluded the very thing being
                         * hunted: a searchlight cone is a textured quad, and
                         * the frame it appears in pays for three full screens
                         * of texels. Instead of narrowing by kind, dedupe by
                         * (code, colour) so a cone reported once does not
                         * report again every frame. */
                        static unsigned seen_key[16];
                        static int      n_key;
                        unsigned key = (unsigned)code << 24 |
                                       (p[0] & 0xFFFFFFu);
                        int k, fresh = 1;
                        for (k = 0; k < n_key; k++)
                            if (seen_key[k] == key) { fresh = 0; break; }

                        if (big > 0 && fresh && (hi_x - lo_x) >= 240 &&
                            (hi_y - lo_y) >= 160 &&
                            !(code == 0x60 && (p[0] & 0xFFFFFF) == 0)) {
                            unsigned long c = p[0];
                            big--;
                            if (n_key < 16) seen_key[n_key++] = key;
                            printf("[cover] code %02X abe %d tex %d rgb "
                                   "%02lX%02lX%02lX box %d,%d..%d,%d\n",
                                   code, (code >> 1) & 1, (code >> 2) & 1,
                                   c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF,
                                   lo_x, lo_y, hi_x, hi_y);
                        }
                    }
                }
#endif
                i += Draw_PushPrim(&queue_buf[i], queue_len - i) - 1;
                break;
            }
            WARNF("unsupported command %02X", code);
#ifdef ESP_PLATFORM
            {
                // stream forensics: an unknown opcode usually means the
                // stream desynced a few words earlier - dump the window
                static int dumps;
                if (dumps < 4) {
                    dumps++;
                    int lo = i >= 8 ? i - 8 : 0;
                    int hi = i + 8 <= queue_len ? i + 8 : queue_len;
                    printf("gpuq desync @%d/%d:", i, queue_len);
                    for (int k = lo; k < hi; k++) {
                        printf(" %08lx%s", (unsigned long)queue_buf[k],
                               k == i ? "*" : "");
                    }
                    printf("\n");
                }
            }
#endif
        }
    }
    Draw_FlushBuffer();
    Draw_ExequeSync();
#ifdef ESP_PLATFORM
    Draw_SetBand(0, VRAM_H - 1);
    {
        const unsigned us = (unsigned)(esp_timer_get_time() - exec_t0);
        exec_us_c[core] = us;
        if (core == 1) {
            psyz_prof_exeq_us += us;
        } else {
            psyz_prof_exeq0_us += us;
        }
    }
#else
    queue_len = 0;
#endif
    return 0;
}

#ifdef ESP_PLATFORM
// A corrupt chain link must not walk the parser into unmapped space: prims
// legitimately live in PSRAM (game buffers) or internal DRAM statics.
static inline int prim_ptr_ok(const void* p) {
    // mapped windows only: external RAM (game buffers, and the flash rodata
    // when it runs XIP from PSRAM) and internal DRAM statics; anything else
    // faults on access. The windows differ per chip (ESP32-S3: 0x3C000000 /
    // 0x3FC80000, ESP32-P4: 0x48000000 / 0x4FF00000), so ask the SoC layer.
    return esp_ptr_external_ram(p) || esp_ptr_in_dram(p);
}
#else
#define prim_ptr_ok(p) 1
#endif

static int GPU_Enqueue(u_long p1, u_long p2) {
    int mask = (int)p2;
    if (mask) {
        WARNF("mask not supported (mask:%08X)", mask);
    }
    DR_ENV* env = (DR_ENV*)(uintptr_t)p1;
    DR_ENV* prev_env = NULL;
#ifdef ESP_PLATFORM
    int walked = 0;
    enqueue_in_progress = 1;
#endif
    while (1) {
        if (!prim_ptr_ok(env)) {
            WARNF("prim chain walked to %p (bad link stored at %p) - rest "
                  "of OT dropped",
                  (void*)env, (void*)prev_env);
#ifdef ESP_PLATFORM
            if (prev_env) {
                /* the packet that carried the bad link, and the words before
                 * it: a texpage or mask command word in a link means a
                 * packet was allocated with the console's 4-byte-tag size */
                static int dumps = 4;
                if (dumps > 0) {
                    const u32* w = (const u32*)prev_env - 40;
                    dumps--;
                    printf("[gpu] OT %p: words around %p (link #%d):", (void*)(uintptr_t)p1,
                           (void*)prev_env, walked);
                    for (int k = 0; k < 52; k++) {
                        printf(" %08lx%s", (unsigned long)w[k], k == 40 ? "*" : "");
                    }
                    printf("\n");
                }
            }
#endif
            break;
        }
        prev_env = env;
#ifdef ESP_PLATFORM
        walked++;
        {
            /* a MoveImage / DR_MOVE packet (0x01 cache clear then 0x80) or a
             * StoreImage (0xC0): VRAM-to-VRAM work that one worker must do
             * before the other draws from it -> this batch runs single */
            const int c0 = (int)(getcode(env) & 0xFF);
            if ((c0 == 0x01 && env->len > 2) || c0 == 0xC0) {
                queue_has_move = 1;
            }
        }
        if ((getcode(env) & 0xFF) == 0 && env->len > 2) { /* 0x01 (cache clear) heads a MoveImage */
            // a multi-word packet with command byte 0: not a legal prim.
            // Its ADDRESS names the producer via the symbol table.
            static int zero_log = 8;
            if (zero_log > 0) {
                zero_log--;
                u32* w = (u32*)env;
                printf("[gpu] code-0 prim at %p:", (void*)env);
                for (int k = 0; k < 14; k++) {
                    printf(" %08lx", (unsigned long)w[k]);
                }
                printf("\n");
            }
        }
#endif
        if (env->len > 0x100) {
            // no legit primitive is this long: drop it WITHOUT advancing
            // queue_len, or the queue walks off into unmapped memory.
            // Logging every drop stalls the frame on slow consoles - count
            // instead (surfaced in the ESP frame telemetry).
#ifdef ESP_PLATFORM
            {
                extern unsigned sotn_pkt_drops;
                static int drop_log = 12;
                sotn_pkt_drops++;
                if (drop_log > 0) {
                    drop_log--;
                    WARNF("dropped pkt at %p: len 0x%X code 0x%02X",
                          (void*)env, (unsigned)env->len,
                          (unsigned)(getcode(env) & 0xFF));
                }
            }
#else
            ERRORF("packet 0x%X long, likely corrupted - dropped", env->len);
#endif
            if (isendprim(env)) {
                break;
            }
            env = (DR_ENV*)nextPrim(env);
            continue;
        }
        if (queue_len + env->len > QUEUE_CAP) {
            // drain and RETRY this packet: falling through without copying
            // would still bump queue_len and enqueue garbage words
            INFOF("GPU queue full, calling exeque");
            Psyz_GpuExeque();
            continue;
        }
        if (sizeof(u_long) == 4) {
            // this is fine on 32-bit systems
            memcpy(queue_buf + queue_len, env->code, env->len * sizeof(u_long));
        } else if (sizeof(u_long) == 8) {
            // Wow okay, this part is uuuugly...
            // Gpu code is usually written to a u_long array, which will work
            // fine on both 32-bit and 64-bit compiled code.
            // But primitives are mapped from structs, we need to align the data
            int code = getcode(env) & ~3;
            if (code >= 0x20 && code < 0x80) {
                // it is a prim, we need to split
                u32* prim_data = (u32*)env->code;
                for (u_long i = 0; i < env->len; i++) {
                    queue_buf[queue_len + i] = prim_data[i];
                }
            } else if (env->len > 0) {
                // TODO this is a temporary solution:
                // if gpu commands get merged with primitives, this will not
                // work
                memcpy(queue_buf + queue_len, env->code,
                       env->len * sizeof(u_long));
            }
        }
        queue_len += (int)env->len;
        if (isendprim(env)) {
            break;
        }
        env = (DR_ENV*)nextPrim(env);
    }
#ifdef ESP_PLATFORM
    /* an ordering table is a whole frame: start drawing it now, while the
     * game goes on to build the next one (the submit clears the flag) */
    Psyz_GpuExeque();
    enqueue_in_progress = 0;
#endif
    return 0;
}
static int GPU_DataWrite(u_long p1, u_long p2) {
#ifdef ESP_PLATFORM
    extern long long esp_timer_get_time(void);
    long long t = esp_timer_get_time();
    {
        /* Queue the upload behind the draws that precede it, as the GPU's
         * own 0xA0 packet with the pixels inline: the caller may reuse its
         * buffer the moment we return (on the console a DMA had to finish
         * first, but the sources are not that careful). An upload too big
         * for the queue waits for the worker and goes straight to VRAM. */
        const RECT* r = (const RECT*)(uintptr_t)p1;
        int words = ((int)r->w * (int)r->h + 1) / 2;
        int need = words + 3;
        if (p2 != 0 && r->w > 0 && r->h > 0 && need <= QUEUE_CAP / 2) {
            /* (a null source -- the menu's "nouse" slots do this -- takes the
             * direct path, which has always tolerated it) */
            if (queue_len + need > QUEUE_CAP) {
                Psyz_GpuExeque();
            }
            enqueue_in_progress = 1;
            queue_buf[queue_len] = 0xA0000000u;
            queue_buf[queue_len + 1] =
                ((u_long)(r->y & 0x1FF) << 16) | (u_long)(r->x & 0x3FF);
            queue_buf[queue_len + 2] =
                ((u_long)(r->h & 0x3FF) << 16) | (u_long)(r->w & 0x7FF);
            memcpy(&queue_buf[queue_len + 3], (const void*)(uintptr_t)p2,
                   (size_t)words * 4u);
            queue_len += need;
            enqueue_in_progress = 0;
        } else {
            Psyz_GpuSync();
            Draw_LoadImage((RECT*)(uintptr_t)p1, (u_long*)(uintptr_t)p2);
        }
    }
#else
    Psyz_GpuExeque();
    Draw_LoadImage((RECT*)(uintptr_t)p1, (u_long*)(uintptr_t)p2);
#endif
#ifdef ESP_PLATFORM
    psyz_prof_load_us += (unsigned)(esp_timer_get_time() - t);
    psyz_prof_load_n++;
    {
        /* who loads an image every frame in steady state? */
        static unsigned calls;
        calls++;
        if (calls > 400 && calls <= 406) {
            const RECT* r = (const RECT*)(uintptr_t)p1;
            printf("[load] rect %d,%d %dx%d\n", r->x, r->y, r->w, r->h);
        }
    }
#endif
    return 0;
}
static int GPU_DataRead(u_long p1, u_long p2) {
#ifdef ESP_PLATFORM
    Psyz_GpuSync(); /* the read must see every queued draw */
#else
    Psyz_GpuExeque();
#endif
    Draw_StoreImage((RECT*)(uintptr_t)p1, (u_long*)(uintptr_t)p2);
    return 0;
}

static int _param(int x) { return 0; }
static int psyz_addque2(
    int (*exec)(u_long p1, u_long p2), u_long p1, int len, u_long p2) {
    return exec(p1, p2);
}
static int psyz_addque(
    int (*exec)(u_long p1, u_long p2), u_long p1, u_long p2) {
    return psyz_addque2(exec, p1, 0, p2);
}

// psyz_clr is very similar to _clr from libgpu/sys.c
static DR_ENV clear_cmd;
static int psyz_clr(RECT* rect, u32 color) {
    const u32 wh = get_vram_wh();
    const unsigned short w = wh & 0xFFFF;
    const unsigned short h = wh >> 16;
    rect->w = CLAMP(rect->w, 0, w - 1);
    rect->h = CLAMP(rect->h, 0, h - 1);

    setlen(&clear_cmd, 5);
    clear_cmd.code[0] = 0xE6000000; // mask bit setting
    clear_cmd.code[1] = 0xE1000000 | GPU_STATUS & 0x7FF | (color >> 0x1F) << 10;
    clear_cmd.code[2] = (color & 0xFFFFFF) | 0x02000000;
    clear_cmd.code[3] = (u_long) * (u32*)&rect->x;
    clear_cmd.code[4] = (u_long) * (u32*)&rect->w;
    termPrim(&clear_cmd);
    GPU_Enqueue((u_long)&clear_cmd, 0);
    return 0;
}

void Psyz_GpuDisplayCommand(unsigned int cmd) {
    unsigned char op = (cmd >> 24) & 0x3F;
    switch (op) {
    case 0:
        Draw_Reset();
        break;
    case 1:
        LOG_ONCE("Reset FIFO not implemented");
        break;
    case 2:
        LOG_ONCE("Ack IRQ not implemented");
        break;
    case 3:
        Draw_DisplayEnable(!(cmd & 1));
        break;
    case 4:
        LOG_ONCE("DMA direction not implemented");
        break;
    case 5:
        Draw_DisplayArea(cmd & 0x3FF, (cmd >> 10) & 0x3FF);
        break;
    case 6:
        Draw_DisplayHorizontalRange(cmd & 0xFFF, (cmd >> 12) & 0xFFF);
        break;
    case 7:
        Draw_DisplayVerticalRange(cmd & 0x3FF, (cmd >> 10) & 0x3FF);
        break;
    case 8:
        cmd &= 0xFFFFFF;
        Draw_SetDisplayMode((DisplayMode*)&cmd);
        break;
    default:
        WARNF("unhandled ctl %02X (%08X)", op, cmd);
        break;
    }
}
static int psyz_cwb() {
    NOT_IMPLEMENTED;
    return 0;
}
static int psyz_getctl(int _) {
    NOT_IMPLEMENTED;
    return 0;
}
static void psyz_otc(OT_TYPE* ot, s32 n) {
    // The PSX built the reverse-linked table with the OTC DMA (channel 6); this
    // is that DMA in software: entry i links to entry i-1 and entry 0 carries
    // the terminator (ClearOTagR's caller may re-link entry 0 afterwards).
    // SOTN never hits this path -- it clears forward with ClearOTag -- so the
    // stub only became load-bearing with MGS, whose libdg clears reversed.
    s32 i;
    for (i = n - 1; i > 0; i--) {
        setlen(ot + i, 0);
        setaddr(ot + i, ot + i - 1);
    }
    setlen(ot, 0);
    setaddr(ot, 0xFFFFFF);
}
static int psyz_param(int _) {
    NOT_IMPLEMENTED;
    return 0;
}
static int psyz_reset(int _) { return GPU_V0; }
static u_long psyz_status(void) {
    NOT_IMPLEMENTED;
    return 0;
}
static int psyz_sync(int mode) {
    // see decomp/src/libgpu/sys.c
    // mode 0 waits until all the queue is drawn on screen
    // mode 1 process the queue and return how many elements have been queued
    // return -1 if GPU has timed out
    // but on PC the implementation is much simpler as it's always synced
#ifdef ESP_PLATFORM
    if (mode == 1) {
        /* The game asks this from its vsync callback and SKIPS the vblank
         * when the GPU is busy (a console GPU that has not finished within
         * a frame means the frame is lost). The worker is busy most of the
         * time by design -- that is the overlap we want -- so answer "idle":
         * the frame swap waits for the worker before showing a buffer, which
         * is the guarantee the check was standing in for. */
        Psyz_GpuPoll();
        return 0;
    }
    Psyz_GpuSync();
    return 0;
#else
    Psyz_GpuExeque();
    return 0;
#endif
}

int psyz_gpu_version(int mode) { return GPU_V0; }

// forwards raw GP0 words into the internal queue, then trigger execution.
void Psyz_GpuWriteGP0(unsigned int word) {
    if (queue_len >= (int)QUEUE_CAP) {
        WARNF("GPU queue full");
        return;
    }
    queue_buf[queue_len++] = (u_long)word;
}

void GPU_cw(u_long* param) {
    struct Gpu* gpu = (struct Gpu*)param;
    gpu->ver = "psyz";
    gpu->addque = psyz_addque;
    gpu->addque2 = psyz_addque2;
    gpu->clr = psyz_clr;
    gpu->ctl = Psyz_GpuDisplayCommand;
    gpu->cwb = psyz_cwb;
    gpu->cwc = GPU_Enqueue;
    gpu->drs = GPU_DataRead;
    gpu->dws = GPU_DataWrite;
    gpu->exeque = Psyz_GpuExeque;
    gpu->getctl = psyz_getctl;
    gpu->otc = psyz_otc;
    gpu->param = psyz_param;
    gpu->reset = psyz_reset;
    gpu->status = psyz_status;
    gpu->sync = psyz_sync;
}

// these are not yet decompiled
int _addque2() { return 0; }
int _exeque() { return 0; }
int get_alarm(void) { return 0; }
