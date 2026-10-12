/* Raster profiling counters, one block per core: two rasterizer workers run
 * at once, and a counter both cores bump per span bounces its cache line
 * between the two L1s. Each block is cache-line aligned. */
#ifndef PSYZ_PROF_H
#define PSYZ_PROF_H
typedef struct {
    unsigned rows, px, span_cy, texfrom_cy;
    unsigned cy[6], n[6];
    unsigned tri_off, tri_tiny, tri_ok, tri_huge;
    unsigned prim_by_code[8];
    unsigned retile_n, retile_cy; /* texture shadow regions (re)built */
    unsigned mip_tris[4];         /* textured triangles by mip level */
    /* textured spans by (clamp << 5 | bpp << 3 | gouraud << 2 | semi << 1 | modulate),
     * filled only when the rasterizer is built with SOFT_RASTER_PROF_SPANS */
    unsigned case_cy[64], case_px[64], case_rows[64];
} __attribute__((aligned(64))) PsyzProf;
extern PsyzProf psyz_prof_c[2];
#endif
