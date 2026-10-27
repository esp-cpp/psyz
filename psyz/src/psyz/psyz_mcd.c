/* A memory card in a 128 KB image file (see psyz/mcd.h). */
#include <psyz/mcd.h>

#include <stdio.h>
#include <string.h>

/* directory frame: state, size, next block, name, xor checksum */
enum {
    DIR_STATE = 0x00,
    DIR_SIZE = 0x04,
    DIR_NEXT = 0x08,
    DIR_NAMEOFF = 0x0A,
    DIR_SUM = 0x7F,
};
enum {
    ST_FREE = 0xA0,
    ST_FIRST = 0x51,
    ST_MIDDLE = 0x52,
    ST_LAST = 0x53,
};
#define NO_NEXT 0xFFFF

typedef struct {
    uint8_t f[16][MCD_FRAME]; /* header frame + the 15 directory frames */
} Dir;

static void wr32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static void frame_sum(uint8_t* fr) {
    uint8_t x = 0;
    for (int i = 0; i < DIR_SUM; i++) x ^= fr[i];
    fr[DIR_SUM] = x;
}
/* a frame the BIOS writes for an unused block (and for the broken-sector
 * list: there with 0xFFFFFFFF as the state) */
static void frame_free(uint8_t* fr, uint32_t state) {
    memset(fr, 0, MCD_FRAME);
    wr32(fr + DIR_STATE, state);
    fr[DIR_NEXT] = fr[DIR_NEXT + 1] = 0xFF;
    frame_sum(fr);
}
static int st_free(const uint8_t* fr) { return (fr[DIR_STATE] & 0xF0) == 0xA0; } /* free, or deleted */
static unsigned next_of(const uint8_t* fr) { return fr[DIR_NEXT] | fr[DIR_NEXT + 1] << 8; }

static FILE* card_open(const char* path, int write) {
    FILE* f = fopen(path, write ? "r+b" : "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0 || ftell(f) < MCD_SIZE || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return NULL;
    }
    return f;
}
static int io_at(FILE* f, uint32_t off, void* buf, uint32_t n, int write) {
    if (fseek(f, (long)off, SEEK_SET) != 0) return 0;
    return (write ? fwrite(buf, 1, n, f) : fread(buf, 1, n, f)) == n;
}
/* The directory of a formatted card. */
static int dir_read(FILE* f, Dir* d) {
    if (!io_at(f, 0, d, sizeof *d, 0)) return MCD_ERR_IO;
    if (d->f[0][0] != 'M' || d->f[0][1] != 'C') return MCD_ERR_UNFORMATTED;
    return MCD_OK;
}
static int dir_write_frame(FILE* f, Dir* d, int i) {
    frame_sum(d->f[i]);
    return io_at(f, (uint32_t)i * MCD_FRAME, d->f[i], MCD_FRAME, 1) ? MCD_OK : MCD_ERR_IO;
}
static void entry_name(const uint8_t* fr, char out[MCD_NAME + 1]) {
    memcpy(out, fr + DIR_NAMEOFF, MCD_NAME);
    out[MCD_NAME] = 0;
}
/* The directory index (1-15) of the file's first block, or 0. */
static int dir_find(const Dir* d, const char* name) {
    for (int i = 1; i <= MCD_FILES; i++) {
        const uint8_t* fr = d->f[i];
        if (fr[DIR_STATE] != ST_FIRST) continue;
        char n[MCD_NAME + 1];
        entry_name(fr, n);
        if (strncmp(n, name, MCD_NAME) == 0) return i;
    }
    return 0;
}
/* The file's blocks in order: the count (0 if the chain is broken). */
static int dir_chain(const Dir* d, int first, int out[MCD_FILES]) {
    int n = 0;
    for (int b = first; n < MCD_FILES;) {
        out[n++] = b;
        unsigned nx = next_of(d->f[b]);
        if (nx == NO_NEXT) return n;
        if (nx >= MCD_FILES) return 0;
        b = (int)nx + 1;
    }
    return 0; /* a loop */
}

int Mcd_Format(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return MCD_ERR_IO;
    static const uint8_t zero[MCD_FRAME];
    uint8_t fr[MCD_FRAME];
    int ok = 1;
    for (int i = 0; i < MCD_BLOCK / MCD_FRAME && ok; i++) {
        if (i == 0 || i == 63) { /* the header, and its copy at the end of block 0 */
            memset(fr, 0, sizeof fr);
            fr[0] = 'M';
            fr[1] = 'C';
            frame_sum(fr);
        } else if (i <= MCD_FILES) {
            frame_free(fr, ST_FREE);
        } else if (i < 36) { /* broken-sector list: none */
            frame_free(fr, 0xFFFFFFFFu);
        } else {
            memcpy(fr, zero, sizeof fr);
        }
        ok = fwrite(fr, 1, sizeof fr, f) == sizeof fr;
    }
    for (uint32_t off = MCD_BLOCK; off < MCD_SIZE && ok; off += MCD_FRAME)
        ok = fwrite(zero, 1, sizeof zero, f) == sizeof zero;
    if (fclose(f) != 0) ok = 0;
    return ok ? MCD_OK : MCD_ERR_IO;
}

int Mcd_Check(const char* path, int create) {
    FILE* f = card_open(path, 0);
    if (!f) {
        FILE* e = fopen(path, "rb");
        if (e) { /* there, but not a card image */
            fclose(e);
            return MCD_ERR_IO;
        }
        return create ? Mcd_Format(path) : MCD_ERR_IO;
    }
    Dir d;
    int r = dir_read(f, &d);
    fclose(f);
    return r;
}

int Mcd_Unformat(const char* path) {
    FILE* f = card_open(path, 1);
    if (!f) return MCD_ERR_IO;
    static const uint8_t zero[MCD_FRAME];
    int ok = io_at(f, 0, (void*)zero, sizeof zero, 1);
    if (fclose(f) != 0) ok = 0;
    return ok ? MCD_OK : MCD_ERR_IO;
}

int Mcd_List(const char* path, McdEntry out[MCD_FILES]) {
    FILE* f = card_open(path, 0);
    if (!f) return MCD_ERR_IO;
    Dir d;
    int r = dir_read(f, &d);
    fclose(f);
    if (r != MCD_OK) return r;
    int n = 0;
    for (int i = 1; i <= MCD_FILES; i++) {
        if (d.f[i][DIR_STATE] != ST_FIRST) continue;
        int chain[MCD_FILES];
        int blocks = dir_chain(&d, i, chain);
        if (!blocks) continue; /* damaged: not listed */
        entry_name(d.f[i], out[n].name);
        out[n].size = (uint32_t)blocks * MCD_BLOCK;
        out[n].first = i;
        n++;
    }
    return n;
}

int Mcd_Find(const char* path, const char* name, McdEntry* out) {
    McdEntry all[MCD_FILES];
    int n = Mcd_List(path, all);
    if (n < 0) return n;
    for (int i = 0; i < n; i++) {
        if (strncmp(all[i].name, name, MCD_NAME) != 0) continue;
        if (out) *out = all[i];
        return MCD_OK;
    }
    return MCD_ERR_NOFILE;
}

int Mcd_FreeBlocks(const char* path) {
    FILE* f = card_open(path, 0);
    if (!f) return MCD_ERR_IO;
    Dir d;
    int r = dir_read(f, &d);
    fclose(f);
    if (r != MCD_OK) return r;
    int n = 0;
    for (int i = 1; i <= MCD_FILES; i++) n += st_free(d.f[i]);
    return n;
}

int Mcd_Create(const char* path, const char* name, int blocks) {
    if (blocks < 1 || blocks > MCD_FILES || !name || !name[0]) return MCD_ERR_RANGE;
    FILE* f = card_open(path, 1);
    if (!f) return MCD_ERR_IO;
    Dir d;
    int r = dir_read(f, &d);
    if (r == MCD_OK && dir_find(&d, name)) r = MCD_ERR_EXISTS;
    int use[MCD_FILES], n = 0;
    for (int i = 1; i <= MCD_FILES && n < blocks; i++)
        if (st_free(d.f[i])) use[n++] = i;
    if (r == MCD_OK && n < blocks) r = MCD_ERR_FULL;
    /* the data first, then the directory that makes it a file */
    static const uint8_t zero[MCD_FRAME];
    for (int b = 0; b < n && r == MCD_OK; b++)
        for (uint32_t o = 0; o < MCD_BLOCK && r == MCD_OK; o += MCD_FRAME)
            if (!io_at(f, (uint32_t)use[b] * MCD_BLOCK + o, (void*)zero, MCD_FRAME, 1)) r = MCD_ERR_IO;
    for (int b = n - 1; b >= 0 && r == MCD_OK; b--) {
        uint8_t* fr = d.f[use[b]];
        memset(fr, 0, MCD_FRAME);
        wr32(fr + DIR_STATE, b == 0 ? ST_FIRST : b == n - 1 ? ST_LAST : ST_MIDDLE);
        if (b == 0) {
            wr32(fr + DIR_SIZE, (uint32_t)blocks * MCD_BLOCK);
            strncpy((char*)fr + DIR_NAMEOFF, name, MCD_NAME);
        }
        const unsigned nx = b == n - 1 ? NO_NEXT : (unsigned)(use[b + 1] - 1);
        fr[DIR_NEXT] = (uint8_t)nx;
        fr[DIR_NEXT + 1] = (uint8_t)(nx >> 8);
        r = dir_write_frame(f, &d, use[b]);
    }
    if (fclose(f) != 0 && r == MCD_OK) r = MCD_ERR_IO;
    return r;
}

int Mcd_Delete(const char* path, const char* name) {
    FILE* f = card_open(path, 1);
    if (!f) return MCD_ERR_IO;
    Dir d;
    int r = dir_read(f, &d);
    int first = r == MCD_OK ? dir_find(&d, name) : 0;
    if (r == MCD_OK && !first) r = MCD_ERR_NOFILE;
    int chain[MCD_FILES], n = r == MCD_OK ? dir_chain(&d, first, chain) : 0;
    if (r == MCD_OK && !n) n = 1, chain[0] = first; /* a broken chain: at least the entry goes */
    for (int b = 0; b < n && r == MCD_OK; b++) {
        frame_free(d.f[chain[b]], ST_FREE);
        r = dir_write_frame(f, &d, chain[b]);
    }
    if (fclose(f) != 0 && r == MCD_OK) r = MCD_ERR_IO;
    return r;
}

static int file_io(const char* path, const char* name, uint32_t ofs, void* buf, uint32_t n, int write) {
    FILE* f = card_open(path, write);
    if (!f) return MCD_ERR_IO;
    Dir d;
    int r = dir_read(f, &d);
    int first = r == MCD_OK ? dir_find(&d, name) : 0;
    if (r == MCD_OK && !first) r = MCD_ERR_NOFILE;
    int chain[MCD_FILES], blocks = r == MCD_OK ? dir_chain(&d, first, chain) : 0;
    if (r == MCD_OK && !blocks) r = MCD_ERR_IO;
    if (r == MCD_OK && ((uint64_t)ofs + n > (uint64_t)blocks * MCD_BLOCK)) r = MCD_ERR_RANGE;
    uint8_t* p = (uint8_t*)buf;
    while (r == MCD_OK && n > 0) {
        const uint32_t in = ofs % MCD_BLOCK, k = n < MCD_BLOCK - in ? n : MCD_BLOCK - in;
        if (!io_at(f, (uint32_t)chain[ofs / MCD_BLOCK] * MCD_BLOCK + in, p, k, write)) r = MCD_ERR_IO;
        p += k;
        ofs += k;
        n -= k;
    }
    if (fclose(f) != 0 && r == MCD_OK) r = MCD_ERR_IO;
    return r;
}
int Mcd_Read(const char* path, const char* name, uint32_t ofs, void* buf, uint32_t n) {
    return file_io(path, name, ofs, buf, n, 0);
}
int Mcd_Write(const char* path, const char* name, uint32_t ofs, const void* buf, uint32_t n) {
    return file_io(path, name, ofs, (void*)buf, n, 1);
}

int Mcd_NameMatch(const char* pattern, const char* name) {
    for (int i = 0; i < MCD_NAME; i++) {
        const char p = pattern[i], c = name[i];
        if (p == '*') return 1;
        if (p == 0 || c == 0) return p == c;
        if (p != '?' && p != c) return 0;
    }
    return 1;
}

/* ---- the BIOS's "buXX:" device ---- */
#define DEV_FDS 4
#define DEV_FD0 0x4D430000 /* 'MC': far from any FILE* cast to int */
static char dev_path[2][256];
static struct {
    int used, port;
    char name[MCD_NAME + 1];
    uint32_t pos, size;
} dev_fd[DEV_FDS];
static struct {
    int port, next, n;
    char pattern[MCD_NAME + 1];
    McdEntry all[MCD_FILES];
} dev_dir;

void Mcd_SetPath(int port, const char* path) {
    if (port < 0 || port > 1) return;
    snprintf(dev_path[port], sizeof dev_path[port], "%s", path ? path : "");
}
const char* Mcd_GetPath(int port) {
    return port >= 0 && port <= 1 && dev_path[port][0] ? dev_path[port] : NULL;
}
int Mcd_DevState(int port) {
    const char* p = Mcd_GetPath(port);
    return p ? Mcd_Check(p, 1) : MCD_ERR_IO;
}
int Mcd_DevPort(const char* name) {
    if (!name || name[0] != 'b' || name[1] != 'u' || name[3] != '0' || name[4] != ':') return -1;
    return name[2] == '0' ? 0 : name[2] == '1' ? 1 : -1;
}
static void dev_file_name(const char* name, char out[MCD_NAME + 1]) {
    snprintf(out, MCD_NAME + 1, "%s", name + 5);
}

int Mcd_DevOpen(const char* name, int flags) {
    const int port = Mcd_DevPort(name);
    const char* path = Mcd_GetPath(port);
    if (!path || Mcd_Check(path, 1) != MCD_OK) return -1;
    char fname[MCD_NAME + 1];
    dev_file_name(name, fname);
    if (flags & MCD_DEV_FCREAT) {
        const int blocks = (flags >> 16) & 0xFFFF;
        if (Mcd_Create(path, fname, blocks ? blocks : 1) != MCD_OK) return -1;
    }
    McdEntry e;
    if (Mcd_Find(path, fname, &e) != MCD_OK) return -1;
    for (int i = 0; i < DEV_FDS; i++) {
        if (dev_fd[i].used) continue;
        dev_fd[i].used = 1;
        dev_fd[i].port = port;
        memcpy(dev_fd[i].name, fname, sizeof fname);
        dev_fd[i].pos = 0;
        dev_fd[i].size = e.size;
        return DEV_FD0 + i;
    }
    return -1;
}
int Mcd_DevIsFd(int fd) { return fd >= DEV_FD0 && fd < DEV_FD0 + DEV_FDS && dev_fd[fd - DEV_FD0].used; }
static long dev_io(int fd, void* buf, long n, int write) {
    if (!Mcd_DevIsFd(fd) || n < 0) return -1;
    const int i = fd - DEV_FD0;
    const char* path = Mcd_GetPath(dev_fd[i].port);
    if (!path) return -1;
    if (dev_fd[i].pos >= dev_fd[i].size) return 0;
    if ((uint32_t)n > dev_fd[i].size - dev_fd[i].pos) n = (long)(dev_fd[i].size - dev_fd[i].pos);
    const int r = write ? Mcd_Write(path, dev_fd[i].name, dev_fd[i].pos, buf, (uint32_t)n)
                        : Mcd_Read(path, dev_fd[i].name, dev_fd[i].pos, buf, (uint32_t)n);
    if (r != MCD_OK) return -1;
    dev_fd[i].pos += (uint32_t)n;
    return n;
}
long Mcd_DevRead(int fd, void* buf, long n) { return dev_io(fd, buf, n, 0); }
long Mcd_DevWrite(int fd, const void* buf, long n) { return dev_io(fd, (void*)buf, n, 1); }
long Mcd_DevSeek(int fd, long offset, int whence) {
    if (!Mcd_DevIsFd(fd)) return -1;
    const int i = fd - DEV_FD0;
    long at = whence == SEEK_SET ? 0 : whence == SEEK_CUR ? (long)dev_fd[i].pos : (long)dev_fd[i].size;
    at += offset;
    if (at < 0 || at > (long)dev_fd[i].size) return -1;
    dev_fd[i].pos = (uint32_t)at;
    return at;
}
int Mcd_DevClose(int fd) {
    if (!Mcd_DevIsFd(fd)) return -1;
    dev_fd[fd - DEV_FD0].used = 0;
    return fd;
}

int Mcd_DevNext(McdEntry* out) {
    while (dev_dir.next < dev_dir.n) {
        const McdEntry* e = &dev_dir.all[dev_dir.next++];
        if (!Mcd_NameMatch(dev_dir.pattern, e->name)) continue;
        *out = *e;
        return 1;
    }
    return 0;
}
int Mcd_DevFirst(const char* pattern, McdEntry* out) {
    const int port = Mcd_DevPort(pattern);
    const char* path = Mcd_GetPath(port);
    dev_dir.n = dev_dir.next = 0;
    if (!path) return 0;
    const int n = Mcd_List(path, dev_dir.all);
    if (n < 0) return 0;
    dev_dir.n = n;
    dev_dir.port = port;
    dev_file_name(pattern, dev_dir.pattern);
    if (!dev_dir.pattern[0]) dev_dir.pattern[0] = '*';
    return Mcd_DevNext(out);
}
int Mcd_DevErase(const char* name) {
    const char* path = Mcd_GetPath(Mcd_DevPort(name));
    char fname[MCD_NAME + 1];
    if (!path) return 0;
    dev_file_name(name, fname);
    return Mcd_Delete(path, fname) == MCD_OK;
}
int Mcd_DevFormat(const char* name) {
    const char* path = Mcd_GetPath(Mcd_DevPort(name));
    return path && Mcd_Format(path) == MCD_OK;
}
