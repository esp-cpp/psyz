#ifndef PSYZ_MCD_H
#define PSYZ_MCD_H
/*
 * A PlayStation memory card kept in a file: the raw 128 KB image that
 * emulators call .mcd / .mcr (DuckStation, ePSXe, PCSX, mednafen read and
 * write the same bytes). The game's own saves go in it, so they do not
 * depend on the build that wrote them the way a save state does.
 *
 * Every call opens the file, does its work and closes it again: nothing is
 * cached between calls, so the card is always what is on disk (and a save
 * state restored under it cannot leave stale card data behind).
 *
 * Layout: 16 blocks of 8 KB. Block 0 holds the header frame ("MC"), the
 * directory (frames 1-15, one per data block) and the broken-sector list;
 * blocks 1-15 hold the files, a file being a chain of blocks.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCD_SIZE 0x20000
#define MCD_BLOCK 0x2000
#define MCD_FRAME 0x80
#define MCD_FILES 15
#define MCD_NAME 20 /* longest file name (the directory frame has 21 bytes) */

enum {
    MCD_OK = 0,
    MCD_ERR_IO = -1,          /* no file, short file, or a read/write failed */
    MCD_ERR_UNFORMATTED = -2, /* no "MC" header */
    MCD_ERR_NOFILE = -3,      /* no such file on the card */
    MCD_ERR_EXISTS = -4,      /* create: the name is taken */
    MCD_ERR_FULL = -5,        /* create: not enough free blocks */
    MCD_ERR_RANGE = -6,       /* read/write past the end of the file */
};

typedef struct {
    char name[MCD_NAME + 1];
    uint32_t size;  /* bytes (whole blocks) */
    int first;      /* first block, 1-15 */
} McdEntry;

/* The card's state: MCD_OK (formatted), MCD_ERR_UNFORMATTED or MCD_ERR_IO.
 * With create set, a missing file becomes a new formatted card. */
int Mcd_Check(const char* path, int create);
int Mcd_Format(const char* path);
/* Writes zeros over the header frame (libmcrd's MemCardUnformat). */
int Mcd_Unformat(const char* path);
/* The files on the card, in directory order: the count, or an error. */
int Mcd_List(const char* path, McdEntry out[MCD_FILES]);
int Mcd_Find(const char* path, const char* name, McdEntry* out);
int Mcd_FreeBlocks(const char* path);
/* A new file of `blocks` 8 KB blocks, zero filled. */
int Mcd_Create(const char* path, const char* name, int blocks);
int Mcd_Delete(const char* path, const char* name);
/* n bytes at offset ofs of the file (any sizes; the libraries ask for whole
 * 128-byte frames). */
int Mcd_Read(const char* path, const char* name, uint32_t ofs, void* buf, uint32_t n);
int Mcd_Write(const char* path, const char* name, uint32_t ofs, const void* buf, uint32_t n);
/* The BIOS's name patterns: '?' matches any one character, '*' the rest. */
int Mcd_NameMatch(const char* pattern, const char* name);

#ifdef __cplusplus
}
#endif
#endif
