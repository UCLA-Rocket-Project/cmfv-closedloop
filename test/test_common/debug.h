/* 
 * File for debug functions e.g. print packet as hexdump
 */

#ifndef DEBUG_H
#define DEBUG_H

// Credit to https://github.com/AsahiLinux/m1n1/blob/692239444937fc00ec55a9b845b5bc61df6345e0/src/utils.c#L26
static char ascii(char s)
{
    if (s < 0x20)
        return '.';
    if (s > 0x7E)
        return '.';
    return s;
}

// Credit to https://github.com/AsahiLinux/m1n1/blob/692239444937fc00ec55a9b845b5bc61df6345e0/src/utils.c#L26
void hexdump(const void *d, size_t len)
{
    u8 *data;
    size_t i, off;
    data = (u8 *)d;
    for (off = 0; off < len; off += 16) {
        printf("%08lx  ", off);
        for (i = 0; i < 16; i++) {
            if ((i + off) >= len)
                printf("   ");
            else
                printf("%02x ", data[off + i]);
        }

        printf(" ");
        for (i = 0; i < 16; i++) {
            if ((i + off) >= len)
                printf(" ");
            else
                printf("%c", ascii(data[off + i]));
        }
        printf("\n");
    }
}

#endif // DEBUG_H