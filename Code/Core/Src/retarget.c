// retarget.c
#include "usbd_cdc_if.h"
#include <string.h>

static int cdc_write(const uint8_t* p, uint16_t n) {
    // send in 64B chunks; wait if the USB stack is busy
    while (n) {
        uint16_t chunk = (n > 64) ? 64 : n;
        while (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_BUSY) { /* spin */ }
        p += chunk; n -= chunk;
    }
    return 0;
}

int _write(int file, char *ptr, int len)
{
    // turn \n into \r\n for terminals
    static uint8_t buf[128];
    int i = 0;
    for (int j = 0; j < len; ++j) {
        if (ptr[j] == '\n') {
            if (i >= (int)sizeof buf - 2) { cdc_write(buf, i); i = 0; }
            buf[i++] = '\r';
        }
        if (i >= (int)sizeof buf - 1) { cdc_write(buf, i); i = 0; }
        buf[i++] = (uint8_t)ptr[j];
    }
    if (i) cdc_write(buf, i);
    return len;
}
