#if 0
#include <stddef.h>

#include "pel_log.h"
#include "pel_msp.h"

#define INBUF_SIZE  64

static void setup(void);
static void loop(void);

static void
setup(void)
{
#ifndef NDEBUG
    Serial.begin(115200);
#endif
    
    Naze32Serial.begin(115200);

    pel_log_debug("Setup");
    pel_log_debug("\n");
}

static void
loop(void)
{
    uint8_t buf[INBUF_SIZE];
    uint8_t data = 0;

    (void) pel_msp_send(MSP_IDENT, (uint8_t *) &data, 0);
    (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
    (void) pel_msp_ident(buf);
}
#endif

