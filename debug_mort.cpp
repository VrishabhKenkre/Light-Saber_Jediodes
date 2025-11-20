#include "mbed.h"
#include "debug_mort.h"

// static RawSerial vcp(USBTX, USBRX);   // ST-LINK virtual COM port UART

// extern "C" int fputc(int ch, FILE *f) {
//     (void)f;
//     vcp.putc((uint8_t)ch);
//     return ch;
// }
// extern "C" int fgetc(FILE *f) {
//     (void)f;
//     return vcp.getc();
// }
// static inline void ensure_vcp_baud() {
//     static bool inited = false;
//     if (!inited) { vcp.baud(115200); inited = true; }
// }

void debugprint(uint16_t number)
{
    printf("Got to %u\n",number);    
}

void debugprintHelloWorld( void )
{
    printf("I'm aliiiiiivveeee!!!\n");
} 
