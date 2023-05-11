#include "Vacuumizer.h"

void uart_print(const char *fmt, ...)
{
    va_list ap;
    char buffer[64];

    va_start(ap, fmt);
    vsprintf(buffer, fmt, ap);
    va_end(ap);

    for (uint8_t i = 0; i < 64; i++)
    {
        if (buffer[i] == 0)
            return;
        uart_putc(uart0, buffer[i]);
    }
    
}