#include "uart.h"

#define UART_TX *(volatile char *)(0x40002000ul)
void uart_init(void) {}

void uart_putc(char c) { UART_TX = c; }

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s);
        s++;
    }
}
