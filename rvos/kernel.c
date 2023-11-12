#include "uart.h"

int main(void)
{
    uart_init();
    uart_puts("hello world!\n");
    //for (;;) {};
}
