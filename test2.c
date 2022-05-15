/*
 riscv32-unknown-elf-gcc -O3 -nostdlib test2.c -o test2
 riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O3 -nostdlib test2.c -o test2
*/

#define UART_TX *(volatile char *)(0x40002000ul)

void uart_putc(char);
void uart_puts(const char *);

// 目前elf加载存在一些限制, 需要将这个方法放到最前面(可以采用先声明后定义的方式)
void _start() {
    UART_TX = '7';
    UART_TX = '7';
    UART_TX = '7';
    uart_puts("hello world");
}

void _start2() {
    const char *hello = "hello from uart\n";
    while (*hello) {
        UART_TX = *hello;
        hello++;
    }
}

void uart_putc(char c) { UART_TX = c; }

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s);
        s++;
    }
}
