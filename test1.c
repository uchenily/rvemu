/*
 compile with:
 riscv32-unknown-elf-gcc -O3 -nostdlib test1.c -o test1
 or:
 riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O3 -nostdlib test1.c -o test1
*/
void _start() {
    volatile char *tx = (volatile char *)0x40002000;
    const char *hello = "hello risc-v!\n";
    while (*hello) {
        *tx = *hello;
        hello++;
    }
}
