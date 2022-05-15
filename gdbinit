set args 01_hello/kernel.elf
set print pretty on
b riscv_interpret
tui enable
r
