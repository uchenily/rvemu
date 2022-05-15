CROSS_COMPILE = riscv64-unknown-elf-
#CFLAGS = -nostdlib -fno-builtin -march=rv32ima -mabi=ilp32 -g -Wall
CFLAGS = -nostdlib -fno-builtin -march=rv32i -mabi=ilp32 -g -Wall
# ifdef DEBUG
# 	CFLAGS += -DDEBUG
# endif

QEMU = qemu-system-riscv32
QFLAGS = -nographic -smp 2 -machine virt -bios none

#GDB = gdb-multiarch
GDB = riscv64-unknown-elf-gdb
CC = ${CROSS_COMPILE}gcc
OBJCOPY = ${CROSS_COMPILE}objcopy
OBJDUMP = ${CROSS_COMPILE}objdump
