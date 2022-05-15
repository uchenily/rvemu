all: rvemu

rvemu: rvemu.c rvos/link.lds
	gcc -Og -g -Wall -lelf rvemu.c -o rvemu
	cd rvos; make clean; make; cd -

test1: test1.c
	riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O3 -nostdlib test1.c -o test1

test2: test2.c
	riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O3 -nostdlib test2.c -o test2

run:
	./rvemu rvos/kernel.elf
.PHONY: run

debug:
	gdb ./rvemu -x gdbinit
.PHONY: debug

clean:
	rm rvemu test1
.PHONY: clean
