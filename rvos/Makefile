include ./common.mk

SRCS_ASM = \
	   start.S \

SRCS_C = \
	 kernel.c \
	 uart.c \

OBJS = $(SRCS_ASM:.S=.o)
OBJS += $(SRCS_C:.c=.o)

all: kernel.elf
kernel.elf: ${OBJS}
	${CC} ${CFLAGS} -Wl,-Map,kernel.map -T link.lds -o $@ $^

%.o : %.c
	${CC} ${CFLAGS} -c -o $@ $<

%.o : %.S
	${CC} ${CFLAGS} -c -o $@ $<

.PHONY : run
run: all
	@${QEMU} -M ? | grep virt > /dev/null || exit
	@echo "Press CTRL-A and then X to exit QEMU"
	@echo "------------------------------------"
	@${QEMU} ${QFLAGS} -kernel kernel.elf

.PHONY : debug
debug: all
	@echo "Press CTRL-C and then input 'quit' to exit GDB and QEMU"
	@echo "-------------------------------------------------------"
	@${QEMU} ${QFLAGS} -kernel kernel.elf -s -S &
	@${GDB} kernel.elf -q -x ../gdbinit

.PHONY : dump
dump: all
	@${OBJDUMP} -S kernel.elf | less

.PHONY : clean
clean:
	rm -rf *.o *.elf
