/*
RISCV emulator for the RV32I architecture

Requires libelf-dev:
sudo apt-get install libelf-dev

Compile it like this:
gcc -O3 -Wall -lelf rvemu.c -o rvemu
*/

#include <fcntl.h>
#include <gelf.h>
#include <libelf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* memory mapped registers */
#define MTIME_ADDR 0x40000000
#define MTIMECMP_ADDR 0x40000008
#define UART_TX_ADDR 0x40002000

#define XLEN 32
/* emulate RAM */
#define RAM_SIZE 0x10000
uint8_t ram[RAM_SIZE];

/* special memory mapped registers */
uint64_t mtime;
uint64_t mtimecmp;

/* virtual start address for index 0 in the ram array */
uint32_t ram_start;

/* last byte of the memory initialized and temporary value */
uint32_t ram_last = 0;
uint32_t ram_curr = 0;

/* is set to false to exit the emulator */
int machine_running = true;

/* privilege levels */
#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3

/* CPU state */
uint32_t pc;
uint32_t next_pc;
uint32_t insn;
uint32_t reg[32]; // 32个通用寄存器

uint8_t priv = PRV_M; /* see PRV_x */
uint8_t fs;           /* MSTATUS_FS value */
uint8_t mxl;          /* MXL field in MISA register */

uint64_t insn_counter = 0;
int pending_exception; /* used during MMU exception handling */
uint32_t pending_tval;

/* CSRs */
uint32_t mstatus;    // machine status register
uint32_t mtvec;      // machine trap-vector base-address register
uint32_t mscratch;   // machine srcatch register
uint32_t mepc;       // machine exception program counter
uint32_t mcause;     // machine cause register
uint32_t mtval;      // machine trap value register
uint32_t mhartid;    // hart ID register (readonly)
uint32_t misa;       // machine ISA register
uint32_t mie;        // machine interrupt enable register
uint32_t mip;        // machine interrupt pending register
uint32_t medeleg;    // machine exception delegation register
uint32_t mideleg;    // machine interrupt delegation register
uint32_t mcounteren; // machine counter-enable register

uint32_t stvec;
uint32_t sscratch;
uint32_t sepc;
uint32_t scause;
uint32_t stval;
uint32_t satp;
uint32_t scounteren;
uint32_t load_res; /* for atomic LR/SC */

/* exception causes */
#define CAUSE_MISALIGNED_FETCH 0x0
#define CAUSE_FAULT_FETCH 0x1
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_BREAKPOINT 0x3
#define CAUSE_MISALIGNED_LOAD 0x4
#define CAUSE_FAULT_LOAD 0x5
#define CAUSE_MISALIGNED_STORE 0x6
#define CAUSE_FAULT_STORE 0x7
#define CAUSE_USER_ECALL 0x8
#define CAUSE_SUPERVISOR_ECALL 0x9
#define CAUSE_HYPERVISOR_ECALL 0xa
#define CAUSE_MACHINE_ECALL 0xb
#define CAUSE_FETCH_PAGE_FAULT 0xc
#define CAUSE_LOAD_PAGE_FAULT 0xd
#define CAUSE_STORE_PAGE_FAULT 0xf
#define CAUSE_INTERRUPT ((uint32_t)1 << 31)

/* misa CSR */
#define MCPUID_SUPER (1 << ('S' - 'A'))
#define MCPUID_USER (1 << ('U' - 'A'))
#define MCPUID_I (1 << ('I' - 'A'))
#define MCPUID_M (1 << ('M' - 'A'))
#define MCPUID_A (1 << ('A' - 'A'))
#define MCPUID_F (1 << ('F' - 'A'))
#define MCPUID_D (1 << ('D' - 'A'))
#define MCPUID_Q (1 << ('Q' - 'A'))
#define MCPUID_C (1 << ('C' - 'A'))

#define MIP_USIP (1 << 0)
#define MIP_SSIP (1 << 1)
#define MIP_HSIP (1 << 2)
#define MIP_MSIP (1 << 3)
#define MIP_UTIP (1 << 4)
#define MIP_STIP (1 << 5)
#define MIP_HTIP (1 << 6)
#define MIP_MTIP (1 << 7)
#define MIP_UEIP (1 << 8)
#define MIP_SEIP (1 << 9)
#define MIP_HEIP (1 << 10)
#define MIP_MEIP (1 << 11)

/* mstatus CSR */
#define MSTATUS_SPIE_SHIFT 5
#define MSTATUS_MPIE_SHIFT 7
#define MSTATUS_SPP_SHIFT 8
#define MSTATUS_MPP_SHIFT 11
#define MSTATUS_FS_SHIFT 13
#define MSTATUS_UXL_SHIFT 32
#define MSTATUS_SXL_SHIFT 34

#define MSTATUS_UIE (1 << 0)
#define MSTATUS_SIE (1 << 1)
#define MSTATUS_HIE (1 << 2)
#define MSTATUS_MIE (1 << 3)
#define MSTATUS_UPIE (1 << 4)
#define MSTATUS_SPIE (1 << MSTATUS_SPIE_SHIFT)
#define MSTATUS_HPIE (1 << 6)
#define MSTATUS_MPIE (1 << MSTATUS_MPIE_SHIFT)
#define MSTATUS_SPP (1 << MSTATUS_SPP_SHIFT)
#define MSTATUS_HPP (3 << 9)
#define MSTATUS_MPP (3 << MSTATUS_MPP_SHIFT)
#define MSTATUS_FS (3 << MSTATUS_FS_SHIFT)
#define MSTATUS_XS (3 << 15)
#define MSTATUS_MPRV (1 << 17)
#define MSTATUS_SUM (1 << 18)
#define MSTATUS_MXR (1 << 19)
#define MSTATUS_UXL_MASK ((uint64_t)3 << MSTATUS_UXL_SHIFT)
#define MSTATUS_SXL_MASK ((uint64_t)3 << MSTATUS_SXL_SHIFT)

// Return the number of consecutive low order 0 bits of value.  If value is
// zero, returns 32.
int ctz32(uint32_t a) {
    int i;
    if (a == 0)
        return 32;
    for (i = 0; i < 32; i++) {
        if ((a >> i) & 1)
            return i;
    }
    return 32;
}

#define SSTATUS_MASK0                                                          \
    (MSTATUS_UIE | MSTATUS_SIE | MSTATUS_UPIE | MSTATUS_SPIE | MSTATUS_SPP |   \
     MSTATUS_FS | MSTATUS_XS | MSTATUS_SUM | MSTATUS_MXR)
#define SSTATUS_MASK SSTATUS_MASK0

#define MSTATUS_MASK                                                           \
    (MSTATUS_UIE | MSTATUS_SIE | MSTATUS_MIE | MSTATUS_UPIE | MSTATUS_SPIE |   \
     MSTATUS_MPIE | MSTATUS_SPP | MSTATUS_MPP | MSTATUS_FS | MSTATUS_MPRV |    \
     MSTATUS_SUM | MSTATUS_MXR)

/* cycle and insn counters */
#define COUNTEREN_MASK ((1 << 0) | (1 << 2))

/* return the complete mstatus with the SD bit */
uint32_t get_mstatus(uint32_t mask) {
    uint32_t val;
    int sd;
    val = mstatus | (fs << MSTATUS_FS_SHIFT);
    val &= mask;
    sd =
        ((val & MSTATUS_FS) == MSTATUS_FS) | ((val & MSTATUS_XS) == MSTATUS_XS);
    if (sd)
        val |= (uint32_t)1 << (XLEN - 1);
    return val;
}

void set_mstatus(uint32_t val) {
    fs = (val >> MSTATUS_FS_SHIFT) & 3;

    uint32_t mask = MSTATUS_MASK & ~MSTATUS_FS;
    mstatus = (mstatus & ~mask) | (val & mask);
}

void invalid_csr(uint32_t *pval, uint32_t csr) {
    /* the 'time' counter is usually emulated */
    if (csr != 0xc01 && csr != 0xc81) {
        printf("csr_read: invalid CSR=0x%x\n", csr);
    }
    *pval = 0;
}

/* return -1 if invalid CSR. 0 if OK. 'will_write' indicate that the
   csr will be written after (used for CSR access check) */
int csr_read(uint32_t *pval, uint32_t csr, int will_write) {
    uint32_t val;

    if (((csr & 0xc00) == 0xc00) && will_write)
        return -1; /* read-only CSR */
    if (priv < ((csr >> 8) & 3))
        return -1; /* not enough priviledge */

    switch (csr) {
    case 0xc00: /* cycle */
    case 0xc02: /* instret */
    {
        uint32_t counteren;
        if (priv < PRV_M) {
            if (priv < PRV_S)
                counteren = scounteren;
            else
                counteren = mcounteren;
            if (((counteren >> (csr & 0x1f)) & 1) == 0) {
                invalid_csr(pval, csr);
                return -1;
            }
        }
    }
        val = (int64_t)insn_counter;
        break;
    case 0xc80: /* cycleh */
    case 0xc82: /* instreth */
    {
        uint32_t counteren;
        if (priv < PRV_M) {
            if (priv < PRV_S)
                counteren = scounteren;
            else
                counteren = mcounteren;
            if (((counteren >> (csr & 0x1f)) & 1) == 0) {
                invalid_csr(pval, csr);
                return -1;
            }
        }
    }
        val = insn_counter >> 32;
        break;

    case 0x100:
        val = get_mstatus(SSTATUS_MASK);
        break;
    case 0x104: /* sie */
        val = mie & mideleg;
        break;
    case 0x105:
        val = stvec;
        break;
    case 0x106:
        val = scounteren;
        break;
    case 0x140:
        val = sscratch;
        break;
    case 0x141:
        val = sepc;
        break;
    case 0x142:
        val = scause;
        break;
    case 0x143:
        val = stval;
        break;
    case 0x144: /* sip */
        val = mip & mideleg;
        break;
    case 0x180:
        val = satp;
        break;
    case 0x300:
        val = get_mstatus((uint32_t)-1);
        break;
    case 0x301:
        val = misa;
        val |= (uint32_t)mxl << (XLEN - 2);
        break;
    case 0x302:
        val = medeleg;
        break;
    case 0x303:
        val = mideleg;
        break;
    case 0x304:
        val = mie;
        break;
    case 0x305:
        val = mtvec;
        break;
    case 0x306:
        val = mcounteren;
        break;
    case 0x340:
        val = mscratch;
        break;
    case 0x341:
        val = mepc;
        break;
    case 0x342:
        val = mcause;
        break;
    case 0x343:
        val = mtval;
        break;
    case 0x344:
        val = mip;
        break;
    case 0xb00: /* mcycle */
    case 0xb02: /* minstret */
        val = (int64_t)insn_counter;
        break;
    case 0xb80: /* mcycleh */
    case 0xb82: /* minstreth */
        val = insn_counter >> 32;
        break;
    case 0xf14:
        val = mhartid;
        break;
    default:
        invalid_csr(pval, csr);
        /* return -1; */
        return 0;
    }

    *pval = val;
    return 0;
}

/* return -1 if invalid CSR, 0 if OK, 1 if the interpreter loop must be
   exited (e.g. XLEN was modified), 2 if TLBs have been flushed. */
int csr_write(uint32_t csr, uint32_t val) {
    uint32_t mask;
    switch (csr) {
    case 0x100: /* sstatus */
        set_mstatus((mstatus & ~SSTATUS_MASK) | (val & SSTATUS_MASK));
        break;
    case 0x104: /* sie */
        mask = mideleg;
        mie = (mie & ~mask) | (val & mask);
        break;
    case 0x105:
        stvec = val & ~3;
        break;
    case 0x106:
        scounteren = val & COUNTEREN_MASK;
        break;
    case 0x140:
        sscratch = val;
        break;
    case 0x141:
        sepc = val & ~1;
        break;
    case 0x142:
        scause = val;
        break;
    case 0x143:
        stval = val;
        break;
    case 0x144: /* sip */
        mask = mideleg;
        mip = (mip & ~mask) | (val & mask);
        break;
    case 0x180: /* no ASID implemented */
    {
        int new_mode;
        new_mode = (val >> 31) & 1;
        satp = (val & (((uint32_t)1 << 22) - 1)) | (new_mode << 31);
    }
        return 2;

    case 0x300:
        set_mstatus(val);
        break;
    case 0x301: /* misa */
        break;
    case 0x302:
        mask = (1 << (CAUSE_STORE_PAGE_FAULT + 1)) - 1;
        medeleg = (medeleg & ~mask) | (val & mask);
        break;
    case 0x303:
        mask = MIP_SSIP | MIP_STIP | MIP_SEIP;
        mideleg = (mideleg & ~mask) | (val & mask);
        break;
    case 0x304:
        mask = MIP_MSIP | MIP_MTIP | MIP_SSIP | MIP_STIP | MIP_SEIP;
        mie = (mie & ~mask) | (val & mask);
        break;
    case 0x305:
        mtvec = val & ~3;
        break;
    case 0x306:
        mcounteren = val & COUNTEREN_MASK;
        break;
    case 0x340:
        mscratch = val;
        break;
    case 0x341:
        mepc = val & ~1;
        break;
    case 0x342:
        mcause = val;
        break;
    case 0x343:
        mtval = val;
        break;
    case 0x344:
        mask = MIP_SSIP | MIP_STIP;
        mip = (mip & ~mask) | (val & mask);
        break;
    default:
        return 0;
        /* return -1; */
    }
    return 0;
}

void handle_sret() {
    int spp, spie;
    spp = (mstatus >> MSTATUS_SPP_SHIFT) & 1;
    /* set the IE state to previous IE state */
    spie = (mstatus >> MSTATUS_SPIE_SHIFT) & 1;
    mstatus = (mstatus & ~(1 << spp)) | (spie << spp);
    /* set SPIE to 1 */
    mstatus |= MSTATUS_SPIE;
    /* set SPP to U */
    mstatus &= ~MSTATUS_SPP;
    priv = spp;
    next_pc = sepc;
}

void handle_mret() {
    int mpp, mpie;
    mpp = (mstatus >> MSTATUS_MPP_SHIFT) & 3;
    /* set the IE state to previous IE state */
    mpie = (mstatus >> MSTATUS_MPIE_SHIFT) & 1;
    mstatus = (mstatus & ~(1 << mpp)) | (mpie << mpp);
    /* set MPIE to 1 */
    mstatus |= MSTATUS_MPIE;
    /* set MPP to U */
    mstatus &= ~MSTATUS_MPP;
    priv = mpp;
    next_pc = mepc;
}

void raise_exception(uint32_t cause, uint32_t tval) {
    int deleg;

    if (cause == CAUSE_ILLEGAL_INSTRUCTION) {
        machine_running = false;
        return;
    }

    if (priv <= PRV_S) {
        /* delegate the exception to the supervisor priviledge */
        if (cause & CAUSE_INTERRUPT)
            deleg = (mideleg >> (cause & (XLEN - 1))) & 1;
        else
            deleg = (medeleg >> cause) & 1;
    } else {
        deleg = 0;
    }

    if (deleg) {
        scause = cause;
        sepc = pc;
        stval = tval;
        mstatus = (mstatus & ~MSTATUS_SPIE) |
                  (((mstatus >> priv) & 1) << MSTATUS_SPIE_SHIFT);
        mstatus = (mstatus & ~MSTATUS_SPP) | (priv << MSTATUS_SPP_SHIFT);
        mstatus &= ~MSTATUS_SIE;
        priv = PRV_S;
        next_pc = stvec;
    } else {
        mcause = cause;
        mepc = pc;
        mtval = tval;
        mstatus = (mstatus & ~MSTATUS_MPIE) |
                  (((mstatus >> priv) & 1) << MSTATUS_MPIE_SHIFT);
        mstatus = (mstatus & ~MSTATUS_MPP) | (priv << MSTATUS_MPP_SHIFT);
        mstatus &= ~MSTATUS_MIE;
        priv = PRV_M;
        next_pc = mtvec;
    }
}

uint32_t get_pending_irq_mask() {
    uint32_t pending_ints, enabled_ints;

    pending_ints = mip & mie;
    if (pending_ints == 0)
        return 0;

    enabled_ints = 0;
    switch (priv) {
    case PRV_M:
        if (mstatus & MSTATUS_MIE)
            enabled_ints = ~mideleg;
        break;
    case PRV_S:
        enabled_ints = ~mideleg;
        if (mstatus & MSTATUS_SIE)
            enabled_ints |= mideleg;
        break;
    default:
    case PRV_U:
        enabled_ints = -1;
        break;
    }
    return pending_ints & enabled_ints;
}

int raise_interrupt() {
    uint32_t mask;
    int irq_num;

    mask = get_pending_irq_mask();
    if (mask == 0)
        return 0;
    irq_num = ctz32(mask); // 返回第一个不为0的位, 这个数就是中断号
    raise_exception(irq_num | CAUSE_INTERRUPT, 0);
    return -1;
}

/* read 32-bit instruction from memory by PC */
uint32_t get_instruction(uint32_t pc) {
    uint32_t ptr = pc - ram_start;
    if (ptr > RAM_SIZE)
        return 1;
    uint8_t *p = ram + ptr;
    insn_counter++;
    return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

/* read 8-bit data from memory */
int read_u8(uint8_t *pval, uint32_t addr) {
    addr -= ram_start;
    if (addr > RAM_SIZE) {
        *pval = 0;
        printf("illegal read 8, PC: 0x%08x, address: 0x%08x\n", pc,
               addr + ram_start);
        return 1;
    } else {
        uint8_t *p = ram + addr;
        *pval = p[0];
    }
    return 0;
}

/* read 16-bit data from memory */
int read_u16(uint16_t *pval, uint32_t addr) {
    if (addr & 1) {
        pending_exception = CAUSE_MISALIGNED_LOAD;
        pending_tval = addr;
        return 1;
    }
    addr -= ram_start;
    if (addr > RAM_SIZE) {
        *pval = 0;
        printf("illegal read 16, PC: 0x%08x, address: 0x%08x\n", pc,
               addr + ram_start);
        return 1;
    } else {
        uint8_t *p = ram + addr;
        *pval = p[0] | (p[1] << 8);
    }
    return 0;
}

/* read 32-bit data from memory */
int read_u32(uint32_t *pval, uint32_t addr) {
    if (addr & 3) {
        pending_exception = CAUSE_MISALIGNED_LOAD;
        pending_tval = addr;
        return 1;
    }
    // 如果读取的地址是mtime_addr, 则返回mtimecmp, 而不是从ram读取
    if (addr == MTIMECMP_ADDR) {
        *pval = (uint32_t)mtimecmp;
    } else if (addr == MTIMECMP_ADDR + 4) {
        *pval = (uint32_t)(mtimecmp >> 32);
    } else if (addr == MTIME_ADDR) {
        *pval = (uint32_t)mtime;
    } else if (addr == MTIME_ADDR + 4) {
        *pval = (uint32_t)(mtime >> 32);
    } else {
        addr -= ram_start;
        if (addr > RAM_SIZE) {
            *pval = 0;
            printf("illegal read 32, PC: 0x%08x, address: 0x%08x\n", pc,
                   addr + ram_start);
            return 1;
        } else {
            uint8_t *p = ram + addr;
            *pval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
        }
    }
    return 0;
}

/* write 8-bit data to memory */
int write_u8(uint32_t addr, uint8_t val) {
    if (addr == UART_TX_ADDR) {
        /* test for UART output, compatible with QEMU */
        printf("%c", val);
    } else {
        addr -= ram_start;
        if (addr > RAM_SIZE - 1) {
            printf("illegal write 8, PC: 0x%08x, address: 0x%08x\n", pc,
                   addr + ram_start);
            return 1;
        } else {
            uint8_t *p = ram + addr;
            p[0] = val & 0xff;
        }
    }
    return 0;
}

/* write 16-bit data to memory */
int write_u16(uint32_t addr, uint16_t val) {
    if (addr & 1) {
        pending_exception = CAUSE_MISALIGNED_STORE;
        pending_tval = addr;
        return 1;
    }
    addr -= ram_start;
    if (addr > RAM_SIZE - 2) {
        printf("illegal write 16, PC: 0x%08x, address: 0x%08x\n", pc,
               addr + ram_start);
        return 1;
    } else {
        uint8_t *p = ram + addr;
        p[0] = val & 0xff;
        p[1] = (val >> 8) & 0xff;
    }
    return 0;
}

/* write 32-bit data to memory */
int write_u32(uint32_t addr, uint32_t val) {
    if (addr & 3) {
        pending_exception = CAUSE_MISALIGNED_STORE;
        pending_tval = addr;
        return 1;
    }
    // 设置mtimecmp
    if (addr == MTIMECMP_ADDR) {
        mtimecmp = (mtimecmp & 0xffffffff00000000ll) | val;
        mip &= ~MIP_MTIP;
    } else if (addr == MTIMECMP_ADDR + 4) {
        mtimecmp = (mtimecmp & 0xffffffffll) | (((uint64_t)val) << 32);
        mip &= ~MIP_MTIP;
    } else {
        addr -= ram_start;
        if (addr > RAM_SIZE - 4) {
            return 1;
        } else {
            uint8_t *p = ram + addr;
            p[0] = val & 0xff;
            p[1] = (val >> 8) & 0xff;
            p[2] = (val >> 16) & 0xff;
            p[3] = (val >> 24) & 0xff;
        }
    }
    return 0;
}

void execute_instruction() {
    uint32_t opcode, rd, rs1, rs2, funct3;
    int32_t imm, cond, err;
    uint32_t addr, val = 0, val2;

    /* test for misaligned fetches */
    if (next_pc & 3) {
        raise_exception(CAUSE_MISALIGNED_FETCH, next_pc);
    }

    /* Instruction Formats:
     *   31          2524      2019      1514  1211      7 6           0
     * | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
     * -----------------------------------------------------------------
     * |             |   rs2   |   rs1   |     |    rd   |   opcode    |
     * |      7      |   5     |   5     |  3  |    5    |   7         |
     */
    opcode = insn & 0b01111111;
    rd = (insn >> 7) & 0b00011111;
    rs1 = (insn >> 15) & 0b00011111;
    rs2 = (insn >> 20) & 0b00011111;

#if 0
	printf("insn: 0x%08x\n", insn);
#endif
    switch (opcode) {
    case 0x37: /* lui */
        if (rd != 0)
            reg[rd] = (int32_t)(insn & 0xfffff000);
        break;

    case 0x17: /* auipc */
        if (rd != 0)
            reg[rd] = (int32_t)(pc + (int32_t)(insn & 0xfffff000));
        break;

    case 0x6f: /* jal */
        imm = ((insn >> (31 - 20)) & (1 << 20)) | ((insn >> (21 - 1)) & 0x7fe) |
              ((insn >> (20 - 11)) & (1 << 11)) | (insn & 0xff000);
        imm = (imm << 11) >> 11;
        if (rd != 0)
            reg[rd] = pc + 4;
        next_pc = (int32_t)(pc + imm);
        break;

    case 0x67: /* jalr */
        imm = (int32_t)insn >> 20;
        val = pc + 4;
        next_pc = (int32_t)(reg[rs1] + imm) & ~1;
        if (rd != 0)
            reg[rd] = val;
        break;

    case 0x63: /* BRANCH */
        funct3 = (insn >> 12) & 7;
        switch (funct3 >> 1) {
        case 0: /* beq/bne */
            cond = (reg[rs1] == reg[rs2]);
            break;
        case 2: /* blt/bge */
            cond = ((int32_t)reg[rs1] < (int32_t)reg[rs2]);
            break;
        case 3: /* bltu/bgeu */
            cond = (reg[rs1] < reg[rs2]);
            break;
        default:
            raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
            return;
        }
        cond ^= (funct3 & 1);
        if (cond) {
            imm = ((insn >> (31 - 12)) & (1 << 12)) |
                  ((insn >> (25 - 5)) & 0x7e0) | ((insn >> (8 - 1)) & 0x1e) |
                  ((insn << (11 - 7)) & (1 << 11));
            imm = (imm << 19) >> 19;
            next_pc = (int32_t)(pc + imm);
            break;
        }
        break;

    case 0x03: /* LOAD */
        funct3 = (insn >> 12) & 7;
        imm = (int32_t)insn >> 20;
        addr = reg[rs1] + imm;
        switch (funct3) {
        case 0: /* lb */
        {
            uint8_t rval;
            if (read_u8(&rval, addr)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            val = (int8_t)rval;
        } break;

        case 1: /* lh */
        {
            uint16_t rval;
            if (read_u16(&rval, addr)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            val = (int16_t)rval;
        } break;

        case 2: /* lw */
        {
            uint32_t rval;
            if (read_u32(&rval, addr)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            val = (int32_t)rval;
        } break;

        case 4: /* lbu */
        {
            uint8_t rval;
            if (read_u8(&rval, addr)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            val = rval;
        } break;

        case 5: /* lhu */
        {
            uint16_t rval;
            if (read_u16(&rval, addr)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            val = rval;
        } break;

        default:
            raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
            return;
        }
        if (rd != 0)
            reg[rd] = val;
        break;

    case 0x23: /* STORE */
        funct3 = (insn >> 12) & 7;
        imm = rd | ((insn >> (25 - 5)) & 0xfe0);
        imm = (imm << 20) >> 20;
        addr = reg[rs1] + imm;
        val = reg[rs2];
        switch (funct3) {
        case 0: /* sb */
            if (write_u8(addr, val)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            break;

        case 1: /* sh */
            if (write_u16(addr, val)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            break;

        case 2: /* sw */
            if (write_u32(addr, val)) {
                raise_exception(pending_exception, pending_tval);
                return;
            }
            break;

        default:
            raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
            return;
        }
        break;

    case 0x13: /* OP-IMM */
        funct3 = (insn >> 12) & 7;
        imm = (int32_t)insn >> 20;
        switch (funct3) {
        case 0: /* addi */
            val = (int32_t)(reg[rs1] + imm);
            break;
        case 1: /* slli */
            if ((imm & ~(XLEN - 1)) != 0) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            val = (int32_t)(reg[rs1] << (imm & (XLEN - 1)));
            break;
        case 2: /* slti */
            val = (int32_t)reg[rs1] < (int32_t)imm;
            break;
        case 3: /* sltiu */
            val = reg[rs1] < (uint32_t)imm;
            break;
        case 4: /* xori */
            val = reg[rs1] ^ imm;
            break;
        case 5: /* srli/srai */
            if ((imm & ~((XLEN - 1) | 0x400)) != 0) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            if (imm & 0x400) {
                val = (int32_t)reg[rs1] >> (imm & (XLEN - 1));
            } else {
                val = (int32_t)((uint32_t)reg[rs1] >> (imm & (XLEN - 1)));
            }
            break;
        case 6: /* ori */
            val = reg[rs1] | imm;
            break;
        case 7: /* andi */
            val = reg[rs1] & imm;
            break;
        }
        if (rd != 0)
            reg[rd] = val;
        break;

    case 0x33: /* OP */
        imm = insn >> 25;
        val = reg[rs1];
        val2 = reg[rs2];
        {
            if (imm & ~0x20) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            funct3 = ((insn >> 12) & 7) | ((insn >> (30 - 3)) & (1 << 3));
            switch (funct3) {
            case 0: /* add */
                val = (int32_t)(val + val2);
                break;
            case 0 | 8: /* sub */
                val = (int32_t)(val - val2);
                break;
            case 1: /* sll */
                val = (int32_t)(val << (val2 & (XLEN - 1)));
                break;
            case 2: /* slt */
                val = (int32_t)val < (int32_t)val2;
                break;
            case 3: /* sltu */
                val = val < val2;
                break;
            case 4: /* xor */
                val = val ^ val2;
                break;
            case 5: /* srl */
                val = (int32_t)((uint32_t)val >> (val2 & (XLEN - 1)));
                break;
            case 5 | 8: /* sra */
                val = (int32_t)val >> (val2 & (XLEN - 1));
                break;
            case 6: /* or */
                val = val | val2;
                break;
            case 7: /* and */
                val = val & val2;
                break;
            default:
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
        }
        if (rd != 0)
            reg[rd] = val;
        break;

    case 0x73: /* SYSTEM */
        funct3 = (insn >> 12) & 7;
        imm = insn >> 20;
        if (funct3 & 4)
            val = rs1;
        else
            val = reg[rs1];
        funct3 &= 3;
        switch (funct3) {
        case 1: /* csrrw & csrrwi */
            if (csr_read(&val2, imm, true)) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            val2 = (int32_t)val2;
            err = csr_write(imm, val);
            if (err < 0) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            if (rd != 0)
                reg[rd] = val2;
            if (err > 0) {
                /* pc = pc + 4; */
            }
            break;

        case 2: /* csrrs & csrrsi */
        case 3: /* csrrc & csrrci */
            if (csr_read(&val2, imm, (rs1 != 0))) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            val2 = (int32_t)val2;
            if (rs1 != 0) {
                if (funct3 == 2) {
                    val = val2 | val;
                } else {
                    val = val2 & ~val;
                }
                err = csr_write(imm, val);
                if (err < 0) {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
            } else {
                err = 0;
            }
            if (rd != 0)
                reg[rd] = val2;
            break;

        case 0:
            switch (imm) {
            case 0x000: /* ecall */
                if (insn & 0x000fff80) {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
                /* on real hardware, an exception is
                 * raised, the I-ECALL-01 compliance
                 * test tests this as well */
                raise_exception(CAUSE_USER_ECALL + priv, 0);
                return;

            case 0x001: /* ebreak */
                if (insn & 0x000fff80) {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
                raise_exception(CAUSE_BREAKPOINT, 0);
                return;

            case 0x102: /* sret */
            {
                if ((insn & 0x000fff80) || (priv < PRV_S)) {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
                handle_sret();
                return;
            } break;

            case 0x105: /* wfi */
                /* wait for interrupt: it is allowed to execute
                 * it as nop */
                break;

            case 0x302: /* mret */
            {
                if ((insn & 0x000fff80) || (priv < PRV_M)) {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
                handle_mret();
                return;
            } break;

            default:
                if ((imm >> 5) == 0x09) {
                    /* sfence.vma */
                    if ((insn & 0x00007f80) || (priv == PRV_U)) {
                        raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                        return;
                    }
                } else {
                    raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                    return;
                }
                break;
            }
            break;

        default:
            raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
            return;
        }
        break;

    case 0x0f: /* MISC-MEM */

        funct3 = (insn >> 12) & 7;
        switch (funct3) {
        case 0: /* fence */
            if (insn & 0xf00fff80) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            break;

        case 1: /* fence.i */
            if (insn != 0x0000100f) {
                raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
                return;
            }
            break;

        default:
            raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
            return;
        }
        break;

    default:
        printf("Illegal instruction: 0x%08x\n", insn);
        raise_exception(CAUSE_ILLEGAL_INSTRUCTION, insn);
        return;
    }
}

/* returns realtime in nanoseconds */
int64_t get_clock() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int load_elf(const char *elf_file) {
    /*
     * 通过libelf库加载ELF文件, 主要执行两个步骤
     * 1.扫描符号表, 设置程序起始位置
     * 2.将程序加载到内存
     */
    /* open ELF file */
    elf_version(EV_CURRENT);
    int fd = open(elf_file, O_RDONLY);
    if (fd == -1) {
        printf("can't open file %s\n", elf_file);
        return 1;
    }
    Elf *elf = elf_begin(fd, ELF_C_READ, NULL);

    /* scan for symbol table */
    Elf_Scn *scn = NULL;
    GElf_Shdr shdr;
    while ((scn = elf_nextscn(elf, scn)) != NULL) {
        gelf_getshdr(scn, &shdr);
        // 符号表
        if (shdr.sh_type == SHT_SYMTAB) {
            Elf_Data *data = elf_getdata(scn, NULL);
            int count = shdr.sh_size / shdr.sh_entsize;
            for (int i = 0; i < count; i++) {
                GElf_Sym sym;
                gelf_getsym(data, i, &sym);
                char *name = elf_strptr(elf, shdr.sh_link, sym.st_name);
#if 0
                if(*name) printf("sym '%s' %lx\n",name,sym.st_value);
#endif
                /* for compliance test */
                if (strcmp(name, "_start") == 0) {
                    ram_start = sym.st_value;
                }
            }
        }
    }

    /* scan for program */
    while ((scn = elf_nextscn(elf, scn)) != NULL) {
        gelf_getshdr(scn, &shdr);
        // SHT_PROGBITS: 程序段, 代码段, 数据段
        if (shdr.sh_type == SHT_PROGBITS) {
            Elf_Data *data = elf_getdata(scn, NULL);
            if (shdr.sh_addr >= ram_start) {
                for (size_t i = 0; i < shdr.sh_size; i++) {
                    ram_curr = shdr.sh_addr + i - ram_start;
                    if (ram_curr >= RAM_SIZE) {
                        /* break; */
                        // printf("? ram_curr: 0x%x\n",
                        // ram_curr);
                    } else {
                        // printf("ram_curr: 0x%x\n",
                        // ram_curr);
                        ram[ram_curr] = ((uint8_t *)data->d_buf)[i];
                        if (ram_curr > ram_last)
                            ram_last = ram_curr;
                    }
                }
            } else {
                //    //printf(".");
                //    // 这里处理有问题,
                //    地址小于ram_start的不会加载 ==>
                //    如果保证_start地址在最前面倒也没问题
            }
        }
    }
    /* close ELF file */
    elf_end(elf);
    close(fd);
    return 0;
}

void riscv_interpret() {
    while (machine_running) {
#if 1
        /* update timer, assuming 10 MHz clock (100 ns period) for the
         * mtime counter */
        mtime = get_clock() / 100ll;

        /* for reproducible debug runs, you can use a fixed
         * increment per instruction */
#else
        mtime += 10;
#endif
        /* default value for next PC is next instruction, can be changed
         * by branches or exceptions */
        next_pc = pc + 4;

        /* test for timer interrupt */
        if (mtimecmp <= mtime) {
            mip |= MIP_MTIP;
        }
        if ((mip & mie) != 0 && (mstatus & MSTATUS_MIE)) {
            raise_interrupt();
        }

        /* normal instruction execution */
        insn = get_instruction(pc);

        execute_instruction();

        /* update current PC */
        pc = next_pc;
    }
}

int main(int argc, char **argv) {

    /* automatic STDOUT flushing, no fflush needed */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* parse command line */
    const char *elf_file = NULL;
    for (int i = 1; i < argc; i++) {
        char *args = argv[i];
        if (args[0] != '-') {
            elf_file = args;
        }
    }
    if (elf_file == NULL) {
        printf("missing ELF file\n");
        return 1;
    }

    /* 内存清零 */
    for (uint32_t u = 0; u < RAM_SIZE; u++)
        ram[u] = 0;

    int ret = load_elf(elf_file);
    if (ret != 0) {
        printf("Failed to load ELF file\n");
        return 2;
    }

    uint64_t ns1 = get_clock();

    /* run program in emulator */
    pc = ram_start;
    printf("ram_start: 0x%0x\n", ram_start);
    riscv_interpret();

    uint64_t ns2 = get_clock();

#if 1
    printf("\n");
    printf("> Execution time: %llu ns\n", (unsigned long long)(ns2 - ns1));
#endif
    return 0;
}
