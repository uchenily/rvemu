#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>

#define SFR8 *(volatile uint8_t *)

// read/write memory-maped registers

// read/write general-purpose registers

#define READ_REG(reg, value) asm volatile("mv %0, " #reg : "=r"(value));

#define WRITE_REG(reg, value) asm volatile("mv " #reg ", %0" : : "r"(value));

#endif /* _PLATFORM_H_ */
