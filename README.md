# RVEMU

一个只包含risc-v 32位整数指令集(rv32i)的简单模拟器.


## 开始

如果不是在riscv环境下, 需要安装对应版本的riscv32交叉编译器, 例如在64位x86平台需要安装 `riscv64-unknown-elf-gcc` 来将c代码编译成risc-v指令.

rvos目录下是一个简单实现的risc-v操作系统, 仅实现了uart打印功能.

```shell
make
make run1
make run2
# 运行rvos
make run
```

## 演示

```shell
(venv) [root@archlinux rvemu]# make run1
./rvemu test1
ram_start: 0x10054
hello risc-v!
Illegal instruction: 0x00000001

> Execution time: 54153 ns
(venv) [root@archlinux rvemu]# make run2
./rvemu test2
ram_start: 0x10054
777hello worldIllegal instruction: 0x00000001

> Execution time: 51072 ns
(venv) [root@archlinux rvemu]# make run
./rvemu rvos/kernel.elf
ram_start: 0x1000
666666hello world!
Illegal instruction: 0x00000001

> Execution time: 85911 ns
```
