.section .init, "ax"
.global _start

_start:
    # 在自己实现的模拟器上就不需要了
    #csrr    t0, mhartid
    #bnez    t0, 1f

    ## setup initial stack pointer
    #la      sp, _stack_end
    #j       main

    # 写入字符串6666666
    lui     t0,0x40002 # 设置高20位
    li      t1,54 # 字符'6'
    sb      t1,0(t0) # a5=0x40002000
    sb      t1,0(t0)
    sb      t1,0(t0)
    sb      t1,0(t0)
    sb      t1,0(t0)
    sb      t1,0(t0)

    # setup initial stack pointer
    la      sp, _stack_end
    j       main

#1:
#    wfi
#    j       1b
