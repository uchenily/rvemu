OUTPUT_ARCH( riscv )

ENTRY( _start )

__stack_ram_size = 4K;
__heap_ram_size = 4K;

MEMORY
{
    /* 这里为什么改成0x00000000就不行了...? */
    data      : ORIGIN = 0x00001000, LENGTH = 4K
    stack     : ORIGIN = 0x00002000, LENGTH = 4K
    heap      : ORIGIN = 0x00003000, LENGTH = 4K
}

SECTIONS
{
    .init : {
        _init_start = .;
        *(.init)
        _init_end = .;
    } > data

    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss)
        *(.sbss)
        _bss_end = .;
    } > data
    PROVIDE(__bss_size = _bss_end - bss_start);

    .data : {
        _data_start = .;
        *(.data)
        *(.sdata)
        *(.rodata)
        *(.srodata)
        _data_end = .;
    } > data
    PROVIDE(__data_size = _data_end - _data_start);

    .stack (NOLOAD) : {
        _stack_start = .;
        . = __stack_ram_size;
        _stack_end = .;
    } > stack

    .heap (NOLOAD) : {
        _heap_start = .;
        . = __heap_ram_size;
        _heap_end = .;
    } > heap

    .text : {
        . = ALIGN(4);
        _text_start = .;
        *(.text)
        _text_end = .;
    } > data
}
