MEMORY {
    RAM (rwx) : ORIGIN = 0x1000, LENGTH = 0xFD0  /* 4KB - reserve tohost/putchar at top */
}

SECTIONS {
    .text : {
        *(.text.start)
        *(.text*)
    } > RAM

    .rodata : { *(.rodata*) } > RAM

    .data : { *(.data*) } > RAM

    .bss : {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        _bss_end = .;
    } > RAM
}

_stack_top = 0x1FD0;   /* Stack top, grows downward */
_tohost    = 0x1FF0;   /* Exit signal address: 0=pass, nonzero=fail code */
_putchar   = 0x1FE0;   /* Optional: single-byte character output */
