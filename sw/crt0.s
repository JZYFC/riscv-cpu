# Minimal RISC-V RV32I startup code
# Initializes SP, clears BSS, calls main(), writes tohost

    .section .text.start
    .global _start

_start:
    # Initialize stack pointer
    la   sp, _stack_top

    # Clear BSS segment
    la   a0, _bss_start
    la   a1, _bss_end
1:
    bge  a0, a1, 2f
    sw   zero, 0(a0)
    addi a0, a0, 4
    j    1b

2:
    # Call main()
    call main

    # Write return value to tohost
    # a0 contains main()'s return value: 0=pass, nonzero=fail code
    la   t0, _tohost
    sw   a0, 0(t0)

halt:
    # Spin forever — testbench detects tohost write and stops simulation
    j    halt
