# RISC-V RV32I Toolchain
CROSS  = riscv64-unknown-elf
CC     = $(CROSS)-gcc
OBJCOPY = $(CROSS)-objcopy
OBJDUMP = $(CROSS)-objdump

# Compiler flags — RV32I baseline, no standard libs, no division
CFLAGS  = -march=rv32i -mabi=ilp32 -nostdlib -nostartfiles -O2 -g
LDFLAGS = -T link.ld

# Default target: build test.bin from tests/test_add.c
.PHONY: all clean dump help FORCE

TEST ?= tests/test_add.c

all: test.bin

test.bin: test.elf
	$(OBJCOPY) -O binary $< $@
	@printf "Generated: $@ (%d bytes)\n" $$(wc -c < $@)

test.elf: crt0.s $(TEST) link.ld .test-selection
	$(CC) $(CFLAGS) $(LDFLAGS) crt0.s $(TEST) -o $@

# Record the selected test so switching TEST triggers a rebuild.
.test-selection: FORCE
	@printf '%s\n' '$(TEST)' > $@.tmp
	@cmp -s $@.tmp $@ || mv $@.tmp $@
	@rm -f $@.tmp

FORCE:

# Disassembly for debugging
dump: test.elf
	$(OBJDUMP) -d -M no-aliases $< | head -100

clean:
	rm -f test.elf test.bin .test-selection

help:
	@echo "Targets:"
	@echo "  make          - Build test.bin from tests/test_add.c"
	@echo "  make dump     - Disassemble test.elf (first 100 lines)"
	@echo "  make clean    - Remove generated files"
	@echo ""
	@echo "Override test source:"
	@echo "  make TEST=tests/test_sub_simple.c"
	@echo ""
	@echo "Toolchain required: riscv64-unknown-elf-gcc"
	@echo "  Install: sudo apt-get install gcc-riscv64-unknown-elf"
