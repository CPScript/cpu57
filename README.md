# CPU-57 | A emulator and HDL

> [!WARNING]
> This is still in testing and some things that are expected might not be fully implemented, or disabled by default. Contributers are welcome!

## Architecture

- **Data width**: 57-bit
- **Registers**: 32 general-purpose registers (R0-R31)
- **Memory**: 1MB address space
- **I/O**: 256 ports
- **Instruction format**: 10 bytes (opcode, rd, rs1, rs2, 48-bit immediate)
- **Pipeline**: 5-stage (fetch, decode, execute, memory, writeback)

## Instruction Set

### Arithmetic
- ADD, SUB, MUL, DIV

### Logic
- AND, OR, XOR, NOT

### Shifts/Rotates
- SHL, SHR, ROL, ROR

### Memory
- LOAD, STORE, LOADI, MOV

### Control Flow
- JMP, JZ, JNZ, JC, JNC, CALL, RET

### Stack
- PUSH, POP

### I/O
- IN, OUT

### Other
- CMP, NOP, HLT

## Building

### HDL Simulation
```bash
bash build_hdl.sh 
# or
make -f Makefile test_hdl
```
Requires: Icarus Verilog, Verilator (optional), GTKWave (optional)

### Bare Metal Kernel
```bash
make iso
make run
```
Requires: Rust nightly, QEMU, grub-mkrescue

## Images

![525211649-da348ebe-007d-4f4f-92ff-c1762d3c5b3a](https://github.com/user-attachments/assets/03b61596-89fb-4dc8-b613-522edc1f64de)


The kernel displays CPU state in real-time: registers, PC, SP, flags, current instruction, bus activity, memory, and I/O ports.

## License

MIT License - see LICENSE file
