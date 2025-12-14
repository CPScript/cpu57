# CPU-57

A 57-bit CPU architecture with hardware implementation in Verilog and software emulation in Rust, running as a bare-metal x86-64 kernel.

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

## Implementation

- **hdl/cpu57.v**: Verilog implementation with CPU, ALU, memory, and I/O controller
- **src/cpu57_core.rs**: Pure Rust CPU emulator
- **src/hdl_bridge.rs**: Bridge to Verilator-compiled HDL (for hosted environments)
- **src/main.rs**: Bare metal kernel with VGA text mode monitor

The kernel displays CPU state in real-time: registers, PC, SP, flags, current instruction, bus activity, memory, and I/O ports.

## License

MIT License - see LICENSE file
