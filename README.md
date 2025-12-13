# CPU-57

A 57-bit CPU emulator with real-time hardware monitoring, written in Rust and running as a bare-metal x86-64 kernel.

## Architecture

- 57-bit word size
- 32 general-purpose registers
- 1MB memory
- 256 I/O ports
- 10-byte fixed instruction format
- Status flags: Zero, Carry, Overflow, Negative

## Instruction Set

### Arithmetic
- ADD, SUB, MUL, DIV

### Logical
- AND, OR, XOR, NOT

### Shift/Rotate
- SHL, SHR, ROL, ROR

### Memory
- LOAD, STORE, LOADI, MOV

### Control Flow
- CMP, JMP, JZ, JNZ, JC, JNC
- CALL, RET

### Stack
- PUSH, POP

### I/O
- IN, OUT

### Other
- NOP, HLT

## Building

Requirements:
- Rust nightly toolchain
- QEMU
- grub-mkrescue

```bash
make build  # Compile the kernel
make iso    # Create bootable ISO
make run    # Run in QEMU
```

## Monitor Display

The real-time monitor shows:

- **Registers**: All 32 registers, PC, and SP
- **Current Instruction**: Opcode and operands
- **Flags**: Z, C, O, N status
- **Bus Activity**: Address and data bus values
- **Memory**: 64 bytes at PC location
- **I/O Ports**: First 8 port values

## Test Program

The default program:
1. Loads 100 into R0
2. Loads 5 into R1
3. Subtracts R1 from R0
4. Multiplies R0 by 2, stores in R2
5. Writes R2 to memory at address 1000
6. Writes R2 to I/O port 5
7. Loops until R0 reaches zero
8. Halts

## Assembler

The included assembler supports:
- Labels
- Hex and decimal immediates
- Register notation (R0-R31)
- Comments (semicolon)

Example:
```asm
; Load values
LOADI R0 100
LOADI R1 5

loop:
    SUB R0 R0 R1
    JZ end
    JMP loop

end:
    HLT
```

---

## Images

* Gif of the current output
![output](https://github.com/user-attachments/assets/da348ebe-007d-4f4f-92ff-c1762d3c5b3a)


> [!WARNING]
> This project is under;
> MIT License - [see LICENSE file](https://github.com/CPScript/cpu57/blob/main/LICENSE)
