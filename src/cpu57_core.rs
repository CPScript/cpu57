use core::fmt;
extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;

pub const REGISTER_COUNT: usize = 32;
pub const MEMORY_SIZE: usize = 1024 * 1024;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Word57(u64);

impl Word57 {
    pub const MASK: u64 = 0x01FF_FFFF_FFFF_FFFF;
    
    pub fn new(value: u64) -> Self {
        Word57(value & Self::MASK)
    }
    
    pub fn get(&self) -> u64 {
        self.0 & Self::MASK  // always mask on read to ensure 57-bit
    }
    
    pub fn set(&mut self, value: u64) {
        self.0 = value & Self::MASK;
    }
    
    pub fn wrapping_add(&self, other: Word57) -> (Word57, bool) {
        let sum = (self.0 & Self::MASK).wrapping_add(other.0 & Self::MASK);
        let carry = sum > Self::MASK;
        (Word57::new(sum), carry)
    }
    
    pub fn wrapping_sub(&self, other: Word57) -> (Word57, bool) {
        let a = self.0 & Self::MASK;
        let b = other.0 & Self::MASK;
        let (diff, borrow) = a.overflowing_sub(b);
        (Word57::new(diff), borrow)
    }
    
    pub fn wrapping_mul(&self, other: Word57) -> Word57 {
        let a = self.0 & Self::MASK;
        let b = other.0 & Self::MASK;
        Word57::new(a.wrapping_mul(b))
    }
    
    pub fn wrapping_div(&self, other: Word57) -> Option<Word57> {
        let b = other.0 & Self::MASK;
        if b == 0 {
            None
        } else {
            let a = self.0 & Self::MASK;
            Some(Word57::new(a / b))
        }
    }
}

impl fmt::Debug for Word57 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "0x{:014X}", self.get())
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Opcode {
    NOP = 0x00,
    ADD = 0x01,
    SUB = 0x02,
    MUL = 0x03,
    DIV = 0x04,
    AND = 0x05,
    OR = 0x06,
    XOR = 0x07,
    NOT = 0x08,
    SHL = 0x09,
    SHR = 0x0A,
    ROL = 0x0B,
    ROR = 0x0C,
    LOAD = 0x10,
    STORE = 0x11,
    LOADI = 0x12,
    MOV = 0x13,
    CMP = 0x14,
    JMP = 0x20,
    JZ = 0x21,
    JNZ = 0x22,
    JC = 0x23,
    JNC = 0x24,
    CALL = 0x30,
    RET = 0x31,
    PUSH = 0x32,
    POP = 0x33,
    IN = 0x40,
    OUT = 0x41,
    HLT = 0xFF,
}

impl Opcode {
    pub fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Opcode::NOP),
            0x01 => Some(Opcode::ADD),
            0x02 => Some(Opcode::SUB),
            0x03 => Some(Opcode::MUL),
            0x04 => Some(Opcode::DIV),
            0x05 => Some(Opcode::AND),
            0x06 => Some(Opcode::OR),
            0x07 => Some(Opcode::XOR),
            0x08 => Some(Opcode::NOT),
            0x09 => Some(Opcode::SHL),
            0x0A => Some(Opcode::SHR),
            0x0B => Some(Opcode::ROL),
            0x0C => Some(Opcode::ROR),
            0x10 => Some(Opcode::LOAD),
            0x11 => Some(Opcode::STORE),
            0x12 => Some(Opcode::LOADI),
            0x13 => Some(Opcode::MOV),
            0x14 => Some(Opcode::CMP),
            0x20 => Some(Opcode::JMP),
            0x21 => Some(Opcode::JZ),
            0x22 => Some(Opcode::JNZ),
            0x23 => Some(Opcode::JC),
            0x24 => Some(Opcode::JNC),
            0x30 => Some(Opcode::CALL),
            0x31 => Some(Opcode::RET),
            0x32 => Some(Opcode::PUSH),
            0x33 => Some(Opcode::POP),
            0x40 => Some(Opcode::IN),
            0x41 => Some(Opcode::OUT),
            0xFF => Some(Opcode::HLT),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Instruction {
    pub opcode: Opcode,
    pub rd: u8,
    pub rs1: u8,
    pub rs2: u8,
    pub immediate: Word57,
}

impl Instruction {
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 10 {
            return None;
        }
        
        let opcode = Opcode::from_byte(data[0])?;
        let rd = data[1] & 0x1F;
        let rs1 = data[2] & 0x1F;
        let rs2 = data[3] & 0x1F;
        
        let mut imm_bytes = [0u8; 8];
        imm_bytes[0..6].copy_from_slice(&data[4..10]);
        let immediate = Word57::new(u64::from_le_bytes(imm_bytes));
        
        Some(Instruction {
            opcode,
            rd,
            rs1,
            rs2,
            immediate,
        })
    }
    
    pub fn encode(&self) -> [u8; 10] {
        let mut bytes = [0u8; 10];
        bytes[0] = self.opcode as u8;
        bytes[1] = self.rd & 0x1F;
        bytes[2] = self.rs1 & 0x1F;
        bytes[3] = self.rs2 & 0x1F;
        let imm_bytes = self.immediate.get().to_le_bytes();
        bytes[4..10].copy_from_slice(&imm_bytes[0..6]);
        bytes
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StatusFlags {
    pub zero: bool,
    pub carry: bool,
    pub overflow: bool,
    pub negative: bool,
}

impl StatusFlags {
    pub fn new() -> Self {
        StatusFlags {
            zero: false,
            carry: false,
            overflow: false,
            negative: false,
        }
    }
}

pub struct CPU57 {
    pub registers: [Word57; REGISTER_COUNT],
    pub pc: Word57,
    pub sp: Word57,
    pub flags: StatusFlags,
    pub memory: Vec<u8>,
    pub io_ports: [Word57; 256],
    pub cycle_count: u64,
    pub halted: bool,
    pub current_instruction: Option<Instruction>,
    pub data_bus: Word57,
    pub address_bus: Word57,
    pub bus_active: bool,
}

impl CPU57 {
    pub fn new() -> Self {
        CPU57 {
            registers: [Word57::new(0); REGISTER_COUNT],
            pc: Word57::new(0),
            sp: Word57::new(MEMORY_SIZE as u64 - 8),
            flags: StatusFlags::new(),
            memory: vec![0; MEMORY_SIZE],
            io_ports: [Word57::new(0); 256],
            cycle_count: 0,
            halted: false,
            current_instruction: None,
            data_bus: Word57::new(0),
            address_bus: Word57::new(0),
            bus_active: false,
        }
    }
    
    pub fn load_program(&mut self, program: &[u8], offset: usize) {
        let len = program.len().min(MEMORY_SIZE - offset);
        self.memory[offset..offset + len].copy_from_slice(&program[..len]);
    }
    
    pub fn read_memory(&mut self, addr: Word57) -> Word57 {
        let addr = (addr.get() as usize) % MEMORY_SIZE;
        self.address_bus = Word57::new(addr as u64);
        self.bus_active = true;
        
        let mut bytes = [0u8; 8];
        let end = (addr + 7).min(MEMORY_SIZE - 1);
        let len = end - addr + 1;
        bytes[0..len].copy_from_slice(&self.memory[addr..=end]);
        
        self.data_bus = Word57::new(u64::from_le_bytes(bytes));
        self.data_bus
    }
    
    pub fn write_memory(&mut self, addr: Word57, value: Word57) {
        let addr = (addr.get() as usize) % MEMORY_SIZE;
        self.address_bus = Word57::new(addr as u64);
        self.data_bus = value;
        self.bus_active = true;
        
        let bytes = value.get().to_le_bytes();
        let end = (addr + 7).min(MEMORY_SIZE - 1);
        let len = end - addr + 1;
        self.memory[addr..=end].copy_from_slice(&bytes[0..len]);
    }
    
    pub fn fetch(&mut self) -> Option<Instruction> {
        if self.halted {
            return None;
        }
        
        let pc_addr = self.pc.get() as usize;
        if pc_addr + 10 > MEMORY_SIZE {
            self.halted = true;
            return None;
        }
        
        let instruction = Instruction::decode(&self.memory[pc_addr..pc_addr + 10]);
        self.pc = Word57::new(self.pc.get() + 10);
        instruction
    }
    
    pub fn execute(&mut self, instruction: Instruction) {
        self.current_instruction = Some(instruction);
        self.bus_active = false;
        
        match instruction.opcode {
            Opcode::NOP => {}
            
            Opcode::ADD => {
                let a = self.registers[instruction.rs1 as usize];
                let b = self.registers[instruction.rs2 as usize];
                let (result, carry) = a.wrapping_add(b);
                self.registers[instruction.rd as usize] = result;
                self.flags.carry = carry;
                self.flags.zero = result.get() == 0;
                self.flags.negative = (result.get() & (1 << 56)) != 0;
            }
            
            Opcode::SUB => {
                let a = self.registers[instruction.rs1 as usize].get();
                let b = self.registers[instruction.rs2 as usize].get();
                let a_masked = a & Word57::MASK;
                let b_masked = b & Word57::MASK;
                let (result, borrow) = a_masked.overflowing_sub(b_masked);
                let result_word = Word57::new(result);
                self.registers[instruction.rd as usize] = result_word;
                self.flags.carry = borrow;
                self.flags.zero = result_word.get() == 0;
                self.flags.negative = (result_word.get() & (1 << 56)) != 0;
            }
            
            Opcode::MUL => {
                let a = self.registers[instruction.rs1 as usize].get();
                let b = self.registers[instruction.rs2 as usize].get();
                let a_masked = a & Word57::MASK;
                let b_masked = b & Word57::MASK;
                let result = a_masked.wrapping_mul(b_masked);
                let result_word = Word57::new(result);
                self.registers[instruction.rd as usize] = result_word;
                self.flags.zero = result_word.get() == 0;
            }
            
            Opcode::DIV => {
                let a = self.registers[instruction.rs1 as usize];
                let b = self.registers[instruction.rs2 as usize];
                if let Some(result) = a.wrapping_div(b) {
                    self.registers[instruction.rd as usize] = result;
                    self.flags.zero = result.get() == 0;
                }
            }
            
            Opcode::AND => {
                let a = self.registers[instruction.rs1 as usize].get();
                let b = self.registers[instruction.rs2 as usize].get();
                let result = Word57::new(a & b);
                self.registers[instruction.rd as usize] = result;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::OR => {
                let a = self.registers[instruction.rs1 as usize].get();
                let b = self.registers[instruction.rs2 as usize].get();
                let result = Word57::new(a | b);
                self.registers[instruction.rd as usize] = result;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::XOR => {
                let a = self.registers[instruction.rs1 as usize].get();
                let b = self.registers[instruction.rs2 as usize].get();
                let result = Word57::new(a ^ b);
                self.registers[instruction.rd as usize] = result;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::NOT => {
                let a = self.registers[instruction.rs1 as usize].get();
                let result = Word57::new(!a);
                self.registers[instruction.rd as usize] = result;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::SHL => {
                let a = self.registers[instruction.rs1 as usize].get();
                let shift = self.registers[instruction.rs2 as usize].get() & 0x3F;
                let result = Word57::new(a << shift);
                self.registers[instruction.rd as usize] = result;
                self.flags.carry = shift > 0 && (a >> (57 - shift)) != 0;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::SHR => {
                let a = self.registers[instruction.rs1 as usize].get();
                let shift = self.registers[instruction.rs2 as usize].get() & 0x3F;
                let result = Word57::new(a >> shift);
                self.registers[instruction.rd as usize] = result;
                self.flags.carry = shift > 0 && (a & ((1 << shift) - 1)) != 0;
                self.flags.zero = result.get() == 0;
            }
            
            Opcode::ROL => {
                let a = self.registers[instruction.rs1 as usize].get();
                let shift = (self.registers[instruction.rs2 as usize].get() & 0x3F) as u32;
                let result = Word57::new((a << shift) | (a >> (57 - shift)));
                self.registers[instruction.rd as usize] = result;
            }
            
            Opcode::ROR => {
                let a = self.registers[instruction.rs1 as usize].get();
                let shift = (self.registers[instruction.rs2 as usize].get() & 0x3F) as u32;
                let result = Word57::new((a >> shift) | (a << (57 - shift)));
                self.registers[instruction.rd as usize] = result;
            }
            
            Opcode::LOAD => {
                let addr = self.registers[instruction.rs1 as usize];
                let value = self.read_memory(addr);
                self.registers[instruction.rd as usize] = value;
            }
            
            Opcode::STORE => {
                let addr = self.registers[instruction.rs1 as usize];
                let value = self.registers[instruction.rd as usize];
                self.write_memory(addr, value);
            }
            
            Opcode::LOADI => {
                self.registers[instruction.rd as usize] = instruction.immediate;
            }
            
            Opcode::MOV => {
                self.registers[instruction.rd as usize] = self.registers[instruction.rs1 as usize];
            }
            
            Opcode::CMP => {
                let a = self.registers[instruction.rs1 as usize];
                let b = self.registers[instruction.rs2 as usize];
                let (result, _) = a.wrapping_sub(b);
                self.flags.zero = result.get() == 0;
                self.flags.carry = a.get() < b.get();
                self.flags.negative = (result.get() & (1 << 56)) != 0;
            }
            
            Opcode::JMP => {
                self.pc = instruction.immediate;
            }
            
            Opcode::JZ => {
                if self.flags.zero {
                    self.pc = instruction.immediate;
                }
            }
            
            Opcode::JNZ => {
                if !self.flags.zero {
                    self.pc = instruction.immediate;
                }
            }
            
            Opcode::JC => {
                if self.flags.carry {
                    self.pc = instruction.immediate;
                }
            }
            
            Opcode::JNC => {
                if !self.flags.carry {
                    self.pc = instruction.immediate;
                }
            }
            
            Opcode::CALL => {
                let return_addr = self.pc;
                self.write_memory(self.sp, return_addr);
                self.sp = Word57::new(self.sp.get().wrapping_sub(8));
                self.pc = instruction.immediate;
            }
            
            Opcode::RET => {
                self.sp = Word57::new(self.sp.get().wrapping_add(8));
                let return_addr = self.read_memory(self.sp);
                self.pc = return_addr;
            }
            
            Opcode::PUSH => {
                let value = self.registers[instruction.rd as usize];
                self.write_memory(self.sp, value);
                self.sp = Word57::new(self.sp.get().wrapping_sub(8));
            }
            
            Opcode::POP => {
                self.sp = Word57::new(self.sp.get().wrapping_add(8));
                let value = self.read_memory(self.sp);
                self.registers[instruction.rd as usize] = value;
            }
            
            Opcode::IN => {
                let port = (instruction.immediate.get() & 0xFF) as usize;
                self.registers[instruction.rd as usize] = self.io_ports[port];
            }
            
            Opcode::OUT => {
                let port = (instruction.immediate.get() & 0xFF) as usize;
                self.io_ports[port] = self.registers[instruction.rd as usize];
            }
            
            Opcode::HLT => {
                self.halted = true;
            }
        }
        
        self.cycle_count += 1;
    }
    
    pub fn step(&mut self) {
        if let Some(instruction) = self.fetch() {
            self.execute(instruction);
        }
    }
    
    pub fn run(&mut self, max_cycles: u64) {
        while !self.halted && self.cycle_count < max_cycles {
            self.step();
        }
    }
}