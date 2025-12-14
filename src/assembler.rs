use crate::cpu57_core::{Instruction, Opcode, Word57};
use core::str;
extern crate alloc;
use alloc::vec::Vec;

pub struct Assembler {
    labels: [(u32, u32); 256],
    label_count: usize,
}

impl Assembler {
    pub fn new() -> Self {
        Assembler {
            labels: [(0, 0); 256],
            label_count: 0,
        }
    }
    
    pub fn assemble(&mut self, source: &str) -> Result<Vec<u8>, &'static str> {
        let mut output = Vec::new();
        let mut address = 0u32;
                
        for line in source.lines() {
            let line = line.trim();
            
            if line.is_empty() || line.starts_with(';') {
                continue;
            }
            
            if line.ends_with(':') {
                let label_hash = self.hash_label(&line[..line.len()-1]);
                if self.label_count < 256 {
                    self.labels[self.label_count] = (label_hash, address);
                    self.label_count += 1;
                }
                continue;
            }
            
            if let Some(inst) = self.parse_instruction(line)? {
                output.extend_from_slice(&inst.encode());
                address += 10;
            }
        }
        
        Ok(output)
    }
    
    fn parse_instruction(&self, line: &str) -> Result<Option<Instruction>, &'static str> {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.is_empty() {
            return Ok(None);
        }
        
        let opcode = match tokens[0].to_uppercase().as_str() {
            "NOP" => Opcode::NOP,
            "ADD" => Opcode::ADD,
            "SUB" => Opcode::SUB,
            "MUL" => Opcode::MUL,
            "DIV" => Opcode::DIV,
            "AND" => Opcode::AND,
            "OR" => Opcode::OR,
            "XOR" => Opcode::XOR,
            "NOT" => Opcode::NOT,
            "SHL" => Opcode::SHL,
            "SHR" => Opcode::SHR,
            "ROL" => Opcode::ROL,
            "ROR" => Opcode::ROR,
            "LOAD" => Opcode::LOAD,
            "STORE" => Opcode::STORE,
            "LOADI" => Opcode::LOADI,
            "MOV" => Opcode::MOV,
            "CMP" => Opcode::CMP,
            "JMP" => Opcode::JMP,
            "JZ" => Opcode::JZ,
            "JNZ" => Opcode::JNZ,
            "JC" => Opcode::JC,
            "JNC" => Opcode::JNC,
            "CALL" => Opcode::CALL,
            "RET" => Opcode::RET,
            "PUSH" => Opcode::PUSH,
            "POP" => Opcode::POP,
            "IN" => Opcode::IN,
            "OUT" => Opcode::OUT,
            "HLT" => Opcode::HLT,
            _ => return Err("Invalid opcode"),
        };
        
        let rd = if tokens.len() > 1 {
            self.parse_register(tokens[1])?
        } else {
            0
        };
        
        let rs1 = if tokens.len() > 2 {
            self.parse_register(tokens[2])?
        } else {
            0
        };
        
        let rs2 = if tokens.len() > 3 {
            self.parse_register(tokens[3])?
        } else {
            0
        };
        
        let immediate = if tokens.len() > 4 {
            self.parse_immediate(tokens[4])?
        } else {
            Word57::new(0)
        };
        
        Ok(Some(Instruction {
            opcode,
            rd,
            rs1,
            rs2,
            immediate,
        }))
    }
    
    fn parse_register(&self, token: &str) -> Result<u8, &'static str> {
        if !token.starts_with('R') && !token.starts_with('r') {
            return Err("Invalid register format");
        }
        
        let num_str = &token[1..];
        let num = self.parse_u8(num_str)?;
        
        if num >= 32 {
            return Err("Register number out of range");
        }
        
        Ok(num)
    }
    
    fn parse_immediate(&self, token: &str) -> Result<Word57, &'static str> {
        if token.starts_with("0x") || token.starts_with("0X") {
            let hex_str = &token[2..];
            let value = self.parse_hex(hex_str)?;
            Ok(Word57::new(value))
        } else {
            let value = self.parse_u64(token)?;
            Ok(Word57::new(value))
        }
    }
    
    fn parse_u8(&self, s: &str) -> Result<u8, &'static str> {
        let mut result = 0u8;
        for c in s.chars() {
            if !c.is_ascii_digit() {
                return Err("Invalid number");
            }
            let digit = (c as u8) - b'0';
            result = result.wrapping_mul(10).wrapping_add(digit);
        }
        Ok(result)
    }
    
    fn parse_u64(&self, s: &str) -> Result<u64, &'static str> {
        let mut result = 0u64;
        for c in s.chars() {
            if !c.is_ascii_digit() {
                return Err("Invalid number");
            }
            let digit = (c as u8 - b'0') as u64;
            result = result.wrapping_mul(10).wrapping_add(digit);
        }
        Ok(result)
    }
    
    fn parse_hex(&self, s: &str) -> Result<u64, &'static str> {
        let mut result = 0u64;
        for c in s.chars() {
            let digit = match c {
                '0'..='9' => (c as u8 - b'0') as u64,
                'a'..='f' => (c as u8 - b'a' + 10) as u64,
                'A'..='F' => (c as u8 - b'A' + 10) as u64,
                _ => return Err("Invalid hex digit"),
            };
            result = (result << 4) | digit;
        }
        Ok(result)
    }
    
    fn hash_label(&self, label: &str) -> u32 {
        let mut hash = 0u32;
        for byte in label.bytes() {
            hash = hash.wrapping_mul(31).wrapping_add(byte as u32);
        }
        hash
    }
    
    pub fn resolve_label(&self, label: &str) -> Option<u32> {
        let hash = self.hash_label(label);
        for i in 0..self.label_count {
            if self.labels[i].0 == hash {
                return Some(self.labels[i].1);
            }
        }
        None
    }
}

pub fn example_program() -> &'static str {
    r#"
; Test program for CPU-57
; Compute factorial of 5

start:
    LOADI R0 5          ; Load 5 into R0
    LOADI R1 1          ; Initialize result to 1
    
loop:
    CMP R0 R0 R0        ; Check if R0 is zero
    JZ end              ; Jump to end if zero
    
    MUL R1 R1 R0        ; Multiply result by R0
    LOADI R2 1          ; Load 1 for subtraction
    SUB R0 R0 R2        ; Decrement R0
    JMP loop            ; Continue loop
    
end:
    STORE R1 R0 R0      ; Store result at address 0
    HLT                 ; Halt execution
"#
}