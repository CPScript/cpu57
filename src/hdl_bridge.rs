use crate::cpu57_core::{Instruction, Opcode, Word57, StatusFlags, REGISTER_COUNT, MEMORY_SIZE};
extern crate alloc;
use alloc::vec::Vec;

extern "C" {
    fn hdl_cpu_init() -> *mut HDLCPUState;
    fn hdl_cpu_reset(state: *mut HDLCPUState);
    fn hdl_cpu_clock(state: *mut HDLCPUState);
    fn hdl_cpu_load_memory(state: *mut HDLCPUState, addr: u64, data: *const u8, len: usize);
    fn hdl_cpu_read_register(state: *const HDLCPUState, reg: u8) -> u64;
    fn hdl_cpu_read_pc(state: *const HDLCPUState) -> u64;
    fn hdl_cpu_read_sp(state: *const HDLCPUState) -> u64;
    fn hdl_cpu_read_flags(state: *const HDLCPUState) -> u8;
    fn hdl_cpu_is_halted(state: *const HDLCPUState) -> bool;
    fn hdl_cpu_get_cycle_count(state: *const HDLCPUState) -> u64;
    fn hdl_cpu_read_memory(state: *const HDLCPUState, addr: u64, data: *mut u8, len: usize);
    fn hdl_cpu_read_io_port(state: *const HDLCPUState, port: u8) -> u64;
    fn hdl_cpu_get_bus_state(state: *const HDLCPUState, addr_out: *mut u64, data_out: *mut u64, active_out: *mut bool);
    fn hdl_cpu_destroy(state: *mut HDLCPUState);
}

#[repr(C)]
struct HDLCPUState {
    _opaque: [u8; 0],
}

pub struct CPU57HDL {
    hdl_state: *mut HDLCPUState,
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

impl CPU57HDL {
    pub fn new() -> Self {
        let hdl_state = unsafe { hdl_cpu_init() };
        
        CPU57HDL {
            hdl_state,
            registers: [Word57::new(0); REGISTER_COUNT],
            pc: Word57::new(0),
            sp: Word57::new(MEMORY_SIZE as u64 - 8),
            flags: StatusFlags::new(),
            memory: alloc::vec![0; MEMORY_SIZE],
            io_ports: [Word57::new(0); 256],
            cycle_count: 0,
            halted: false,
            current_instruction: None,
            data_bus: Word57::new(0),
            address_bus: Word57::new(0),
            bus_active: false,
        }
    }
    
    pub fn reset(&mut self) {
        unsafe {
            hdl_cpu_reset(self.hdl_state);
        }
        
        self.registers = [Word57::new(0); REGISTER_COUNT];
        self.pc = Word57::new(0);
        self.sp = Word57::new(MEMORY_SIZE as u64 - 8);
        self.flags = StatusFlags::new();
        self.cycle_count = 0;
        self.halted = false;
        self.current_instruction = None;
        self.data_bus = Word57::new(0);
        self.address_bus = Word57::new(0);
        self.bus_active = false;
    }
    
    pub fn load_program(&mut self, program: &[u8], offset: usize) {
        let len = program.len().min(MEMORY_SIZE - offset);
        self.memory[offset..offset + len].copy_from_slice(&program[..len]);
        
        unsafe {
            hdl_cpu_load_memory(
                self.hdl_state,
                offset as u64,
                program.as_ptr(),
                len
            );
        }
    }
    
    pub fn step(&mut self) {
        if self.halted {
            return;
        }
        
        unsafe {
            hdl_cpu_clock(self.hdl_state);
        }
        
        self.sync_state();
    }
    
    pub fn run(&mut self, max_cycles: u64) {
        while !self.halted && self.cycle_count < max_cycles {
            self.step();
        }
    }
    
    fn sync_state(&mut self) {
        unsafe {
            for i in 0..REGISTER_COUNT {
                let value = hdl_cpu_read_register(self.hdl_state, i as u8);
                self.registers[i] = Word57::new(value);
            }
            
            self.pc = Word57::new(hdl_cpu_read_pc(self.hdl_state));
            self.sp = Word57::new(hdl_cpu_read_sp(self.hdl_state));
            
            let flags = hdl_cpu_read_flags(self.hdl_state);
            self.flags.zero = (flags & 0x01) != 0;
            self.flags.carry = (flags & 0x02) != 0;
            self.flags.overflow = (flags & 0x04) != 0;
            self.flags.negative = (flags & 0x08) != 0;
            
            self.halted = hdl_cpu_is_halted(self.hdl_state);
            self.cycle_count = hdl_cpu_get_cycle_count(self.hdl_state);
            
            hdl_cpu_read_memory(
                self.hdl_state,
                0,
                self.memory.as_mut_ptr(),
                MEMORY_SIZE
            );
            
            for i in 0..256 {
                let value = hdl_cpu_read_io_port(self.hdl_state, i as u8);
                self.io_ports[i] = Word57::new(value);
            }
            
            let mut addr: u64 = 0;
            let mut data: u64 = 0;
            let mut active: bool = false;
            
            hdl_cpu_get_bus_state(
                self.hdl_state,
                &mut addr,
                &mut data,
                &mut active
            );
            
            self.address_bus = Word57::new(addr);
            self.data_bus = Word57::new(data);
            self.bus_active = active;
            
            let pc_addr = self.pc.get() as usize;
            if pc_addr + 10 <= MEMORY_SIZE {
                self.current_instruction = Instruction::decode(&self.memory[pc_addr..pc_addr + 10]);
            }
        }
    }
    
    pub fn read_memory(&self, addr: Word57) -> Word57 {
        let addr = (addr.get() as usize) % MEMORY_SIZE;
        let mut bytes = [0u8; 8];
        let end = (addr + 7).min(MEMORY_SIZE - 1);
        let len = end - addr + 1;
        bytes[0..len].copy_from_slice(&self.memory[addr..=end]);
        Word57::new(u64::from_le_bytes(bytes))
    }
    
    pub fn write_memory(&mut self, addr: Word57, value: Word57) {
        let addr = (addr.get() as usize) % MEMORY_SIZE;
        let bytes = value.get().to_le_bytes();
        let end = (addr + 7).min(MEMORY_SIZE - 1);
        let len = end - addr + 1;
        self.memory[addr..=end].copy_from_slice(&bytes[0..len]);
    }
}

impl Drop for CPU57HDL {
    fn drop(&mut self) {
        unsafe {
            hdl_cpu_destroy(self.hdl_state);
        }
    }
}