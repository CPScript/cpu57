#![no_std]
#![no_main]

mod cpu57_core;
#[cfg(not(target_os = "none"))]
mod hdl_bridge;
mod framebuffer;
mod assembler;
mod multiboot;

extern crate alloc;

use core::panic::PanicInfo;
use alloc::vec::Vec;
use cpu57_core::{CPU57, Instruction, Opcode, Word57};

#[cfg(not(target_os = "none"))]
use hdl_bridge::CPU57HDL;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        unsafe { core::arch::asm!("hlt") };
    }
}

#[no_mangle]
pub extern "C" fn rust_alloc(size: usize) -> *mut u8 {
    use core::alloc::{GlobalAlloc, Layout};
    unsafe {
        let layout = Layout::from_size_align_unchecked(size, 8);
        ALLOCATOR.alloc(layout)
    }
}

#[no_mangle]
pub extern "C" fn rust_dealloc(ptr: *mut u8, size: usize) {
    use core::alloc::{GlobalAlloc, Layout};
    unsafe {
        let layout = Layout::from_size_align_unchecked(size, 8);
        ALLOCATOR.dealloc(ptr, layout);
    }
}

static mut HEAP: [u8; 16 * 1024 * 1024] = [0; 16 * 1024 * 1024];

#[global_allocator]
static ALLOCATOR: BumpAllocator = BumpAllocator::new();

struct BumpAllocator {
    next: core::sync::atomic::AtomicUsize,
}

impl BumpAllocator {
    const fn new() -> Self {
        BumpAllocator {
            next: core::sync::atomic::AtomicUsize::new(0),
        }
    }
    
    fn heap_start(&self) -> usize {
        core::ptr::addr_of!(HEAP) as usize
    }
    
    fn heap_size(&self) -> usize {
        16 * 1024 * 1024
    }
}

unsafe impl core::alloc::GlobalAlloc for BumpAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        let size = layout.size();
        let align = layout.align();
        
        let heap_start = self.heap_start();
        let heap_end = heap_start + self.heap_size();
        
        let mut current = self.next.load(core::sync::atomic::Ordering::Relaxed);
        loop {
            let alloc_start = heap_start + current;
            let aligned = (alloc_start + align - 1) & !(align - 1);
            let alloc_end = aligned + size;
            
            if alloc_end > heap_end {
                return core::ptr::null_mut();
            }
            
            let new_current = current + (alloc_end - alloc_start);
            
            match self.next.compare_exchange_weak(
                current,
                new_current,
                core::sync::atomic::Ordering::Relaxed,
                core::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return aligned as *mut u8,
                Err(x) => current = x,
            }
        }
    }
    
    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: core::alloc::Layout) {
    }
}

#[repr(align(4096))]
struct PageTable {
    entries: [u64; 512],
}

#[link_section = ".bss.page_tables"]
#[no_mangle]
static mut BOOT_PML4: PageTable = PageTable { entries: [0; 512] };

#[link_section = ".bss.page_tables"]
#[no_mangle]
static mut BOOT_PDPT: PageTable = PageTable { entries: [0; 512] };

#[link_section = ".bss.page_tables"]
#[no_mangle]
static mut BOOT_PD: PageTable = PageTable { entries: [0; 512] };

#[link_section = ".bss.page_tables"]
#[no_mangle]
static mut BOOT_PT: PageTable = PageTable { entries: [0; 512] };

#[repr(align(16))]
struct Stack {
    data: [u8; 65536],
}

#[link_section = ".bss.stack"]
#[no_mangle]
static mut BOOT_STACK: Stack = Stack { data: [0; 65536] };

#[repr(C, align(16))]
struct GDT {
    null: u64,
    code: u64,
    data: u64,
}

#[link_section = ".rodata.gdt"]
#[no_mangle]
static BOOT_GDT: GDT = GDT {
    null: 0,
    code: 0x00AF9A000000FFFF,
    data: 0x00CF92000000FFFF,
};

#[repr(C, packed)]
struct GDTPointer {
    limit: u16,
    base: u64,
}

#[link_section = ".data.gdt_ptr"]
#[no_mangle]
static mut BOOT_GDT_PTR: GDTPointer = GDTPointer {
    limit: 23,
    base: 0,
};

#[unsafe(naked)]
#[no_mangle]
#[link_section = ".text._start"]
pub unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        ".code32",
        
        "cli",
        "cld",
        
        "mov esp, offset {stack}",
        "add esp, 65536",
        
        "mov edi, eax",
        "mov esi, ebx",
        
        "xor eax, eax",
        "mov edi, offset {pml4}",
        "mov ecx, 2048",
        "rep stosd",
        
        "mov edi, offset {pml4}",
        "lea eax, [{pdpt}]",
        "or eax, 3",
        "mov [edi], eax",
        
        "mov edi, offset {pdpt}",
        "lea eax, [{pd}]",
        "or eax, 3",
        "mov [edi], eax",
        
        "mov edi, offset {pd}",
        "mov eax, 0x83",
        "mov ecx, 512",
        "1:",
        "mov [edi], eax",
        "add eax, 0x200000",
        "add edi, 8",
        "loop 1b",
        
        "lea eax, [{pml4}]",
        "mov cr3, eax",
        
        "mov eax, cr4",
        "or eax, 0x20",
        "mov cr4, eax",
        
        "mov ecx, 0xC0000080",
        "rdmsr",
        "or eax, 0x100",
        "wrmsr",
        
        "mov eax, cr0",
        "or eax, 0x80000001",
        "mov cr0, eax",
        
        "lea eax, [{gdt}]",
        "mov dword ptr [{gdt_ptr} + 2], eax",
        "xor eax, eax",
        "mov dword ptr [{gdt_ptr} + 6], eax",
        "lgdt [{gdt_ptr}]",
        
        "push 0x08",
        "lea eax, [2f]",
        "push eax",
        "retf",
        
        ".code64",
        "2:",
        
        "mov ax, 0x10",
        "mov ds, ax",
        "mov es, ax",
        "mov ss, ax",
        "xor ax, ax",
        "mov fs, ax",
        "mov gs, ax",
        
        "lea rsp, [{stack}]",
        "add rsp, 65536",
        "and rsp, -16",
        
        "xor rax, rax",
        "xor rbx, rbx",
        "xor rcx, rcx",
        "xor rdx, rdx",
        "xor rsi, rsi",
        "xor rdi, rdi",
        "xor rbp, rbp",
        "xor r8, r8",
        "xor r9, r9",
        "xor r10, r10",
        "xor r11, r11",
        "xor r12, r12",
        "xor r13, r13",
        "xor r14, r14",
        "xor r15, r15",
        
        "call {main}",
        
        "3:",
        "hlt",
        "jmp 3b",
        
        stack = sym BOOT_STACK,
        pml4 = sym BOOT_PML4,
        pdpt = sym BOOT_PDPT,
        pd = sym BOOT_PD,
        gdt = sym BOOT_GDT,
        gdt_ptr = sym BOOT_GDT_PTR,
        main = sym rust_main,
    )
}

// VGA text mode helper
unsafe fn clear_vga_text() {
    let vga_text = 0xB8000 as *mut u16;
    for i in 0..(80 * 25) {
        core::ptr::write_volatile(vga_text.add(i), 0x0000);
    }
}

unsafe fn draw_text_ui(cpu: &CPU57) {
    let vga_text = 0xB8000 as *mut u16;
    
    let title = b"CPU-57 MONITOR [RUST]";
    for i in 0..80 {
        if i < title.len() {
            core::ptr::write_volatile(vga_text.add(i), 0x1B00 | title[i] as u16);
        } else {
            core::ptr::write_volatile(vga_text.add(i), 0x1B00 | b' ' as u16);
        }
    }
    
    let cycle_label = b"CYCLE:";
    for (i, &byte) in cycle_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(50 + i), 0x1F00 | byte as u16);
    }
    draw_hex_at(vga_text, 57, 0, cpu.cycle_count, 12, 0x1E);
    
    let status = if cpu.halted { b"HALTED " } else { b"RUNNING" };
    for (i, &byte) in status.iter().enumerate() {
        let color = if cpu.halted { 0x1C } else { 0x1A };
        core::ptr::write_volatile(vga_text.add(70 + i), (color << 8) | byte as u16);
    }
    
    let reg_label = b"REGISTERS:";
    for (i, &byte) in reg_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(160 + i), 0x0E00 | byte as u16);
    }
    
    for i in 0..16 {
        let row = 3 + (i / 4);
        let col = (i % 4) * 20;
        let pos = row * 80 + col;
        
        let reg_name = format_register(i);
        for (j, &byte) in reg_name.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(pos + j), 0x0A00 | byte as u16);
        }
        
        draw_hex_at(vga_text, pos + 4, 0, cpu.registers[i].get(), 14, 0x0B);
    }
    
    let pc_label = b"PC:";
    for (i, &byte) in pc_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(560 + i), 0x0C00 | byte as u16);
    }
    draw_hex_at(vga_text, 564, 0, cpu.pc.get(), 14, 0x0B);
    
    let sp_label = b"SP:";
    for (i, &byte) in sp_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(600 + i), 0x0C00 | byte as u16);
    }
    draw_hex_at(vga_text, 604, 0, cpu.sp.get(), 14, 0x0B);
    
    let inst_label = b"CURRENT INSTRUCTION:";
    for (i, &byte) in inst_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(720 + i), 0x0E00 | byte as u16);
    }
    
    if let Some(inst) = cpu.current_instruction {
        let opcode_name = opcode_to_str(inst.opcode);
        for (i, &byte) in opcode_name.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(800 + i), 0x0D00 | byte as u16);
        }
        
        let rd_label = b"RD:";
        for (i, &byte) in rd_label.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(810 + i), 0x0A00 | byte as u16);
        }
        draw_hex_at(vga_text, 814, 0, inst.rd as u64, 2, 0x0B);
        
        let rs1_label = b"RS1:";
        for (i, &byte) in rs1_label.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(820 + i), 0x0A00 | byte as u16);
        }
        draw_hex_at(vga_text, 825, 0, inst.rs1 as u64, 2, 0x0B);
        
        let rs2_label = b"RS2:";
        for (i, &byte) in rs2_label.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(830 + i), 0x0A00 | byte as u16);
        }
        draw_hex_at(vga_text, 835, 0, inst.rs2 as u64, 2, 0x0B);
        
        let imm_label = b"IMM:";
        for (i, &byte) in imm_label.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(880 + i), 0x0A00 | byte as u16);
        }
        draw_hex_at(vga_text, 885, 0, inst.immediate.get(), 14, 0x0B);
    }
    
    let flag_label = b"FLAGS:";
    for (i, &byte) in flag_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(960 + i), 0x0E00 | byte as u16);
    }
    
    let z_color = if cpu.flags.zero { 0x0A } else { 0x08 };
    let c_color = if cpu.flags.carry { 0x0A } else { 0x08 };
    let o_color = if cpu.flags.overflow { 0x0A } else { 0x08 };
    let n_color = if cpu.flags.negative { 0x0A } else { 0x08 };
    
    core::ptr::write_volatile(vga_text.add(970), (z_color << 8) | b'Z' as u16);
    core::ptr::write_volatile(vga_text.add(972), (c_color << 8) | b'C' as u16);
    core::ptr::write_volatile(vga_text.add(974), (o_color << 8) | b'O' as u16);
    core::ptr::write_volatile(vga_text.add(976), (n_color << 8) | b'N' as u16);
    
    let bus_label = b"BUS ACTIVITY:";
    for (i, &byte) in bus_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1120 + i), 0x0E00 | byte as u16);
    }
    
    let active_color = if cpu.bus_active { 0x0A } else { 0x08 };
    let active_text = if cpu.bus_active { b"ACTIVE  " } else { b"INACTIVE" };
    for (i, &byte) in active_text.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1135 + i), (active_color << 8) | byte as u16);
    }
    
    let addr_label = b"ADDR:";
    for (i, &byte) in addr_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1200 + i), 0x0A00 | byte as u16);
    }
    draw_hex_at(vga_text, 1206, 0, cpu.address_bus.get(), 14, 0x0C);
    
    let data_label = b"DATA:";
    for (i, &byte) in data_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1240 + i), 0x0A00 | byte as u16);
    }
    draw_hex_at(vga_text, 1246, 0, cpu.data_bus.get(), 14, 0x0C);
    
    let mem_label = b"MEMORY (at PC):";
    for (i, &byte) in mem_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1360 + i), 0x0E00 | byte as u16);
    }
    
    let addr = cpu.pc.get() as usize;
    for i in 0..4 {
        let row = 18 + i;
        let mem_addr = (addr + i * 16).min(cpu.memory.len() - 1);
        let pos = row * 80;
        
        draw_hex_at(vga_text, pos, 0, mem_addr as u64, 8, 0x0A);
        
        for j in 0..16 {
            let byte_addr = mem_addr + j;
            if byte_addr < cpu.memory.len() {
                let byte = cpu.memory[byte_addr];
                draw_hex_at(vga_text, pos + 10 + j * 3, 0, byte as u64, 2, 0x0B);
            }
        }
    }
    
    let io_label = b"I/O PORTS (0-7):";
    for (i, &byte) in io_label.iter().enumerate() {
        core::ptr::write_volatile(vga_text.add(1760 + i), 0x0E00 | byte as u16);
    }
    
    for i in 0..8 {
        let pos = 1840 + i * 10;
        draw_hex_at(vga_text, pos, 0, i as u64, 1, 0x0A);
        core::ptr::write_volatile(vga_text.add(pos + 2), 0x0F00 | b':' as u16);
        draw_hex_at(vga_text, pos + 3, 0, cpu.io_ports[i].get(), 4, 0x0B);
    }
}

unsafe fn draw_hex_at(vga_text: *mut u16, pos: usize, _row: usize, value: u64, width: usize, color: u8) {
    let hex_chars = b"0123456789ABCDEF";
    for i in 0..width {
        let nibble = ((value >> ((width - 1 - i) * 4)) & 0xF) as usize;
        core::ptr::write_volatile(
            vga_text.add(pos + i),
            ((color as u16) << 8) | hex_chars[nibble] as u16
        );
    }
}

fn format_register(i: usize) -> [u8; 4] {
    let mut result = [b' '; 4];
    result[0] = b'R';
    if i < 10 {
        result[1] = b'0';
        result[2] = b'0' + i as u8;
    } else {
        result[1] = b'0' + (i / 10) as u8;
        result[2] = b'0' + (i % 10) as u8;
    }
    result[3] = b':';
    result
}

fn opcode_to_str(opcode: Opcode) -> &'static [u8] {
    match opcode {
        Opcode::NOP => b"NOP   ",
        Opcode::ADD => b"ADD   ",
        Opcode::SUB => b"SUB   ",
        Opcode::MUL => b"MUL   ",
        Opcode::DIV => b"DIV   ",
        Opcode::AND => b"AND   ",
        Opcode::OR => b"OR    ",
        Opcode::XOR => b"XOR   ",
        Opcode::NOT => b"NOT   ",
        Opcode::SHL => b"SHL   ",
        Opcode::SHR => b"SHR   ",
        Opcode::ROL => b"ROL   ",
        Opcode::ROR => b"ROR   ",
        Opcode::LOAD => b"LOAD  ",
        Opcode::STORE => b"STORE ",
        Opcode::LOADI => b"LOADI ",
        Opcode::MOV => b"MOV   ",
        Opcode::CMP => b"CMP   ",
        Opcode::JMP => b"JMP   ",
        Opcode::JZ => b"JZ    ",
        Opcode::JNZ => b"JNZ   ",
        Opcode::JC => b"JC    ",
        Opcode::JNC => b"JNC   ",
        Opcode::CALL => b"CALL  ",
        Opcode::RET => b"RET   ",
        Opcode::PUSH => b"PUSH  ",
        Opcode::POP => b"POP   ",
        Opcode::IN => b"IN    ",
        Opcode::OUT => b"OUT   ",
        Opcode::HLT => b"HLT   ",
    }
}

#[no_mangle]
extern "C" fn rust_main() -> ! {
    unsafe {
        clear_vga_text();
        
        let vga_text = 0xB8000 as *mut u16;
        let msg = b"Initializing CPU-57 (Pure Rust)";
        for (i, &byte) in msg.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(i), 0x0F00 | byte as u16);
        }
    }
    
    // Use pure Rust CPU for bare metal. HDL supposrt is not implemented fully yet for bare metal, but works.
    let mut cpu = CPU57::new();
    
    unsafe {
        let vga_text = 0xB8000 as *mut u16;
        let msg = b"Loading program...              ";
        for (i, &byte) in msg.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(i), 0x0A00 | byte as u16);
        }
    }
    
    let program = create_test_program();
    cpu.load_program(&program, 0);
    
    unsafe {
        clear_vga_text();
    }
    
    let mut _instruction_counter = 0u64;
    
    loop {
        if !cpu.halted {
            cpu.step();
            _instruction_counter += 1;
        }
        
        unsafe {
            draw_text_ui(&cpu);
        }
        
        delay(200000);
        
        if cpu.halted {
            delay(2000000);
        }
    }
}

fn create_test_program() -> Vec<u8> {
    let mut program = Vec::new();
    
    let loadi_r0 = Instruction {
        opcode: Opcode::LOADI,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(100),
    };
    program.extend_from_slice(&loadi_r0.encode());
    
    let loadi_r1 = Instruction {
        opcode: Opcode::LOADI,
        rd: 1,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(5),
    };
    program.extend_from_slice(&loadi_r1.encode());
    
    let sub_r0 = Instruction {
        opcode: Opcode::SUB,
        rd: 0,
        rs1: 0,
        rs2: 1,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&sub_r0.encode());
    
    let loadi_r3 = Instruction {
        opcode: Opcode::LOADI,
        rd: 3,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(2),
    };
    program.extend_from_slice(&loadi_r3.encode());
    
    let mul_r2 = Instruction {
        opcode: Opcode::MUL,
        rd: 2,
        rs1: 0,
        rs2: 3,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&mul_r2.encode());
    
    let loadi_r4 = Instruction {
        opcode: Opcode::LOADI,
        rd: 4,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(1000),
    };
    program.extend_from_slice(&loadi_r4.encode());
    
    let store = Instruction {
        opcode: Opcode::STORE,
        rd: 2,
        rs1: 4,
        rs2: 0,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&store.encode());
    
    let out_inst = Instruction {
        opcode: Opcode::OUT,
        rd: 2,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(5),
    };
    program.extend_from_slice(&out_inst.encode());
    
    let cmp = Instruction {
        opcode: Opcode::CMP,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&cmp.encode());
    
    let jz = Instruction {
        opcode: Opcode::JZ,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(100),
    };
    program.extend_from_slice(&jz.encode());
    
    let jmp = Instruction {
        opcode: Opcode::JMP,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(20),
    };
    program.extend_from_slice(&jmp.encode());
    
    let hlt = Instruction {
        opcode: Opcode::HLT,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&hlt.encode());
    
    program
}

fn delay(count: u32) {
    for _ in 0..count {
        unsafe {
            core::arch::asm!("nop", options(nomem, nostack));
        }
    }
}