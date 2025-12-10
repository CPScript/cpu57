#![no_std]
#![no_main]

mod cpu57_core;
mod framebuffer;
mod assembler;
mod multiboot;

extern crate alloc;

use core::panic::PanicInfo;
use alloc::vec::Vec;
use cpu57_core::{CPU57, Instruction, Opcode, Word57};
use framebuffer::{CPUUI, Framebuffer, VGA_HEIGHT, VGA_WIDTH};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        unsafe { core::arch::asm!("hlt") };
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
        
        "mov edi, eax",  // save magic
        "mov esi, ebx",  // Save info pointer
        
        "xor eax, eax",
        "mov edi, offset {pml4}",
        "mov ecx, 2048",
        "rep stosd",
        
        // PML4[0] -> PDPT
        "mov edi, offset {pml4}",
        "lea eax, [{pdpt}]",
        "or eax, 3",
        "mov [edi], eax",
        
        // PDPT[0] -> PD
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
        
        // load CR3
        "lea eax, [{pml4}]",
        "mov cr3, eax",
        
        // enable PAE
        "mov eax, cr4",
        "or eax, 0x20",
        "mov cr4, eax",
        
        // long mode in EFER MSR
        "mov ecx, 0xC0000080",
        "rdmsr",
        "or eax, 0x100",
        "wrmsr",
        
        "mov eax, cr0",
        "or eax, 0x80000001",
        "mov cr0, eax",
        
        // Setup GDT pointer
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
        
        // clear registers
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
        
        // Halt
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

#[no_mangle]
extern "C" fn rust_main() -> ! {
    // Simple test.
    let _fb_ptr = 0xB8000 as *mut u32;
    
    unsafe {
        let vga_text = 0xB8000 as *mut u16;
        let msg = b"CPU-57 BOOT";
        for (i, &byte) in msg.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(i), 0x0F00 | byte as u16);
        }
    }
    
    // setup framebuffer properly, but framebuffer has been disabled for now during testing
    // For QEMU framebuffer is typically at 0xE0000000 or 0xFD000000
    
    // Try to detect framebuffer
    let _fb_addr = 0xFD000000usize;
    
    // For safety, lets do a simple CPU test without graphics
    let mut cpu = CPU57::new();
    
    let program = create_test_program();
    cpu.load_program(&program, 0);
    
    // Run CPU for a bit
    cpu.run(100);
    
    // Display results
    unsafe {
        let vga_text = 0xB8000 as *mut u16;
        let msg = b"R0:";
        for (i, &byte) in msg.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(80 + i), 0x0A00 | byte as u16);
        }
        
        // R0 value
        let r0_val = cpu.registers[0].get();
        let hex_chars = b"0123456789ABCDEF";
        for i in 0..16 {
            let nibble = ((r0_val >> ((15 - i) * 4)) & 0xF) as usize;
            core::ptr::write_volatile(
                vga_text.add(80 + 4 + i), 
                0x0E00 | hex_chars[nibble] as u16
            );
        }
        
        // display R2
        let msg2 = b"R2:";
        for (i, &byte) in msg2.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(160 + i), 0x0A00 | byte as u16);
        }
        
        let r2_val = cpu.registers[2].get();
        for i in 0..16 {
            let nibble = ((r2_val >> ((15 - i) * 4)) & 0xF) as usize;
            core::ptr::write_volatile(
                vga_text.add(160 + 4 + i), 
                0x0E00 | hex_chars[nibble] as u16
            );
        }
        
        // Display cycle count
        let msg3 = b"CYCLES:";
        for (i, &byte) in msg3.iter().enumerate() {
            core::ptr::write_volatile(vga_text.add(240 + i), 0x0A00 | byte as u16);
        }
        
        let cycles = cpu.cycle_count;
        for i in 0..16 {
            let nibble = ((cycles >> ((15 - i) * 4)) & 0xF) as usize;
            core::ptr::write_volatile(
                vga_text.add(240 + 8 + i), 
                0x0E00 | hex_chars[nibble] as u16
            );
        }
        
        let status_msg = if cpu.halted { b"HALTED  " } else { b"RUNNING " };
        for (i, &byte) in status_msg.iter().enumerate() {
            core::ptr::write_volatile(
                vga_text.add(320 + i), 
                0x0C00 | byte as u16
            );
        }
    }
    
    loop {
        unsafe { core::arch::asm!("hlt") };
    }
}

fn create_test_program() -> Vec<u8> {
    let mut program = Vec::new();
    
    // load 42 into R0
    let loadi_r0 = Instruction {
        opcode: Opcode::LOADI,
        rd: 0,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(42),
    };
    program.extend_from_slice(&loadi_r0.encode());
    
    // load 58 into R1
    let loadi_r1 = Instruction {
        opcode: Opcode::LOADI,
        rd: 1,
        rs1: 0,
        rs2: 0,
        immediate: Word57::new(58),
    };
    program.extend_from_slice(&loadi_r1.encode());
    
    let add_r2 = Instruction {
        opcode: Opcode::ADD,
        rd: 2,
        rs1: 0,
        rs2: 1,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&add_r2.encode());
    
    let mul_r3 = Instruction {
        opcode: Opcode::MUL,
        rd: 3,
        rs1: 0,
        rs2: 1,
        immediate: Word57::new(0),
    };
    program.extend_from_slice(&mul_r3.encode());
    
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

fn delay(_count: u32) {
    for _ in 0.._count {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}