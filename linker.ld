ENTRY(_start)

OUTPUT_FORMAT(elf64-x86-64)
OUTPUT_ARCH(i386:x86-64)

SECTIONS
{
    . = 0x100000;
    
    _kernel_start = .;
    
    .multiboot ALIGN(8) : {
        KEEP(*(.multiboot))
    }
    
    .text ALIGN(16) : {
        KEEP(*(.text._start))
        *(.text*)
    }

    .rodata ALIGN(4K) : {
        *(.rodata*)
    }

    .data ALIGN(4K) : {
        *(.data*)
    }

    .bss ALIGN(4K) : {
        *(COMMON)
        *(.bss*)
    }
    
    _kernel_end = .;

    /DISCARD/ : {
        *(.eh_frame)
        *(.note*)
        *(.comment)
    }
}