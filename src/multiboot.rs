const MULTIBOOT1_MAGIC: u32 = 0x1BADB002;
const MULTIBOOT1_FLAGS: u32 = 0x00000003;
const MULTIBOOT1_CHECKSUM: u32 = 0u32.wrapping_sub(MULTIBOOT1_MAGIC + MULTIBOOT1_FLAGS);

#[repr(C, align(4))]
struct Multiboot1Header {
    magic: u32,
    flags: u32,
    checksum: u32,
}

#[link_section = ".multiboot"]
#[no_mangle]
#[used]
pub static MULTIBOOT1_HEADER: Multiboot1Header = Multiboot1Header {
    magic: MULTIBOOT1_MAGIC,
    flags: MULTIBOOT1_FLAGS,
    checksum: MULTIBOOT1_CHECKSUM,
};