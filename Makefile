VERILATOR = verilator
VERILATOR_FLAGS = --cc --build --public -Wall -Wno-fatal \
	-Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-VARHIDDEN -Wno-EOFNEWLINE
VERILATOR_ROOT = $(shell verilator --getenv VERILATOR_ROOT)

HDL_SOURCES = hdl/cpu57.v
TB_SOURCES = hdl/cpu57_tb.v

TARGET = x86_64-unknown-none
BUILD_MODE = release
KERNEL_BIN = target/$(TARGET)/$(BUILD_MODE)/cpu57
OUTPUT_DIR = target
ISO_DIR = $(OUTPUT_DIR)/iso
ISO_FILE = $(OUTPUT_DIR)/cpu57.iso

.PHONY: all build_kernel iso run clean test_hdl

all: iso

test_hdl: $(HDL_SOURCES) $(TB_SOURCES)
	bash build_hdl.sh

build_kernel:
	cargo build --release --target $(TARGET)

iso: build_kernel
	mkdir -p $(ISO_DIR)/boot/grub
	cp $(KERNEL_BIN) $(ISO_DIR)/boot/kernel.bin
	echo 'set timeout=0' > $(ISO_DIR)/boot/grub/grub.cfg
	echo 'set default=0' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo 'menuentry "CPU-57 HDL" {' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '    multiboot /boot/kernel.bin' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '    boot' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '}' >> $(ISO_DIR)/boot/grub/grub.cfg
	grub-mkrescue -o $(ISO_FILE) $(ISO_DIR)

run: iso
	qemu-system-x86_64 -cdrom $(ISO_FILE) \
		-serial stdio \
		-m 512M \
		-display gtk \
		-vga std \
		-device bochs-display

clean:
	cargo clean
	rm -rf hdl/*.vcd
	rm -rf $(OUTPUT_DIR)/iso
	rm -f $(ISO_FILE)