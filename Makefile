TARGET = x86_64-unknown-none
BUILD_MODE = release
KERNEL_BIN = target/$(TARGET)/$(BUILD_MODE)/cpu57
OUTPUT_DIR = target
ISO_DIR = $(OUTPUT_DIR)/iso
ISO_FILE = $(OUTPUT_DIR)/cpu57.iso

.PHONY: all build iso run clean

all: iso

build:
	cargo build --release --target $(TARGET)

iso: build
	mkdir -p $(ISO_DIR)/boot/grub
	cp $(KERNEL_BIN) $(ISO_DIR)/boot/kernel.bin
	echo 'set timeout=0' > $(ISO_DIR)/boot/grub/grub.cfg
	echo 'set default=0' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo 'menuentry "CPU-57" {' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '    multiboot /boot/kernel.bin' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '    boot' >> $(ISO_DIR)/boot/grub/grub.cfg
	echo '}' >> $(ISO_DIR)/boot/grub/grub.cfg
	grub-mkrescue -o $(ISO_FILE) $(ISO_DIR)

run: iso
	qemu-system-x86_64 -cdrom $(ISO_FILE) \
		-serial stdio \
		-m 512M \
		-device VGA,vgamem_mb=32 \
		-d int,cpu_reset


debug: iso
	qemu-system-x86_64 -cdrom $(ISO_FILE) \
		-serial stdio \
		-m 512M \
		-device VGA,vgamem_mb=32 \
		-s -S

clean:
	cargo clean
	rm -rf $(OUTPUT_DIR)/iso
	rm -f $(ISO_FILE)