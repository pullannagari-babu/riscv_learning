# RISC-V VisionFive2 Deep Boot Flow Analysis

**Target:** StarFive JH7110 SoC  
**Architecture:** RISC-V RV64GC  
**Boot Flow:** Hardware Reset → SPL → OpenSBI → U-Boot → Kernel → Init  

---

## 📋 Table of Contents

1. [RISC-V Boot Flow Overview](#1-risc-v-boot-flow-overview)
2. [Hardware Reset to SPL](#2-hardware-reset-to-spl)
3. [SPL Internal Operations](#3-spl-internal-operations)
4. [SPL to OpenSBI Handoff](#4-spl-to-opensbi-handoff)
5. [OpenSBI Internal Operations](#5-opensbi-internal-operations)
6. [OpenSBI to U-Boot Handoff](#6-opensbi-to-u-boot-handoff)
7. [U-Boot Internal Operations](#7-u-boot-internal-operations)
8. [U-Boot to Kernel Handoff](#8-u-boot-to-kernel-handoff)
9. [RISC-V Assembly Language Primer](#9-risc-v-assembly-language-primer)
10. [Register Usage and Calling Conventions](#10-register-usage-and-calling-conventions)

---

## 1. RISC-V Boot Flow Overview

### 1.1 Complete Boot Chain
```
Hardware Reset (0x1000)
    ↓ [RISC-V Hart 0 starts here]
ROM Bootloader (Built-in)
    ↓ [Loads from SPI Flash offset 0x0]
U-Boot SPL (0x08000000)
    ↓ [Initializes DDR, loads next stage]
OpenSBI + U-Boot (0x40000000)
    ↓ [Machine mode firmware]
U-Boot Proper (Supervisor mode)
    ↓ [Rich bootloader environment]
Linux Kernel (0x44000000)
    ↓ [Operating system]
Init Process (/sbin/init)
    ↓ [User space]
```

### 1.2 Memory Layout During Boot
|**Address_Range**| **Component** | **Size** | **Mode** | **Purpose** |
|--------------|---------------|----------|----------|-------------|
| `0x1000`     | ROM Code | 32KB | M-Mode | Built-in bootloader |
| `0x08000000` | SPL | 128KB | M-Mode | First stage loader |
| `0x40000000` | OpenSBI | 1MB | M-Mode | SBI runtime services |
| `0x40100000` | U-Boot | 2MB | S-Mode | Rich bootloader |
| `0x44000000` | Kernel | 20MB | S-Mode | Linux kernel |
| `0x46000000` | DTB | 64KB | - | Device tree |
| `0x47000000` | Initramfs | Variable | - | Initial filesystem |

---

## 2. Hardware Reset to SPL

### 2.1 Hardware Reset Vector
When the JH7110 SoC powers on, all harts (hardware threads) start at the reset vector:

```assembly
# Reset vector at address 0x1000 (ROM)
# This is built into the JH7110 SoC

.section .text.reset
.global _start
_start:
    # Hart 0 continues, others wait
    csrr    a0, mhartid         # Read hardware thread ID
    bnez    a0, wait_for_ipi    # If not hart 0, wait
    
    # Initialize basic CSRs
    li      t0, 0x1800          # Machine mode, interrupts disabled
    csrw    mstatus, t0         # Set machine status
    
    # Set trap vector (for exceptions)
    la      t0, trap_vector
    csrw    mtvec, t0
    
    # Clear BSS section
    la      t0, __bss_start
    la      t1, __bss_end
clear_bss:
    sd      zero, 0(t0)         # Store 64-bit zero
    addi    t0, t0, 8           # Advance pointer
    blt     t0, t1, clear_bss   # Loop until done
    
    # Jump to ROM bootloader main
    jal     rom_main
    
wait_for_ipi:
    wfi                         # Wait for interrupt
    j       wait_for_ipi        # Loop forever
```

### 2.2 ROM Bootloader Functions
```c
// ROM bootloader (simplified version of what's in JH7110 ROM)
void rom_main(void) {
    // Initialize basic hardware
    init_clocks();              // Set up CPU and peripheral clocks
    init_spi_flash();           // Initialize SPI flash controller
    
    // Try to load SPL from SPI flash
    if (load_spl_from_spi() == 0) {
        jump_to_spl();          // Success - jump to SPL
    }
    
    // Fallback: try UART boot mode
    if (uart_boot_detected()) {
        uart_boot_mode();       // Load via UART
    }
    
    // Final fallback: hang
    while(1) {
        __asm__("wfi");         // Wait for interrupt (low power)
    }
}

int load_spl_from_spi(void) {
    uint8_t *spl_dest = (uint8_t *)0x08000000;  // SPL load address
    uint32_t spl_size = 128 * 1024;             // 128KB max
    
    // Read SPL from SPI flash offset 0x0
    return spi_flash_read(0x0, spl_dest, spl_size);
}

void jump_to_spl(void) {
    void (*spl_entry)(void) = (void (*)(void))0x08000000;
    
    // Flush instruction cache
    __asm__("fence.i");
    
    // Jump to SPL (no return)
    spl_entry();
}
```

---

## 3. SPL Internal Operations

### 3.1 SPL Entry Point
```assembly
# U-Boot SPL entry point (arch/riscv/cpu/start.S)
# Loaded at 0x08000000

.section .text.spl_start
.global _start
_start:
    # Save hart ID and any boot parameters
    csrr    a0, mhartid         # a0 = hart ID
    mv      s0, a0              # Save hart ID in s0
    
    # Only hart 0 continues, others wait
    bnez    a0, secondary_hart_wait
    
    # Set up stack pointer for SPL
    la      sp, __spl_stack_top # Load SPL stack address
    
    # Clear BSS section
    la      t0, __bss_start
    la      t1, __bss_end
clear_bss_spl:
    sd      zero, 0(t0)
    addi    t0, t0, 8
    blt     t0, t1, clear_bss_spl
    
    # Set up global data pointer
    la      gp, __global_data
    
    # Jump to SPL C code
    jal     board_init_f        # Early board initialization
    
secondary_hart_wait:
    wfi
    j       secondary_hart_wait
```

### 3.2 SPL Board Initialization
```c
// SPL board initialization (common/spl/spl.c)
void board_init_f(ulong dummy) {
    int ret;
    
    // Initialize SPL framework
    ret = spl_early_init();
    if (ret) {
        debug("SPL early init failed: %d\n", ret);
        hang();
    }
    
    // Set up early console for debugging
    preloader_console_init();
    debug("SPL: Starting on hart %d\n", current_hartid());
    
    // Initialize timer (needed for delays)
    timer_init();
    
    // Platform-specific early initialization
    ret = spl_board_init();
    if (ret) {
        debug("SPL board init failed: %d\n", ret);
        hang();
    }
    
    // Continue to main SPL function
    board_init_r(NULL, 0);
}

void board_init_r(gd_t *dummy1, ulong dummy2) {
    struct spl_image_info spl_image;
    int ret;
    
    debug("SPL: Initializing DDR\n");
    // Initialize DDR memory
    ret = dram_init();
    if (ret) {
        debug("DDR init failed: %d\n", ret);
        hang();
    }
    
    debug("SPL: DDR initialized, size = %lu MB\n", gd->ram_size >> 20);
    
    // Detect boot device (SPI, eMMC, SD card, etc.)
    spl_boot_list[0] = spl_boot_device();
    debug("SPL: Boot device = %d\n", spl_boot_list[0]);
    
    // Load next stage (OpenSBI + U-Boot)
    ret = spl_load_simple_fit(&spl_image, &info, 
                             BOOT_DEVICE_SPI, 0x100000);  // Offset in SPI flash
    if (ret) {
        debug("Failed to load main firmware: %d\n", ret);
        hang();
    }
    
    debug("SPL: Loaded firmware to 0x%lx, size = %lu\n", 
          spl_image.load_addr, spl_image.size);
    
    // Jump to next stage
    jump_to_image_no_args(&spl_image);
}
```

### 3.3 SPL to OpenSBI Jump
```c
// Jump to loaded firmware (common/spl/spl.c)
void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image) {
    typedef void __noreturn (*image_entry_noargs_t)(void);
    image_entry_noargs_t image_entry;
    
    debug("SPL: Jumping to 0x%lx\n", spl_image->entry_point);
    
    // Cast entry point to function pointer
    image_entry = (image_entry_noargs_t)spl_image->entry_point;
    
    // Invalidate instruction cache
    flush_cache(spl_image->load_addr, spl_image->size);
    
    // Jump to OpenSBI (no return)
    image_entry();
}
```

---

## 4. SPL to OpenSBI Handoff

### 4.1 OpenSBI Entry Point
```assembly
# OpenSBI entry point (firmware/fw_base.S)
# Loaded at 0x40000000

.section .entry, "ax", %progbits
.align 3
.global _start
.global _start_warm
_start:
    # Multi-hart boot process
    csrr    a6, mhartid         # a6 = current hart ID
    
    # Get preferred boot hart
    MOV_3R  s0, a0, s1, a1, s2, a2  # Save parameters
    call    fw_boot_hart        # Get boot hart ID
    add     a6, a0, zero        # Boot hart ID in a6
    MOV_3R  a0, s0, a1, s1, a2, s2  # Restore parameters
    
    # Check if this is the boot hart
    csrr    a7, mhartid
    bne     a6, a7, _wait_relocate_copy_done
    
    # Boot hart initialization
    la      s4, _start_warm     # Warm boot entry point
    la      s5, _link_start     # Link start address
    sub     s4, s4, s5          # Calculate offset
    add     s4, s4, a1          # Add runtime address
    
    # Save boot parameters
    MOV_5R  s0, a0, s1, a1, s2, a2, s3, a3, s4, a4
    call    fw_save_info        # Save firmware info
    MOV_5R  a0, s0, a1, s1, a2, s2, a3, s3, a4, s4
    
    # Initialize OpenSBI
    call    sbi_init            # Main OpenSBI initialization
    
_wait_relocate_copy_done:
    # Secondary harts wait here
    wfi
    j       _wait_relocate_copy_done

_start_warm:
    # Warm boot entry point for secondary harts
    MOV_5R  s0, a0, s1, a1, s2, a2, s3, a3, s4, a4
    call    sbi_hsm_hart_start_finish
    MOV_5R  a0, s0, a1, s1, a2, s2, a3, s3, a4, s4
```

### 4.2 OpenSBI Boot Hart Detection
```c
// OpenSBI boot hart selection (firmware/fw_base.c)
u32 fw_boot_hart(u32 hartid, void *fdt) {
    // For VisionFive2, hart 0 is always the boot hart
    return 0;
}

int fw_save_info(u32 hartid, void *fdt, ulong *next_addr, 
                 ulong *next_mode, ulong *options) {
    // Save device tree address
    *((ulong *)fw_fdt_offset) = (ulong)fdt;
    
    // Set next stage address (U-Boot entry point)
    *next_addr = fw_next_addr(hartid, fdt);
    
    // Set next stage mode (supervisor mode for U-Boot)
    *next_mode = PRV_S;
    
    // No special options
    *options = 0;
    
    return 0;
}

ulong fw_next_addr(u32 hartid, void *fdt) {
    // Return address of embedded U-Boot payload
    return (ulong)&payload_bin;  // Embedded at link time
}
```

---

## 5. OpenSBI Internal Operations

### 5.1 OpenSBI Main Initialization
```c
// OpenSBI main initialization (lib/sbi/sbi_init.c)
void __noreturn sbi_init(struct sbi_scratch *scratch) {
    bool next_mode_supported = false;
    bool coldboot = false, first_boot = false;
    u32 hartid = current_hartid();
    
    // Determine if this is a cold boot
    if (next_mode_supported)
        coldboot = atomic_xchg(&coldboot_lottery, 1) == 0 ? true : false;
    
    // Wait for coldboot hart to initialize
    if (coldboot) {
        init_coldboot(scratch, hartid);  // Primary initialization
    } else {
        init_warmboot(scratch, hartid);  // Secondary hart initialization
    }
}

static void __noreturn init_coldboot(struct sbi_scratch *scratch, u32 hartid) {
    int rc;
    
    // Initialize scratch space
    rc = sbi_scratch_init(scratch);
    if (rc) goto init_coldboot_fail;
    
    // Initialize heap
    rc = sbi_heap_init(scratch);
    if (rc) goto init_coldboot_fail;
    
    // Initialize domains (security/privilege domains)
    rc = sbi_domain_init(scratch, hartid);
    if (rc) goto init_coldboot_fail;
    
    // Initialize hart state management
    rc = sbi_hsm_init(scratch, true);
    if (rc) goto init_coldboot_fail;
    
    // Platform early initialization
    rc = sbi_platform_early_init(plat, true);
    if (rc) goto init_coldboot_fail;
    
    // Initialize this hart
    rc = sbi_hart_init(scratch, true);
    if (rc) goto init_coldboot_fail;
    
    // Initialize interrupt controller
    rc = sbi_irqchip_init(scratch, true);
    if (rc) goto init_coldboot_fail;
    
    // Initialize inter-processor interrupts
    rc = sbi_ipi_init(scratch, true);
    if (rc) goto init_coldboot_fail;
    
    // Initialize timer
    rc = sbi_timer_init(scratch, true);
    if (rc) goto init_coldboot_fail;
    
    // Initialize SBI extensions
    rc = sbi_ecall_init();
    if (rc) goto init_coldboot_fail;
    
    // Initialize console
    sbi_boot_prints(scratch, unpriv_hartid);
    
    // Mark boot complete and start next stage
    sbi_hsm_hart_start_finish(scratch, hartid);
    
init_coldboot_fail:
    sbi_hart_hang();
}
```

### 5.2 SBI Extension Registration
```c
// SBI extension initialization (lib/sbi/sbi_ecall.c)
int sbi_ecall_init(void) {
    int ret;
    
    /* Register base SBI extension */
    ret = sbi_ecall_register_extension(&ecall_base);
    if (ret) return ret;
    
    /* Register timer SBI extension */
    ret = sbi_ecall_register_extension(&ecall_time);
    if (ret) return ret;
    
    /* Register IPI SBI extension */
    ret = sbi_ecall_register_extension(&ecall_ipi);
    if (ret) return ret;
    
    /* Register RFENCE SBI extension */
    ret = sbi_ecall_register_extension(&ecall_rfence);
    if (ret) return ret;
    
    /* Register HSM SBI extension */
    ret = sbi_ecall_register_extension(&ecall_hsm);
    if (ret) return ret;
    
    /* Register system reset SBI extension */
    ret = sbi_ecall_register_extension(&ecall_srst);
    if (ret) return ret;
    
    return 0;
}

// Timer extension example (lib/sbi/sbi_ecall_time.c)
struct sbi_ecall_extension ecall_time = {
    .name = "time",
    .extid_start = SBI_EXT_TIME,
    .extid_end = SBI_EXT_TIME,
    .handle = sbi_ecall_time_handler,
    .probe = sbi_ecall_time_probe,
};

static int sbi_ecall_time_handler(unsigned long extid, unsigned long funcid,
                                  const struct sbi_trap_regs *regs,
                                  unsigned long *out_val,
                                  struct sbi_trap_info *out_trap) {
    int ret = 0;
    
    switch (funcid) {
    case SBI_EXT_TIME_SET_TIMER:
        sbi_timer_event_start((u64)regs->a0);  // Set timer
        break;
    default:
        ret = SBI_ERR_NOT_SUPPORTED;
    }
    
    return ret;
}
```

---

## 6. OpenSBI to U-Boot Handoff

### 6.1 Hart Start Finish Process
```c
// Hart startup completion (lib/sbi/sbi_hsm.c)
void sbi_hsm_hart_start_finish(struct sbi_scratch *scratch, u32 hartid) {
    unsigned long next_addr, next_mode, next_arg1;
    
    // Get next stage parameters
    next_mode = sbi_scratch_offset_ptr(scratch, fw_next_mode_offset);
    next_addr = sbi_scratch_offset_ptr(scratch, fw_next_addr_offset);
    next_arg1 = sbi_scratch_offset_ptr(scratch, fw_next_arg1_offset);
    
    sbi_printf("OpenSBI: Jumping to U-Boot at 0x%lx (hart %d)\n", 
               next_addr, hartid);
    
    // Switch to supervisor mode and jump to U-Boot
    sbi_hart_switch_mode(hartid, next_arg1, next_addr, next_mode, false);
}

// Mode switching function (lib/sbi/sbi_hart.c)
void __noreturn sbi_hart_switch_mode(unsigned long arg0, unsigned long arg1,
                                     unsigned long next_addr, 
                                     unsigned long next_mode,
                                     bool next_virt) {
    unsigned long val;
    
    // Set up machine status register for supervisor mode
    val = csr_read(CSR_MSTATUS);
    val = INSERT_FIELD(val, MSTATUS_MPP, next_mode);  // Set Previous Privilege
    val = INSERT_FIELD(val, MSTATUS_MPIE, 0);         // Disable interrupts
    csr_write(CSR_MSTATUS, val);
    
    // Set machine exception PC to U-Boot entry point
    csr_write(CSR_MEPC, next_addr);
    
    // Set up machine interrupt delegation
    csr_write(CSR_MIDELEG, 0);
    csr_write(CSR_MEDELEG, 0);
    
    // Clear machine interrupt pending
    csr_write(CSR_MIP, 0);
    
    // Return to supervisor mode (this jumps to U-Boot!)
    __asm__ __volatile__("mret");
}
```

### 6.2 Register State at U-Boot Entry
```assembly
# Register state when U-Boot starts (passed from OpenSBI)
# a0 = hart ID (hardware thread ID)
# a1 = device tree blob address (FDT)
# Other registers may contain firmware-specific data
```

---

## 7. U-Boot Internal Operations

### 7.1 U-Boot Entry Point
```assembly
# U-Boot entry point (arch/riscv/cpu/start.S)
# Entry from OpenSBI in supervisor mode

.section .text.start
.global _start
_start:
    # Save hart ID and device tree address
    csrr    a0, mhartid         # Get hart ID (should match a0 from OpenSBI)
    mv      tp, a0              # Save hart ID in thread pointer
    mv      s1, a1              # Save device tree address
    
    # Only hart 0 continues, others wait
    bnez    a0, wait_for_ipi
    
    # Clear BSS section
    la      t0, __bss_start
    la      t1, __bss_end
1:
    sd      zero, 0(t0)
    addi    t0, t0, 8
    blt     t0, t1, 1b
    
    # Set up initial stack
    la      sp, SYS_INIT_SP_ADDR  # Initial stack pointer
    
    # Save device tree address in global data
    la      t0, prior_stage_fdt_address
    sd      s1, 0(t0)
    
    # Call C code for memory setup
    jal     board_init_f_init_reserve
    
    # Call main board initialization
    jal     board_init_f
    
wait_for_ipi:
    wfi
    j       wait_for_ipi
```

### 7.2 U-Boot Board Initialization
```c
// U-Boot board initialization (common/board_f.c)
void board_init_f(ulong boot_flags) {
    gd->flags = boot_flags;
    
    if (initcall_run_list(init_sequence_f))
        hang();
    
    board_init_f_r_trampoline(gd->start_addr_sp);
}

// Initialization sequence (common/board_f.c)
static const init_fnc_t init_sequence_f[] = {
    setup_mon_len,              // Setup monitor length
    fdtdec_setup,              // Device tree setup
    arch_cpu_init,             // CPU-specific initialization
    mark_bootstage,            // Boot stage timing
    timer_init,                // Timer initialization (uses SBI!)
    env_init,                  // Environment initialization
    init_baud_rate,            // Serial baud rate
    serial_init,               // Serial port setup
    console_init_f,            // Console initialization
    dram_init,                 // DRAM initialization
    reserve_round_4k,          // Memory alignment
    reserve_mmu,               // MMU setup
    reserve_trace,             // Tracing memory
    reserve_uboot,             // U-Boot relocation
    reserve_malloc,            // Malloc memory
    reserve_board,             // Board-specific reservations
    setup_machine_fdt,         // Machine device tree
    reserve_global_data,       // Global data structure
    reserve_fdt,               // Device tree reservation
    reserve_bootstage,         // Boot stage memory
    reserve_bloblist,          // Blob list memory
    reserve_arch,              // Architecture-specific
    reserve_stacks,            // Stack memory
    dram_init_banksize,        // Bank size setup
    show_dram_config,          // Display DRAM config
    INIT_FUNC_WATCHDOG_RESET   // Watchdog reset
    setup_dest_addr,           // Setup destination address
    reserve_global_data,       // Reserve global data
    reserve_fdt,               // Reserve FDT
    reserve_stacks,            // Reserve stacks
    setup_dram_config,         // Setup DRAM config
    show_dram_config,          // Show DRAM config
    init_func_i2c,             // I2C initialization
    announce_dram_init,        // Announce DRAM init
    dram_init,                 // Initialize DRAM
    post_init_f,               // Post-initialization
    NULL,                      // End marker
};
```

### 7.3 U-Boot Timer Initialization (SBI Interface)
```c
// RISC-V timer initialization using SBI (drivers/timer/riscv_timer.c)
static int riscv_timer_probe(struct udevice *dev) {
    struct timer_dev_priv *uc_priv = dev_get_uclass_priv(dev);
    
    // Get timer frequency from device tree or SBI
    if (dev_read_u32(dev, "clock-frequency", &uc_priv->clock_rate)) {
        // Fallback: get from SBI
        uc_priv->clock_rate = riscv_get_time_freq();
    }
    
    debug("RISC-V timer initialized: %u Hz\n", uc_priv->clock_rate);
    return 0;
}

// Timer count reading (may call SBI)
static u64 riscv_timer_get_count(struct udevice *dev) {
    __maybe_unused u32 hi, lo;
    
    if (IS_ENABLED(CONFIG_64BIT))
        return csr_read(CSR_TIME);      // Direct CSR read on 64-bit
    
    // 32-bit requires careful reading to avoid overflow
    do {
        hi = csr_read(CSR_TIMEH);
        lo = csr_read(CSR_TIME);
    } while (hi != csr_read(CSR_TIMEH));
    
    return ((u64)hi << 32) | lo;
}

// SBI timer setting (arch/riscv/lib/sbi.c)
struct sbiret sbi_set_timer(uint64_t stime_value) {
    return sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER,
                     stime_value, 0, 0, 0, 0, 0);
}

// SBI ecall implementation
struct sbiret sbi_ecall(int ext, int fid, unsigned long arg0,
                       unsigned long arg1, unsigned long arg2,
                       unsigned long arg3, unsigned long arg4,
                       unsigned long arg5) {
    struct sbiret ret;
    
    register uintptr_t a0 asm("a0") = (uintptr_t)(arg0);
    register uintptr_t a1 asm("a1") = (uintptr_t)(arg1);
    register uintptr_t a2 asm("a2") = (uintptr_t)(arg2);
    register uintptr_t a3 asm("a3") = (uintptr_t)(arg3);
    register uintptr_t a4 asm("a4") = (uintptr_t)(arg4);
    register uintptr_t a5 asm("a5") = (uintptr_t)(arg5);
    register uintptr_t a6 asm("a6") = (uintptr_t)(fid);
    register uintptr_t a7 asm("a7") = (uintptr_t)(ext);
    
    asm volatile("ecall"                    // Environment call to OpenSBI
                : "+r"(a0), "+r"(a1)
                : "r"(a2), "r"(a3), "r"(a4), "r"(a5), "r"(a6), "r"(a7)
                : "memory");
                
    ret.error = a0;
    ret.value = a1;
    
    return ret;
}
```

---

## 8. U-Boot to Kernel Handoff

### 8.1 U-Boot Main Loop and Autoboot
```c
// U-Boot main loop (common/main.c)
void main_loop(void) {
    const char *s;
    
    cli_init();                            // Initialize command line interface
    run_preboot_environment_command();     // Run preboot commands
    
    s = bootdelay_process();               // Process boot delay
    
    if (cli_process_fdt(&s))               // Process FDT commands
        cli_secure_boot_cmd(s);            // Secure boot commands
        
    autoboot_command(s);                   // Execute autoboot command
    cli_loop();                            // Interactive command loop
}

// Autoboot command execution (common/autoboot.c)
void autoboot_command(const char *s) {
    debug("### main_loop: bootcmd=\"%s\"\n", s ? s : "<UNDEFINED>");
    
    if (s && !abortboot(bootdelay)) {
        run_command_list(s, -1, 0);       // Execute bootcmd
    }
}

// Typical bootcmd: "load mmc 0:3 ${kernel_addr_r} image.fit; bootm ${kernel_addr_r}"
```

### 8.2 Kernel Loading Process
```c
// Boot Linux kernel (cmd/bootm.c)
int do_bootm(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[]) {
    return do_bootm_states(cmdtp, flag, argc, argv,
                          BOOTM_STATE_START | BOOTM_STATE_FINDOS |
                          BOOTM_STATE_LOADOS | BOOTM_STATE_OS_GO,
                          &images, 1);
}

static int do_bootm_states(struct cmd_tbl *cmdtp, int flag, int argc,
                          char *const argv[], int states,
                          bootm_headers_t *images, int boot_progress) {
    boot_os_fn *boot_fn;
    int ret = 0;
    
    if (states & BOOTM_STATE_START)
        ret = bootm_start(cmdtp, flag, argc, argv);
        
    if (!ret && (states & BOOTM_STATE_FINDOS))
        ret = bootm_find_os(cmdtp, flag, argc, argv);
        
    if (!ret && (states & BOOTM_STATE_LOADOS))
        ret = bootm_load_os(images, 0);
        
    if (!ret && (states & BOOTM_STATE_OS_GO)) {
        boot_fn = bootm_os_get_boot_func(images->os.os);
        boot_fn(0, argc, argv, images);    // Jump to kernel!
    }
    
    return ret;
}
```

### 8.3 Kernel Jump Assembly
```c
// Jump to Linux kernel (arch/riscv/lib/bootm.c)
static void boot_jump_linux(bootm_headers_t *images, int flag) {
    void (*kernel)(ulong hart_id, void *dtb);
    ulong fdt_addr;
    
    kernel = (void (*)(ulong, void *))images->ep;     // Kernel entry point
    fdt_addr = images->ft_addr;                       // Device tree address
    
    announce_and_cleanup(flag);                       // Cleanup U-Boot
    
    if (IMAGE_ENABLE_OF_LIBFDT && images->ft_len) {
        debug("## Flattened Device Tree blob at %08lx\n", fdt_addr);
        if (image_setup_linux(images)) {
            printf("FDT setup failed!");
            hang();
        }
    }
    
    debug("## Transferring control to Linux (at address %08lx) ...\n",
          (ulong)kernel);
    
    // Invalidate instruction cache
    flush_cache((ulong)kernel, 0);
    
    // Jump to kernel with hart ID and device tree
    kernel(gd->arch.boot_hart, (void *)fdt_addr);
}
```

---

## 9. RISC-V Assembly Language Primer

### 9.1 RISC-V Register Set
```assembly
# RISC-V has 32 general-purpose registers (x0-x31)
# ABI names and usage:

x0  (zero) - Always zero (hardwired)
x1  (ra)   - Return address
x2  (sp)   - Stack pointer
x3  (gp)   - Global pointer
x4  (tp)   - Thread pointer
x5  (t0)   - Temporary register 0
x6  (t1)   - Temporary register 1
x7  (t2)   - Temporary register 2
x8  (s0/fp)- Saved register 0 / Frame pointer
x9  (s1)   - Saved register 1
x10 (a0)   - Function argument 0 / Return value 0
x11 (a1)   - Function argument 1 / Return value 1
x12 (a2)   - Function argument 2
x13 (a3)   - Function argument 3
x14 (a4)   - Function argument 4
x15 (a5)   - Function argument 5
x16 (a6)   - Function argument 6
x17 (a7)   - Function argument 7
x18 (s2)   - Saved register 2
x19 (s3)   - Saved register 3
x20 (s4)   - Saved register 4
x21 (s5)   - Saved register 5
x22 (s6)   - Saved register 6
x23 (s7)   - Saved register 7
x24 (s8)   - Saved register 8
x25 (s9)   - Saved register 9
x26 (s10)  - Saved register 10
x27 (s11)  - Saved register 11
x28 (t3)   - Temporary register 3
x29 (t4)   - Temporary register 4
x30 (t5)   - Temporary register 5
x31 (t6)   - Temporary register 6
```

### 9.2 RISC-V Instruction Formats
```assembly
# Basic instruction types:

# R-type (Register): reg-reg operations
add  rd, rs1, rs2      # rd = rs1 + rs2
sub  rd, rs1, rs2      # rd = rs1 - rs2
and  rd, rs1, rs2      # rd = rs1 & rs2
or   rd, rs1, rs2      # rd = rs1 | rs2
xor  rd, rs1, rs2      # rd = rs1 ^ rs2
slt  rd, rs1, rs2      # rd = (rs1 < rs2) ? 1 : 0

# I-type (Immediate): reg-immediate operations
addi rd, rs1, imm      # rd = rs1 + imm
andi rd, rs1, imm      # rd = rs1 & imm
ori  rd, rs1, imm      # rd = rs1 | imm
xori rd, rs1, imm      # rd = rs1 ^ imm
slti rd, rs1, imm      # rd = (rs1 < imm) ? 1 : 0

# Load instructions
ld   rd, offset(rs1)   # rd = memory[rs1 + offset] (64-bit)
lw   rd, offset(rs1)   # rd = memory[rs1 + offset] (32-bit)
lh   rd, offset(rs1)   # rd = memory[rs1 + offset] (16-bit)
lb   rd, offset(rs1)   # rd = memory[rs1 + offset] (8-bit)

# S-type (Store): store operations
sd   rs2, offset(rs1)  # memory[rs1 + offset] = rs2 (64-bit)
sw   rs2, offset(rs1)  # memory[rs1 + offset] = rs2 (32-bit)
sh   rs2, offset(rs1)  # memory[rs1 + offset] = rs2 (16-bit)
sb   rs2, offset(rs1)  # memory[rs1 + offset] = rs2 (8-bit)

# B-type (Branch): conditional branches
beq  rs1, rs2, label   # if (rs1 == rs2) goto label
bne  rs1, rs2, label   # if (rs1 != rs2) goto label
blt  rs1, rs2, label   # if (rs1 < rs2) goto label
bge  rs1, rs2, label   # if (rs1 >= rs2) goto label
bltu rs1, rs2, label   # if (rs1 < rs2) goto label (unsigned)
bgeu rs1, rs2, label   # if (rs1 >= rs2) goto label (unsigned)

# J-type (Jump): unconditional jumps
jal  rd, label         # rd = pc + 4; pc = label
jalr rd, rs1, offset   # rd = pc + 4; pc = rs1 + offset

# U-type (Upper immediate): large immediate values
lui  rd, imm           # rd = imm << 12
auipc rd, imm          # rd = pc + (imm << 12)
```

### 9.3 RISC-V CSR (Control and Status Register) Instructions
```assembly
# CSR instructions for system programming:

csrr rd, csr           # rd = csr (read CSR)
csrw csr, rs1          # csr = rs1 (write CSR)
csrs csr, rs1          # csr |= rs1 (set bits)
csrc csr, rs1          # csr &= ~rs1 (clear bits)

# Common CSRs in boot code:
mhartid                # Machine Hart ID (read-only)
mstatus                # Machine Status register
mtvec                  # Machine Trap Vector Base
mepc                   # Machine Exception Program Counter
mcause                 # Machine Cause register
mtval                  # Machine Trap Value
mie                    # Machine Interrupt Enable
mip                    # Machine Interrupt Pending
time                   # Current time (read-only)
timeh                  # Upper 32 bits of time (RV32 only)

# Privilege instructions:
ecall                  # Environment call (trap to higher privilege)
ebreak                 # Environment break (debugger breakpoint)
mret                   # Return from machine mode
sret                   # Return from supervisor mode
wfi                    # Wait for interrupt (low power)
fence                  # Memory fence
fence.i                # Instruction fence (flush I-cache)
```

### 9.4 RISC-V Calling Convention
```assembly
# Function call example:

caller:
    # Save caller-saved registers (if needed)
    addi sp, sp, -16       # Allocate stack space
    sd   ra, 8(sp)         # Save return address
    sd   s0, 0(sp)         # Save saved register
    
    # Prepare arguments
    li   a0, 42            # First argument
    li   a1, 123           # Second argument
    
    # Call function
    jal  ra, callee        # Call function, ra = return address
    
    # Use return value (in a0)
    mv   s0, a0            # Save return value
    
    # Restore registers
    ld   ra, 8(sp)         # Restore return address
    ld   s0, 0(sp)         # Restore saved register
    addi sp, sp, 16        # Deallocate stack space
    
    # Return
    jalr zero, ra, 0       # Return (or just 'ret')

callee:
    # Function prologue
    addi sp, sp, -16       # Allocate stack frame
    sd   ra, 8(sp)         # Save return address
    sd   s0, 0(sp)         # Save frame pointer
    addi s0, sp, 16        # Set frame pointer
    
    # Function body
    add  a0, a0, a1        # Result = arg0 + arg1
    
    # Function epilogue
    ld   ra, 8(sp)         # Restore return address
    ld   s0, 0(sp)         # Restore frame pointer
    addi sp, sp, 16        # Deallocate stack frame
    
    # Return
    jalr zero, ra, 0       # Return to caller
```

---

## 10. Register Usage and Calling Conventions

### 10.1 Boot Flow Register Usage

#### SPL to OpenSBI Handoff
```assembly
# When SPL jumps to OpenSBI:
# No specific register requirements, OpenSBI is self-contained
# OpenSBI will detect hardware configuration independently
```

#### OpenSBI to U-Boot Handoff
```assembly
# When OpenSBI jumps to U-Boot (sbi_hart_switch_mode):
# a0 = hart ID (hardware thread identifier)
# a1 = device tree blob address (FDT)
# pc = U-Boot entry point
# privilege = supervisor mode
# All other registers undefined
```

#### U-Boot to Kernel Handoff
```assembly
# When U-Boot jumps to Linux kernel:
# a0 = hart ID (hardware thread identifier)  
# a1 = device tree blob address (FDT)
# pc = kernel entry point  
# privilege = supervisor mode
# MMU may be enabled
# All other registers undefined
```

### 10.2 SBI Call Convention
```assembly
# SBI calls use standard RISC-V calling convention:
# a7 = SBI extension ID
# a6 = SBI function ID  
# a0-a5 = function arguments
# Return: a0 = error code, a1 = return value

# Example: Set timer
li   a7, 0x54494D45        # SBI_EXT_TIME  
li   a6, 0                 # SBI_EXT_TIME_SET_TIMER
ld   a0, time_value        # Timer value
ecall                      # Call OpenSBI
# Returns: a0 = error, a1 = return value
```

### 10.3 Memory Management During Boot
```assembly
# Memory layout evolution during boot:

# 1. SPL stage (limited memory, stack in SRAM)
la   sp, 0x08020000        # SPL stack at end of SRAM

# 2. OpenSBI stage (DDR available, setup stacks)  
la   sp, 0x40080000        # OpenSBI stack in DDR

# 3. U-Boot stage (full memory management)
la   sp, CONFIG_SYS_INIT_SP_ADDR  # U-Boot initial stack

# 4. Kernel stage (virtual memory)
# Kernel sets up its own stack and memory management
```

This comprehensive analysis shows how each boot stage carefully hands off control to the next, maintaining the RISC-V calling conventions and register usage patterns throughout the boot process. The modular design allows each component to focus on its specific responsibilities while providing clean interfaces between stages.

---

**Built with:** StarFive JH7110 SoC, RISC-V RV64GC ISA  
**Documentation Date:** November 1, 2025