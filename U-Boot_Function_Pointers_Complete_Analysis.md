# U-Boot Function Calls, APIs, Callbacks and Function Pointers - Complete Analysis

**Target:** StarFive VisionFive2 (JH7110 SoC)  
**U-Boot Version:** 2023.07  
**Architecture:** RISC-V RV64GC  

---

## 📋 Table of Contents

1. [U-Boot Function Pointer Architecture](#1-u-boot-function-pointer-architecture)
2. [Driver Model Callbacks](#2-driver-model-callbacks)
3. [Command Framework Function Pointers](#3-command-framework-function-pointers)
4. [Environment Variable Callbacks](#4-environment-variable-callbacks)
5. [Boot Flow Function Pointers](#5-boot-flow-function-pointers)
6. [Timer and Hardware Callbacks](#6-timer-and-hardware-callbacks)
7. [Network Stack Function Pointers](#7-network-stack-function-pointers)
8. [File System API Callbacks](#8-file-system-api-callbacks)
9. [SPL Function Pointers](#9-spl-function-pointers)
10. [Function Call Examples with Register Usage](#10-function-call-examples-with-register-usage)

---

## 1. U-Boot Function Pointer Architecture

### 1.1 Core Function Pointer Types

U-Boot extensively uses function pointers for modularity and extensibility:

```c
// Common function pointer types in U-Boot
typedef int (*init_fnc_t)(void);                    // Initialization functions
typedef int (*cmd_func_t)(struct cmd_tbl *, int, int, char * const []);  // Command functions
typedef void (*irq_handler_t)(int, void *);         // Interrupt handlers
typedef int (*bootm_os_fn_t)(int, int, char *, bootm_headers_t *);       // OS boot functions
typedef int (*spl_load_fn_t)(struct spl_image_info *, struct spl_boot_device *); // SPL loaders
```

### 1.2 Function Pointer Registration Macros

```c
// U-Boot uses linker-based registration for function pointers
#define U_BOOT_CMD(name, maxargs, rep, cmd, usage, help) \
    static const struct cmd_tbl __u_boot_cmd_##name __used \
    __section(".u_boot_cmd") = { \
        .name = #name, \
        .maxargs = maxargs, \
        .repeatable = rep, \
        .cmd = cmd,             /* Function pointer! */ \
        .usage = usage, \
        .help = help, \
    }

#define U_BOOT_DRIVER(dname) \
    static const struct driver __u_boot_driver_##dname __used \
    __section(".u_boot_driver") = { \
        .name = #dname, \
        .probe = dname##_probe,      /* Function pointer! */ \
        .remove = dname##_remove,    /* Function pointer! */ \
        .ops = &dname##_ops,         /* Operations structure with function pointers! */ \
    }
```

---

## 2. Driver Model Callbacks

### 2.1 Driver Operations Structure

```c
// Example: UART driver operations (function pointer table)
struct serial_device_priv {
    struct dm_serial_ops *ops;              // Function pointer table
};

struct dm_serial_ops {
    int (*setbrg)(struct udevice *dev, int baudrate);       // Set baud rate callback
    int (*getc)(struct udevice *dev);                       // Get character callback  
    int (*putc)(struct udevice *dev, const char ch);        // Put character callback
    int (*pending)(struct udevice *dev, bool input);        // Check pending callback
    int (*clear)(struct udevice *dev);                      // Clear buffer callback
    int (*loop)(struct udevice *dev, int on);               // Loopback mode callback
    int (*getconfig)(struct udevice *dev, uint *config);    // Get config callback
    int (*setconfig)(struct udevice *dev, uint config);     // Set config callback
    int (*getinfo)(struct udevice *dev, struct serial_device_info *info); // Get info callback
};

// RISC-V UART driver implementation for VisionFive2
static const struct dm_serial_ops starfive_serial_ops = {
    .setbrg = starfive_serial_setbrg,       // Function pointer assignment
    .getc = starfive_serial_getc,           // Function pointer assignment
    .putc = starfive_serial_putc,           // Function pointer assignment
    .pending = starfive_serial_pending,     // Function pointer assignment
};

// Function pointer call example
static int serial_getc_dev(struct udevice *dev) {
    struct dm_serial_ops *ops = serial_get_ops(dev);
    
    if (!ops->getc)
        return -ENOSYS;
    
    return ops->getc(dev);                  // Indirect function call via pointer!
}
```

### 2.2 Device Probe/Remove Callbacks

```c
// VisionFive2 specific device callbacks
static int starfive_visionfive2_probe(struct udevice *dev) {
    struct starfive_visionfive2_priv *priv = dev_get_priv(dev);
    int ret;
    
    // Clock callback
    ret = clk_get_by_index(dev, 0, &priv->clk);
    if (ret) return ret;
    
    // Reset callback  
    ret = reset_get_by_index(dev, 0, &priv->reset);
    if (ret) return ret;
    
    // GPIO callback
    ret = gpio_request_by_name(dev, "reset-gpios", 0, 
                              &priv->reset_gpio, GPIOD_IS_OUT);
    
    return ret;
}

static int starfive_visionfive2_remove(struct udevice *dev) {
    struct starfive_visionfive2_priv *priv = dev_get_priv(dev);
    
    // Cleanup callbacks
    clk_free(&priv->clk);
    reset_free(&priv->reset);
    gpio_free_list(dev, &priv->reset_gpio, 1);
    
    return 0;
}

// Driver registration with function pointers
U_BOOT_DRIVER(starfive_visionfive2) = {
    .name = "starfive_visionfive2",
    .id = UCLASS_MISC,
    .of_match = starfive_visionfive2_ids,
    .probe = starfive_visionfive2_probe,        // Probe function pointer
    .remove = starfive_visionfive2_remove,      // Remove function pointer
    .priv_auto = sizeof(struct starfive_visionfive2_priv),
    .ops = &starfive_visionfive2_ops,          // Operations function pointer table
};
```

### 2.3 Device Tree Callback Chain

```c
// Device tree parsing callbacks
struct udevice_id {
    const char *compatible;
    ulong data;                               // Can hold function pointer!
};

// Device tree callback execution
static int device_probe(struct udevice *dev) {
    const struct driver *drv = dev->driver;
    int ret;
    
    // Call pre-probe callbacks
    if (dev->parent && dev->parent->driver->child_pre_probe) {
        ret = dev->parent->driver->child_pre_probe(dev);  // Function pointer call
        if (ret) return ret;
    }
    
    // Call main probe callback
    if (drv->probe) {
        ret = drv->probe(dev);                // Function pointer call
        if (ret) goto fail;
    }
    
    // Call post-probe callbacks  
    if (dev->parent && dev->parent->driver->child_post_probe) {
        ret = dev->parent->driver->child_post_probe(dev); // Function pointer call
        if (ret) goto fail;
    }
    
    return 0;
fail:
    return ret;
}
```

---

## 3. Command Framework Function Pointers

### 3.1 Command Structure and Registration

```c
// Command table structure with function pointer
struct cmd_tbl {
    char *name;                             // Command name
    int maxargs;                            // Maximum arguments
    int repeatable;                         // Can be repeated
    cmd_func_t cmd;                         // Function pointer to command implementation!
    char *usage;                            // Usage string
    char *help;                             // Help text
    cmd_func_t complete;                    // Tab completion function pointer
};

// bootm command implementation (function that gets called via pointer)
int do_bootm(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[]) {
    int states;
    
    // Determine boot states
    states = BOOTM_STATE_START | BOOTM_STATE_FINDOS | 
             BOOTM_STATE_LOADOS | BOOTM_STATE_OS_GO;
    
    return do_bootm_states(cmdtp, flag, argc, argv, states, &images, 1);
}

// Command registration using function pointer
U_BOOT_CMD(bootm, CONFIG_SYS_MAXARGS, 1, do_bootm,  // Function pointer here!
    "boot application image from memory",
    "[addr [arg ...]]\n    - boot application image stored in memory"
);

// Command execution via function pointer
int cmd_process(int flag, int argc, char *const argv[], int *repeatable) {
    struct cmd_tbl *cmdtp;
    
    // Find command in table
    cmdtp = find_cmd(argv[0]);
    if (!cmdtp) {
        printf("Unknown command '%s'\n", argv[0]);
        return CMD_RET_UNKNOWN;
    }
    
    // Execute command via function pointer!
    return cmdtp->cmd(cmdtp, flag, argc, argv);  // Indirect call!
}
```

### 3.2 Sub-command Function Pointers

```c
// Sub-command structure for complex commands like bootm
static struct cmd_tbl cmd_bootm_sub[] = {
    U_BOOT_CMD_MKENT(start, 0, 1, (void *)BOOTM_STATE_START, "", ""),
    U_BOOT_CMD_MKENT(loados, 0, 1, (void *)BOOTM_STATE_LOADOS, "", ""),
    U_BOOT_CMD_MKENT(ramdisk, 0, 1, (void *)BOOTM_STATE_RAMDISK, "", ""),
    U_BOOT_CMD_MKENT(fdt, 0, 1, (void *)BOOTM_STATE_FDT, "", ""),
    U_BOOT_CMD_MKENT(cmdline, 0, 1, (void *)BOOTM_STATE_OS_CMDLINE, "", ""),
    U_BOOT_CMD_MKENT(bdt, 0, 1, (void *)BOOTM_STATE_OS_BD_T, "", ""),
    U_BOOT_CMD_MKENT(prep, 0, 1, (void *)BOOTM_STATE_OS_PREP, "", ""),
    U_BOOT_CMD_MKENT(fake, 0, 1, (void *)BOOTM_STATE_OS_FAKE_GO, "", ""),
    U_BOOT_CMD_MKENT(go, 0, 1, (void *)BOOTM_STATE_OS_GO, "", ""),
};

// Sub-command dispatch using function pointers
static int do_bootm_subcommand(struct cmd_tbl *cmdtp, int flag, int argc,
                               char *const argv[]) {
    struct cmd_tbl *c;
    int state;
    
    // Find sub-command
    c = find_cmd_tbl(argv[0], &cmd_bootm_sub[0], ARRAY_SIZE(cmd_bootm_sub));
    if (c) {
        state = (long)c->cmd;                    // Get state from function pointer field
        if (state == BOOTM_STATE_START)
            return bootm_start(cmdtp, flag, argc, argv);
        else
            return do_bootm_states(cmdtp, flag, argc, argv, state, &images, 0);
    }
    return CMD_RET_USAGE;
}
```

---

## 4. Environment Variable Callbacks

### 4.1 Environment Callback System

```c
// Environment callback structure
struct env_clbk_tbl {
    const char *name;                       // Variable name
    int (*callback)(const char *name, const char *value,
                   enum env_op op, int flags);  // Callback function pointer!
};

// Example: bootdelay callback function
static int on_bootdelay(const char *name, const char *value,
                       enum env_op op, int flags) {
    switch (op) {
    case env_op_create:
    case env_op_overwrite:
        // Validate bootdelay value
        simple_strtoul(value, NULL, 10);
        break;
    case env_op_delete:
        // Reset to default
        break;
    }
    return 0;
}

// Register callback using function pointer
U_BOOT_ENV_CALLBACK(bootdelay, on_bootdelay);  // Function pointer registration!

// Environment callback execution
static int env_call_callback(const char *name, const char *value,
                            enum env_op op, int flags) {
    struct env_clbk_tbl *clbkp;
    
    // Find callback
    clbkp = find_env_callback(name);
    if (clbkp != NULL) {
        // Execute callback via function pointer
        return clbkp->callback(name, value, op, flags);  // Indirect call!
    }
    
    return 0;
}
```

### 4.2 Board-Specific Environment Callbacks

```c
// VisionFive2 specific environment callbacks
static int on_jh7110_cpu_freq(const char *name, const char *value,
                              enum env_op op, int flags) {
    unsigned long freq;
    
    if (op == env_op_create || op == env_op_overwrite) {
        freq = simple_strtoul(value, NULL, 10);
        if (freq < 375000000 || freq > 1500000000) {
            printf("Invalid CPU frequency: %lu Hz\n", freq);
            return 1;
        }
        // Apply frequency change
        starfive_set_cpu_freq(freq);
    }
    
    return 0;
}

static int on_jh7110_ddr_freq(const char *name, const char *value,
                              enum env_op op, int flags) {
    unsigned long freq;
    
    if (op == env_op_create || op == env_op_overwrite) {
        freq = simple_strtoul(value, NULL, 10);
        // Validate and apply DDR frequency
        starfive_set_ddr_freq(freq);
    }
    
    return 0;
}

// Register VisionFive2 callbacks
U_BOOT_ENV_CALLBACK(jh7110_cpu_freq, on_jh7110_cpu_freq);
U_BOOT_ENV_CALLBACK(jh7110_ddr_freq, on_jh7110_ddr_freq);
```

---

## 5. Boot Flow Function Pointers

### 5.1 Initialization Sequence Function Pointers

```c
// Initialization function pointer array
static const init_fnc_t init_sequence_f[] = {
    setup_mon_len,              // Function pointer
    fdtdec_setup,              // Function pointer  
    arch_cpu_init,             // Function pointer
    mark_bootstage,            // Function pointer
    initf_malloc,              // Function pointer
    log_init,                  // Function pointer
    initf_bootstage,           // Function pointer
    arch_cpu_init_dm,          // Function pointer
    timer_init,                // Function pointer (calls SBI!)
    board_early_init_f,        // Function pointer
    NULL,                      // Terminator
};

// Function pointer execution engine
int initcall_run_list(const init_fnc_t init_sequence[]) {
    const init_fnc_t *init_fnc_ptr;
    
    for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
        unsigned long reloc_ofs = 0;
        int ret;
        
        if (gd->flags & GD_FLG_RELOC)
            reloc_ofs = gd->reloc_off;
            
        debug("initcall: %p\n", (char *)*init_fnc_ptr - reloc_ofs);
        
        // Execute function via pointer!
        ret = (*init_fnc_ptr)();                    // Indirect function call!
        
        if (ret) {
            printf("initcall sequence failed at call %p (err=%d)\n",
                   (char *)*init_fnc_ptr - reloc_ofs, ret);
            return -1;
        }
    }
    return 0;
}
```

### 5.2 OS Boot Function Pointers

```c
// Operating system boot function pointer table
struct bootm_os_ops {
    bootm_os_fn_t load_os;                  // Function pointer for OS loading
    bootm_os_fn_t go_os;                    // Function pointer for OS execution
};

// Linux-specific boot functions
static int do_bootm_linux(struct cmd_tbl *cmdtp, int flag, int argc,
                          char *const argv[], bootm_headers_t *images) {
    // Linux-specific boot preparation
    return boot_jump_linux(images, flag);
}

// Linux boot function pointer registration
static struct bootm_os_ops bootm_linux_ops = {
    .load_os = do_bootm_linux,             // Function pointer assignment
    .go_os = do_bootm_linux,               // Function pointer assignment
};

// Boot OS via function pointer
static int bootm_os_get_boot_func(int os) {
    switch (os) {
    case IH_OS_LINUX:
        return &bootm_linux_ops;            // Return function pointer table
    case IH_OS_VXWORKS:
        return &bootm_vxworks_ops;
    case IH_OS_QNX:
        return &bootm_qnx_ops;
    default:
        return NULL;
    }
}

// Execute OS boot via function pointer
static int bootm_load_os(bootm_headers_t *images, int boot_progress) {
    struct bootm_os_ops *ops = bootm_os_get_boot_func(images->os.os);
    
    if (ops && ops->load_os) {
        return ops->load_os(NULL, 0, 0, NULL, images);  // Function pointer call!
    }
    
    return -ENOSYS;
}
```

---

## 6. Timer and Hardware Callbacks

### 6.1 RISC-V Timer Function Pointers

```c
// Timer operations structure with function pointers
struct timer_ops {
    u64 (*get_count)(struct udevice *dev);   // Function pointer for reading timer
    int (*get_rate)(struct udevice *dev, unsigned long *rate); // Function pointer for rate
};

// RISC-V timer implementation using function pointers
static const struct timer_ops riscv_timer_ops = {
    .get_count = riscv_timer_get_count,      // Function pointer assignment
    .get_rate = riscv_timer_get_rate,        // Function pointer assignment
};

// Timer function implementations called via pointers
static u64 riscv_timer_get_count(struct udevice *dev) {
    if (IS_ENABLED(CONFIG_64BIT))
        return csr_read(CSR_TIME);           // Direct CSR read (may trap to OpenSBI)
    
    // 32-bit requires careful reading
    u32 hi, lo;
    do {
        hi = csr_read(CSR_TIMEH);
        lo = csr_read(CSR_TIME);
    } while (hi != csr_read(CSR_TIMEH));
    
    return ((u64)hi << 32) | lo;
}

static int riscv_timer_get_rate(struct udevice *dev, unsigned long *rate) {
    // Get timer frequency from device tree or SBI
    *rate = dev_read_u32_default(dev, "clock-frequency", 0);
    if (!*rate) {
        // Fallback to SBI call
        struct sbiret ret = sbi_get_time_freq();
        if (ret.error)
            return -ENODEV;
        *rate = ret.value;
    }
    return 0;
}

// Timer function pointer usage
int timer_get_count(struct udevice *dev, u64 *count) {
    const struct timer_ops *ops = device_get_ops(dev);
    
    if (!ops->get_count)
        return -ENOSYS;
        
    *count = ops->get_count(dev);           // Function pointer call!
    return 0;
}
```

### 6.2 Interrupt Handler Function Pointers

```c
// Interrupt handler function pointer type
typedef void (*irq_handler_t)(int irq, void *data);

// Interrupt handler structure
struct irq_handler {
    int irq;
    irq_handler_t handler;                  // Function pointer!
    void *data;
    struct irq_handler *next;
};

// RISC-V timer interrupt handler
static void riscv_timer_interrupt(int irq, void *data) {
    // Handle timer interrupt from OpenSBI
    increment_ticks();
    
    // Set next timer interrupt via SBI
    sbi_set_timer(get_ticks() + gd->arch.clk_freq);
    
    // Call registered timer callbacks
    if (timer_callback)
        timer_callback();                   // Another function pointer call!
}

// Register interrupt handler via function pointer
int irq_install_handler(int irq, irq_handler_t handler, void *data) {
    struct irq_handler *new_handler;
    
    new_handler = malloc(sizeof(struct irq_handler));
    new_handler->irq = irq;
    new_handler->handler = handler;         // Store function pointer
    new_handler->data = data;
    
    // Add to handler list
    new_handler->next = irq_handlers[irq];
    irq_handlers[irq] = new_handler;
    
    return 0;
}

// Execute interrupt handler via function pointer
void do_irq(int irq) {
    struct irq_handler *handler = irq_handlers[irq];
    
    while (handler) {
        handler->handler(irq, handler->data); // Function pointer call!
        handler = handler->next;
    }
}
```

---

## 7. Network Stack Function Pointers

### 7.1 Network Protocol Handler Function Pointers

```c
// Network protocol handler function pointer type
typedef void (*net_handler_t)(uchar *pkt, unsigned dest, struct in_addr sip,
                             unsigned src, unsigned len);

// Network protocol structure with function pointer
struct net_protocol {
    char *name;
    net_handler_t handler;                  // Function pointer!
    ushort port;
};

// TFTP protocol handler function
static void tftp_handler(uchar *pkt, unsigned dest, struct in_addr sip,
                        unsigned src, unsigned len) {
    __be16 proto;
    __be16 *s;
    
    if (dest != tftp_our_port)
        return;
        
    if (len < 2)
        return;
        
    len -= 2;
    s = (__be16 *)pkt;
    proto = *s++;
    pkt = (uchar *)s;
    
    switch (ntohs(proto)) {
    case TFTP_DATA:
        tftp_handle_data(pkt, len);
        break;
    case TFTP_ACK:
        tftp_handle_ack(pkt, len);
        break;
    case TFTP_ERROR:
        tftp_handle_error(pkt, len);
        break;
    }
}

// Network protocol registration with function pointer
static struct net_protocol tftp_protocol = {
    .name = "TFTP",
    .handler = tftp_handler,                // Function pointer assignment
    .port = 69,
};

// Network receive callback registration
static net_handler_t packet_handler = NULL;  // Global function pointer

void net_set_handler(net_handler_t f) {
    packet_handler = f;                     // Set function pointer
}

// Network packet processing via function pointer
int net_loop(enum proto_t protocol) {
    // ... network setup ...
    
    for (;;) {
        // Receive packet
        eth_rx();
        
        // Process packet via function pointer
        if (packet_handler)
            packet_handler(pkt, dest, sip, src, len);  // Function pointer call!
    }
}
```

### 7.2 Ethernet Driver Function Pointers

```c
// Ethernet operations structure with function pointers
struct eth_ops {
    int (*start)(struct udevice *dev);       // Function pointer
    int (*send)(struct udevice *dev, void *packet, int length); // Function pointer
    int (*recv)(struct udevice *dev, int flags, uchar **packetp); // Function pointer
    int (*free_pkt)(struct udevice *dev, uchar *packet, int length); // Function pointer
    void (*stop)(struct udevice *dev);       // Function pointer
    int (*mcast)(struct udevice *dev, const u8 *enetaddr, int join); // Function pointer
    int (*write_hwaddr)(struct udevice *dev); // Function pointer
    int (*read_rom_hwaddr)(struct udevice *dev); // Function pointer
};

// VisionFive2 ethernet driver operations
static const struct eth_ops starfive_eth_ops = {
    .start = starfive_eth_start,             // Function pointer assignment
    .send = starfive_eth_send,               // Function pointer assignment  
    .recv = starfive_eth_recv,               // Function pointer assignment
    .free_pkt = starfive_eth_free_pkt,       // Function pointer assignment
    .stop = starfive_eth_stop,               // Function pointer assignment
    .write_hwaddr = starfive_eth_write_hwaddr, // Function pointer assignment
};

// Ethernet function calls via function pointers
int eth_send(void *packet, int length) {
    struct udevice *current = eth_get_dev();
    const struct eth_ops *ops = eth_get_ops(current);
    
    if (!ops->send)
        return -ENOSYS;
        
    return ops->send(current, packet, length);  // Function pointer call!
}

int eth_recv(uchar **packetp) {
    struct udevice *current = eth_get_dev();
    const struct eth_ops *ops = eth_get_ops(current);
    
    if (!ops->recv)
        return -ENOSYS;
        
    return ops->recv(current, 0, packetp);      // Function pointer call!
}
```

---

## 8. File System API Callbacks

### 8.1 File System Operations Function Pointers

```c
// File system operations structure
struct fstype_info {
    char *name;
    int (*probe)(struct blk_desc *fs_dev_desc,
                struct disk_partition *fs_partition);  // Function pointer
    int (*ls)(const char *dirname);                    // Function pointer
    int (*exists)(const char *filename);               // Function pointer  
    int (*size)(const char *filename, loff_t *size);   // Function pointer
    int (*read)(const char *filename, void *buf,
               loff_t offset, loff_t len, loff_t *actread); // Function pointer
    int (*write)(const char *filename, void *buf,
                loff_t offset, loff_t len, loff_t *actwrite); // Function pointer
    void (*close)(void);                               // Function pointer
    int (*uuid)(char *uuid_str);                       // Function pointer
    int (*opendir)(const char *dirname, struct fs_dir_stream **dirsp); // Function pointer
    int (*readdir)(struct fs_dir_stream *dirs, struct fs_dirent **dentp); // Function pointer
    void (*closedir)(struct fs_dir_stream *dirs);      // Function pointer
    int (*unlink)(const char *filename);               // Function pointer
    int (*mkdir)(const char *dirname);                 // Function pointer
};

// FAT file system function pointer registration
static struct fstype_info fstypes[] = {
    {
        .name = "fat",
        .probe = fat_probe,                 // Function pointer
        .ls = fat_ls,                       // Function pointer
        .exists = fat_exists,               // Function pointer
        .size = fat_size,                   // Function pointer
        .read = fat_read_file,              // Function pointer
        .write = fat_write_file,            // Function pointer
        .uuid = fat_uuid,                   // Function pointer
        .opendir = fat_opendir,             // Function pointer
        .readdir = fat_readdir,             // Function pointer
        .closedir = fat_closedir,           // Function pointer
        .unlink = fat_unlink,               // Function pointer
        .mkdir = fat_mkdir,                 // Function pointer
    },
    {
        .name = "ext4",
        .probe = ext4_probe,                // Function pointer
        .ls = ext4_ls,                      // Function pointer
        .exists = ext4_exists,              // Function pointer
        .size = ext4_size,                  // Function pointer
        .read = ext4_read_file,             // Function pointer
        .write = ext4_write_file,           // Function pointer
        .uuid = ext4_uuid,                  // Function pointer
    },
};

// File system function calls via function pointers
int fs_read(const char *filename, ulong addr, loff_t offset, loff_t len,
           loff_t *actread) {
    struct fstype_info *info = fs_get_info(fs_type);
    
    if (!info->read)
        return -ENOSYS;
        
    return info->read(filename, (void *)addr, offset, len, actread); // Function pointer call!
}
```

---

## 9. SPL Function Pointers

### 9.1 SPL Image Loader Function Pointers

```c
// SPL image loader function pointer type
typedef int (*spl_load_image_fn_t)(struct spl_image_info *spl_image,
                                  struct spl_boot_device *bootdev);

// SPL image loader structure
struct spl_image_loader {
    const char *name;
    enum boot_device boot_device;
    spl_load_image_fn_t load_image;         // Function pointer!
};

// SPL MMC loader function
static int spl_mmc_load_image(struct spl_image_info *spl_image,
                             struct spl_boot_device *bootdev) {
    struct mmc *mmc = NULL;
    u32 boot_mode;
    int err = 0;
    
    err = spl_mmc_find_device(&mmc, bootdev->boot_device);
    if (err) return err;
    
    err = mmc_init(mmc);
    if (err) return err;
    
    boot_mode = spl_mmc_boot_mode(mmc, bootdev->boot_device);
    
    switch (boot_mode) {
    case MMCSD_MODE_EMMCBOOT:
        err = mmc_load_image_raw_sector(spl_image, mmc, 0x50);
        break;
    case MMCSD_MODE_FS:
        err = spl_mmc_do_fs_boot(spl_image, mmc, filename);
        break;
    default:
        err = -EINVAL;
    }
    
    return err;
}

// SPL loader registration using function pointer
SPL_LOAD_IMAGE_METHOD("MMC1", 0, BOOT_DEVICE_MMC1, spl_mmc_load_image);
SPL_LOAD_IMAGE_METHOD("MMC2", 0, BOOT_DEVICE_MMC2, spl_mmc_load_image);
SPL_LOAD_IMAGE_METHOD("SPI", 0, BOOT_DEVICE_SPI, spl_spi_load_image);

// SPL loader execution via function pointer
int spl_load_image(struct spl_image_info *spl_image,
                  struct spl_boot_device *bootdev) {
    struct spl_image_loader *loader;
    
    // Find loader for boot device
    loader = spl_ll_find_loader(bootdev->boot_device);
    if (!loader) {
        debug("Unsupported boot device %d\n", bootdev->boot_device);
        return -ENODEV;
    }
    
    debug("Trying to boot from %s\n", loader->name);
    
    // Load image via function pointer!
    return loader->load_image(spl_image, bootdev);  // Function pointer call!
}
```

---

## 10. Function Call Examples with Register Usage

### 10.1 RISC-V Function Call Convention in U-Boot

```assembly
# Example: U-Boot function call with register setup
# Calling sbi_set_timer(time_value) from U-Boot

# Function: sbi_set_timer implementation
sbi_set_timer:
    # Prepare SBI call registers
    li      a7, 0x54494D45      # a7 = SBI_EXT_TIME
    li      a6, 0               # a6 = SBI_EXT_TIME_SET_TIMER  
    # a0 already contains time_value (first argument)
    ecall                       # Call OpenSBI
    # Return: a0 = error, a1 = return value
    ret

# Function: timer_init calling sbi_set_timer
timer_init:
    addi    sp, sp, -16         # Allocate stack frame
    sd      ra, 8(sp)           # Save return address
    sd      s0, 0(sp)           # Save frame pointer
    
    # Call get_ticks()
    jal     ra, get_ticks       # Function pointer call
    mv      s0, a0              # Save current ticks
    
    # Calculate next timer value
    ld      a1, gd_arch_clk_freq(gp)  # Load clock frequency
    add     a0, s0, a1          # next_time = current + freq
    
    # Call sbi_set_timer via function pointer
    jal     ra, sbi_set_timer   # Function pointer call
    
    # Restore and return
    ld      ra, 8(sp)           # Restore return address
    ld      s0, 0(sp)           # Restore frame pointer
    addi    sp, sp, 16          # Deallocate stack frame
    ret
```

### 10.2 Function Pointer Call Example

```c
// Example showing function pointer usage in U-Boot command execution
void example_function_pointer_usage(void) {
    struct cmd_tbl *cmdtp;
    char *argv[] = {"bootm", "0x44000000"};
    int argc = 2;
    int flag = 0;
    
    // Find command via string lookup
    cmdtp = find_cmd("bootm");
    if (!cmdtp) {
        printf("Command not found\n");
        return;
    }
    
    printf("Calling command function at address: %p\n", cmdtp->cmd);
    
    // Call command function via pointer (this calls do_bootm)
    int result = cmdtp->cmd(cmdtp, flag, argc, argv);  // Function pointer call!
    
    printf("Command returned: %d\n", result);
}
```

### 10.3 Callback Chain Example

```c
// Example of callback chain during boot
void boot_callback_chain_example(void) {
    // 1. SPL loads and calls OpenSBI
    void (*opensbi_entry)(void) = (void (*)(void))0x40000000;
    opensbi_entry();  // Function pointer call to OpenSBI
    
    // 2. OpenSBI calls U-Boot (from sbi_hart_switch_mode)
    void (*uboot_entry)(ulong hartid, void *fdt) = (void (*)(ulong, void *))&_start;
    uboot_entry(0, fdt_addr);  // Function pointer call to U-Boot
    
    // 3. U-Boot calls kernel (from boot_jump_linux) 
    void (*kernel_entry)(ulong hartid, void *fdt) = (void (*)(ulong, void *))kernel_addr;
    kernel_entry(0, fdt_addr);  // Function pointer call to Linux
}
```

---

## Summary

U-Boot extensively uses **function pointers, callbacks, and API calls** throughout its architecture:

### **Key Function Pointer Categories:**

1. **Driver Model**: Device operations, probe/remove callbacks
2. **Command Framework**: Command execution, sub-commands
3. **Environment System**: Variable change callbacks
4. **Boot Flow**: Initialization sequences, OS boot functions
5. **Hardware Abstraction**: Timer, interrupt, network callbacks
6. **File Systems**: FS operations, device access
7. **SPL**: Image loaders, boot device handlers

### **Function Pointer Benefits:**

- **Modularity**: Easy to add new drivers, commands, file systems
- **Extensibility**: Runtime selection of implementations
- **Hardware Abstraction**: Same interface, different implementations
- **Maintainability**: Clean separation of concerns

### **RISC-V Specific Considerations:**

- **SBI Calls**: Function pointers used for OpenSBI integration
- **Hart Management**: Callbacks for multi-core initialization
- **Privilege Modes**: Function pointers handle mode transitions
- **Assembly Integration**: C function pointers called from assembly code

The VisionFive2 U-Boot implementation demonstrates sophisticated use of function pointers to create a flexible, modular bootloader that can adapt to different hardware configurations and boot scenarios while maintaining clean interfaces between components.

---

**Built with:** VisionFive2 SDK, U-Boot 2023.07, RISC-V RV64GC  
**Documentation Date:** November 1, 2025