# RISC-V Registers and Architecture Complete Reference

## Table of Contents
1. [Register Overview](#register-overview)
2. [General Purpose Registers](#general-purpose-registers)
3. [Control and Status Registers (CSRs)](#control-and-status-registers-csrs)
4. [CSR Detailed Documentation](#csr-detailed-documentation)
5. [Complete CSR Register Reference](#complete-csr-register-reference)
6. [System Call Mechanism](#system-call-mechanism)
7. [Exception and Interrupt Handling](#exception-and-interrupt-handling)
8. [Privilege Modes](#privilege-modes)
9. [Memory Protection](#memory-protection)
10. [Virtual Memory](#virtual-memory)

---

## Register Overview

### RISC-V ISA Characteristics
- RISC-V is a modular ISA with base integer instruction set + optional extensions
- RV32I (32-bit) and RV64I (64-bit) base integer instructions
- Extensions: M (multiply/divide), A (atomic), F/D (floating point), C (compressed)

---

## General Purpose Registers

### RISC-V Register File (32 Registers)

```
Register ABI Name  Description                      Saved by    Usage
-------- --------  -----------                      --------    -----
x0       zero      Hard-wired zero                  -           Always returns 0
x1       ra        Return address                   Caller      Function return PC
x2       sp        Stack pointer                    Callee      Current stack top
x3       gp        Global pointer                   -           Global data access
x4       tp        Thread pointer                   -           Thread-local storage
x5-x7    t0-t2     Temporary registers              Caller      Scratch values
x8       s0/fp     Saved register/Frame pointer     Callee      Stack frame base
x9       s1        Saved register                   Callee      Preserved across calls
x10-x11  a0-a1     Function args/return values      Caller      Args 1-2, return vals
x12-x17  a2-a7     Function arguments               Caller      Args 3-8
x18-x27  s2-s11    Saved registers                  Callee      Preserved across calls
x28-x31  t3-t6     Temporary registers              Caller      Scratch values
```

### Register Usage Convention

**Caller-Saved (Temporary) Registers:**
- `t0-t6` (x5-x7, x28-x31): Not preserved across function calls
- `a0-a7` (x10-x17): Argument/return value registers
- `ra` (x1): Return address (saved by caller if nested calls)

**Callee-Saved (Preserved) Registers:**
- `s0-s11` (x8-x9, x18-x27): Must be preserved by callee
- `sp` (x2): Stack pointer (must be preserved)

**Special Purpose:**
- `zero` (x0): Always reads as 0, writes are ignored
- `gp` (x3): Global pointer to .data section
- `tp` (x4): Thread-local storage pointer

### Register Usage in Different Contexts

**Function Arguments:**
```
a0 (x10): First argument / First return value
a1 (x11): Second argument / Second return value
a2 (x12): Third argument
a3 (x13): Fourth argument
a4 (x14): Fifth argument
a5 (x15): Sixth argument
a6 (x16): Seventh argument
a7 (x17): Eighth argument / Syscall number
```

**System Calls:**
```
a7 (x17): Syscall number
a0-a5 (x10-x15): Syscall arguments
a0-a1 (x10-x11): Syscall return values
```

---

## Control and Status Registers (CSRs)

### CSR Overview

CSRs are special registers that control CPU behavior, handle exceptions/interrupts, and manage privilege modes. They are accessed using special CSR instructions.

**CSR Address Format:**
```
Bits 11-10: Privilege Level
  00 = Unprivileged
  01 = Supervisor
  10 = Reserved
  11 = Machine

Bits 9-8: Read/Write Permission
  00 = Read/Write
  01 = Read/Write
  10 = Read/Write
  11 = Read-Only
```

### CSR Access Instructions

```asm
# Read CSR
csrr  rd, csr          # rd = csr (pseudo-instruction: csrrs rd, csr, x0)

# Write CSR
csrw  csr, rs          # csr = rs (pseudo-instruction: csrrw x0, csr, rs)

# Set bits in CSR
csrs  csr, rs          # csr = csr | rs (pseudo: csrrs x0, csr, rs)
csrsi csr, imm         # csr = csr | imm

# Clear bits in CSR
csrc  csr, rs          # csr = csr & ~rs (pseudo: csrrc x0, csr, rs)
csrci csr, imm         # csr = csr & ~imm

# Atomic read-write
csrrw rd, csr, rs      # tmp = csr; csr = rs; rd = tmp
csrrs rd, csr, rs      # tmp = csr; csr = csr | rs; rd = tmp
csrrc rd, csr, rs      # tmp = csr; csr = csr & ~rs; rd = tmp

# Atomic read-write immediate
csrrwi rd, csr, imm    # tmp = csr; csr = imm; rd = tmp
csrrsi rd, csr, imm    # tmp = csr; csr = csr | imm; rd = tmp
csrrci rd, csr, imm    # tmp = csr; csr = csr & ~imm; rd = tmp
```

**Common CSR Access Patterns:**
```asm
# Enable specific interrupt
    csrsi mie, (1 << 7)     # Enable timer interrupt (MTIE)

# Disable specific interrupt
    csrci mie, (1 << 7)     # Disable timer interrupt

# Read and modify
    csrr  t0, mstatus       # Read
    ori   t0, t0, 0x8       # Modify
    csrw  mstatus, t0       # Write back

# Atomic set and read previous value
    csrrsi t0, mstatus, 0x8 # Set MIE, return old mstatus

# Check if bit is set
    csrr  t0, mip
    andi  t0, t0, (1 << 7)  # Check MTIP
    bnez  t0, timer_pending
```

---

## CSR Detailed Documentation

### Machine Mode CSRs (Highest Privilege Level)

#### 1. **mstatus** - Machine Status Register (0x300)

**Purpose:** Controls and tracks the machine's operating state, interrupt enables, and privilege mode transitions.

**Key Bit Fields:**
```
Bit Position  Field Name   Description
-----------   ----------   -----------
0            UIE          User Interrupt Enable
1            SIE          Supervisor Interrupt Enable  
3            MIE          Machine Interrupt Enable (GLOBAL interrupt enable)
4            UPIE         User Previous Interrupt Enable
5            SPIE         Supervisor Previous Interrupt Enable
7            MPIE         Machine Previous Interrupt Enable (saved MIE)
8            SPP          Supervisor Previous Privilege (0=U-mode, 1=S-mode)
11-12        MPP          Machine Previous Privilege (00=U, 01=S, 11=M)
13-14        FS[1:0]      Floating-point unit Status
15-16        XS[1:0]      Extension Status
17           MPRV         Modify PRiVilege (memory access uses MPP privilege)
18-19        XS           User extension status
63           SD           State Dirty (FS or XS is dirty)
```

**mstatus.MIE (Bit 3) - Machine Interrupt Enable:**
```
Value: 0 = Interrupts DISABLED globally in machine mode
       1 = Interrupts ENABLED globally in machine mode

When to use:
- Bootloader/firmware initialization (disable during critical setup)
- Exception handler entry (hardware auto-clears this)
- Before returning from exception (restore original value)

Example:
    csrci mstatus, 0x8      # Clear bit 3 (MIE), disable interrupts
    csrsi mstatus, 0x8      # Set bit 3 (MIE), enable interrupts
```

**mstatus.MPIE (Bit 7) - Machine Previous Interrupt Enable:**
```
Purpose: Saves the value of MIE when trap occurs

Automatic behavior:
1. When trap occurs:  MPIE ← MIE (save interrupt state)
2. When mret executes: MIE ← MPIE (restore interrupt state)

Why it exists: Allows nested exception handling
- Outer exception had interrupts enabled (MIE=1)
- Trap occurs, hardware: MPIE=1 (saved), MIE=0 (disabled)
- Inner exception can be handled safely
- mret restores: MIE=1 (from MPIE)

Manual access:
    csrr t0, mstatus        # Read mstatus
    andi t0, t0, 0x80       # Check MPIE (bit 7)
```

**mstatus.MPP (Bits 11-12) - Machine Previous Privilege:**
```
Values: 00 = User mode (U-mode)
        01 = Supervisor mode (S-mode)
        11 = Machine mode (M-mode)

Purpose: Saves privilege level before trap

Automatic behavior:
1. When trap occurs:  MPP ← current_privilege_mode
2. When mret executes: Switch to privilege mode in MPP

Use case - Transitioning to lower privilege:
    # Currently in M-mode, want to enter S-mode
    li   t0, (1 << 11)      # MPP = 01 (S-mode)
    csrw mstatus, t0
    la   t0, supervisor_code
    csrw mepc, t0           # Return address
    mret                    # Privilege drops to S-mode, PC = mepc

Common pattern in bootloaders:
    # Boot in M-mode, jump to kernel in S-mode
    li   t0, (1 << 11) | (1 << 5)  # MPP=S-mode, MPIE=1
    csrw mstatus, t0
    la   t0, start_kernel
    csrw mepc, t0
    mret                    # Enter S-mode with interrupts enabled
```

**Where mstatus is used:**
1. **Bootloader (U-Boot, OpenSBI):** Initialize privilege modes, transition from M-mode to S-mode
2. **Kernel initialization:** Setup initial privilege state, configure exception handling
3. **Exception handlers:** Save/restore interrupt state, track privilege transitions

---

#### 2. **mepc** - Machine Exception Program Counter (0x341)

**Purpose:** Stores the address to return to after handling an exception/interrupt.

**Size:** Full register width (32-bit in RV32, 64-bit in RV64)

**Automatic Behavior:**
```
When trap occurs:
    mepc ← PC (address of instruction that caused exception)
           OR PC+4 (for interrupts - address of next instruction)

When mret executes:
    PC ← mepc (return to saved address)
```

**Key Points:**
- For **synchronous exceptions** (like illegal instruction, ecall): mepc = PC of faulting instruction
- For **asynchronous interrupts** (like timer, external): mepc = PC + 4 (next instruction)
- Bit 0 is always 0 (instructions are 2 or 4 byte aligned)
- Bit 1 may be 0 (depends on compressed instruction support)

**Usage Examples:**

```asm
# Example 1: Exception handler reading fault address
trap_handler:
    csrr t0, mepc           # Read the PC that caused the trap
    # If it was ecall, mepc points to the ecall instruction
    # Need to skip it before returning:
    addi t0, t0, 4          # Move to next instruction
    csrw mepc, t0           # Update return address
    # ... handle trap ...
    mret                    # Return to mepc (now past the ecall)

# Example 2: Bootloader jumping to kernel
    la   t0, start_kernel   # Kernel entry point address
    csrw mepc, t0           # Set return address
    mret                    # Jump to kernel (via mepc)

# Example 3: Checking fault address in handler
trap_handler:
    csrr a0, mepc           # Pass fault PC to C handler
    csrr a1, mcause         # Pass cause
    call handle_exception   # C function can analyze fault
```

**Where mepc is used:**
1. **Exception handlers** - Every trap handler reads/writes mepc
2. **System calls** - Increment mepc to skip ecall instruction
3. **Debuggers** - Read mepc to show where exception occurred
4. **Bootloaders** - Set mepc as jump target before mret
5. **Context switching** - Save/restore mepc in task state

---

#### 3. **mcause** - Machine Cause Register (0x342)

**Purpose:** Indicates why the trap occurred (interrupt or exception type).

**Format:**
```
Bit 63/31 (MSB)    Bits 62-0 / 30-0
[Interrupt]        [Exception Code]

If Interrupt=1: Asynchronous interrupt
If Interrupt=0: Synchronous exception
```

**Exception Codes (Interrupt=0):**
```
Code  Name                          Description
----  ----                          -----------
0     Instruction address misaligned  PC not properly aligned
1     Instruction access fault        Cannot fetch instruction
2     Illegal instruction             Invalid/unsupported instruction
3     Breakpoint                      ebreak instruction executed
4     Load address misaligned         Load from unaligned address
5     Load access fault               Cannot read memory
6     Store address misaligned        Store to unaligned address
7     Store access fault              Cannot write memory
8     Environment call from U-mode    ecall in User mode
9     Environment call from S-mode    ecall in Supervisor mode
11    Environment call from M-mode    ecall in Machine mode
12    Instruction page fault          Virtual memory: instruction fetch failed
13    Load page fault                 Virtual memory: load failed
15    Store page fault                Virtual memory: store failed
```

**Interrupt Codes (Interrupt=1):**
```
Code  Name                          Description
----  ----                          -----------
3     Machine software interrupt      Inter-processor interrupt (IPI)
7     Machine timer interrupt         Timer expired
11    Machine external interrupt      External device interrupt
```

**Usage Examples:**

```asm
# Example 1: Dispatch different exception types
trap_handler:
    csrr t0, mcause         # Read trap cause
    bltz t0, handle_interrupt   # If MSB=1, it's an interrupt
    
handle_exception:
    # Check exception code
    li   t1, 8              # Code 8 = ecall from U-mode
    beq  t0, t1, handle_syscall
    
    li   t1, 2              # Code 2 = illegal instruction
    beq  t0, t1, handle_illegal
    
    j    unknown_exception

handle_interrupt:
    # Clear MSB to get interrupt code
    slli t0, t0, 1          # Shift out MSB
    srli t0, t0, 1
    
    li   t1, 7              # Code 7 = timer interrupt
    beq  t0, t1, handle_timer
    
    li   t1, 11             # Code 11 = external interrupt
    beq  t0, t1, handle_external
    
    j    unknown_interrupt
```

**Where mcause is used:**
1. **Exception dispatchers** - Route to correct handler
2. **System call handlers** - Verify it's actually a syscall
3. **Debuggers** - Display exception type to user
4. **Error logging** - Record what caused the fault
5. **Kernel panic** - Print cause before halting

---

#### 4. **mtvec** - Machine Trap Vector (0x305)

**Purpose:** Holds the base address of the trap handler. When any trap occurs, the CPU jumps here.

**Format:**
```
Bits 63/31-2      Bits 1-0
[BASE]            [MODE]

MODE values:
  00 = Direct mode    - All traps jump to BASE
  01 = Vectored mode  - Interrupts jump to BASE + 4*cause
                        Exceptions jump to BASE
```

**Direct Mode (MODE=00):**
```
All traps (interrupts and exceptions) jump to the same address:
    PC ← BASE (mtvec with lower 2 bits cleared)

Example:
    la   t0, trap_handler   # Single handler for everything
    csrw mtvec, t0          # MODE=00 (automatic if address aligned)
```

**Vectored Mode (MODE=01):**
```
Interrupts: PC ← BASE + 4 * mcause_code
Exceptions: PC ← BASE

Allows separate handlers for each interrupt type.

Example:
    la   t0, trap_vector_table
    ori  t0, t0, 1          # Set MODE=01
    csrw mtvec, t0

trap_vector_table:
    j    exception_handler  # Offset 0: All exceptions
    j    unhandled          # Offset 4: (not used)
    j    unhandled          # Offset 8: (not used)
    j    machine_sw_int     # Offset 12: Code 3 * 4
    j    unhandled          # Offset 16
    j    unhandled          # Offset 20
    j    unhandled          # Offset 24
    j    machine_timer_int  # Offset 28: Code 7 * 4
```

**Where mtvec is used:**
1. **Bootloader initialization** - First setup of exception handling
2. **Kernel boot** - Switch from bootloader handler to kernel handler
3. **Context switching** - Some OSes change per-process
4. **Debugging** - Temporary handler for testing
5. **Interrupt controller setup** - Configure vectored mode

---

#### 5. **mie** - Machine Interrupt Enable Register (0x304)

**Purpose:** Individual enable bits for each interrupt source. Works with mstatus.MIE as two-level control.

**Format:**
```
Bit  Name   Description
---  ----   -----------
3    MSIE   Machine Software Interrupt Enable
7    MTIE   Machine Timer Interrupt Enable
11   MEIE   Machine External Interrupt Enable
```

**Two-Level Interrupt Control:**
```
For interrupt to fire, BOTH must be true:
1. mstatus.MIE = 1 (global enable)
2. Specific bit in mie = 1 (individual enable)

Think of it as:  interrupt_enabled = mstatus.MIE && mie.MTIE
```

**Usage Examples:**

```asm
# Enable timer interrupts
    li   t0, (1 << 7)       # Bit 7 = MTIE
    csrs mie, t0            # Set MTIE (enable timer)
    csrsi mstatus, 0x8      # Set MIE (global enable)

# Disable only external interrupts (keep others)
    li   t0, (1 << 11)      # Bit 11 = MEIE
    csrc mie, t0            # Clear MEIE (disable external)

# Enable all machine-mode interrupts
    li   t0, 0x888          # Bits 3, 7, 11 (MSI, MTI, MEI)
    csrs mie, t0            # Enable all three
    csrsi mstatus, 0x8      # Global enable

# Critical section (disable all interrupts temporarily)
    csrci mstatus, 0x8      # Clear MIE (fast disable)
    # ... critical code ...
    csrsi mstatus, 0x8      # Set MIE (re-enable)
```

**Where mie is used:**
1. **Timer initialization** - Enable timer for scheduling
2. **Interrupt controller setup** - Configure which sources are active
3. **Power management** - Disable unused interrupt sources
4. **Critical sections** - Fine-grained interrupt control
5. **Device drivers** - Enable/disable device interrupts

---

#### 6. **mip** - Machine Interrupt Pending Register (0x344)

**Purpose:** Read-only bits showing which interrupts are pending (waiting to be handled).

**Format:** (Same layout as mie)
```
Bit  Name   Description
---  ----   -----------
3    MSIP   Machine Software Interrupt Pending
7    MTIP   Machine Timer Interrupt Pending
11   MEIP   Machine External Interrupt Pending
```

**Reading Pending Interrupts:**
```asm
# Check if timer interrupt is pending
    csrr t0, mip            # Read pending interrupts
    andi t0, t0, 0x80       # Check bit 7 (MTIP)
    bnez t0, timer_pending

# Poll for any interrupt
    csrr t0, mip
    andi t0, t0, 0x888      # Check all three sources
    beqz t0, no_interrupts_pending

# Wait for specific interrupt (polling)
wait_for_timer:
    csrr t0, mip
    andi t0, t0, 0x80       # Check MTIP
    beqz t0, wait_for_timer
    # Timer fired
```

**Software Interrupts (Inter-Processor Interrupts):**
```asm
# Send IPI to another hart
    li   t0, 1
    li   t1, CLINT_MSIP_BASE    # Platform-specific register
    sw   t0, 0(t1)              # Set MSIP for target hart
    # This will set mip.MSIP in the target hart

# Clear software interrupt (in handler)
    li   t0, CLINT_MSIP_BASE
    sw   zero, 0(t0)            # Clear MSIP
```

**Where mip is used:**
1. **Interrupt handlers** - Check which interrupt fired
2. **Polling loops** - Wait for interrupt without enabling
3. **IPI mechanism** - Multi-core synchronization
4. **Debugging** - Check if interrupts are arriving
5. **Nested handlers** - Prioritize multiple pending interrupts

---

#### 7. **mtval** - Machine Trap Value (0x343)

**Purpose:** Provides additional information about the trap.

**Contents vary by exception type:**
```
Exception Type                Value in mtval
--------------                --------------
Instruction address misaligned   Faulting address
Instruction access fault         Faulting address
Illegal instruction              The instruction bits
Breakpoint                       Program counter
Load address misaligned          Faulting address
Load access fault                Faulting address
Store address misaligned         Faulting address
Store access fault               Faulting address
Page fault                       Faulting virtual address
```

---

### Supervisor Mode CSRs (Used by Operating System)

#### 8. **sstatus** - Supervisor Status Register (0x100)

**Purpose:** Supervisor-level view of mstatus (subset of fields).

**Key fields:**
```
Bit  Name   Description
---  ----   -----------
1    SIE    Supervisor Interrupt Enable
5    SPIE   Supervisor Previous Interrupt Enable
8    SPP    Supervisor Previous Privilege (0=U, 1=S)
18   SUM    Permit Supervisor User Memory access
19   MXR    Make eXecutable Readable
```

Similar to mstatus.MIE/MPIE/MPP but for supervisor mode.

---

#### 9. **sepc** - Supervisor Exception Program Counter (0x141)

**Purpose:** Like mepc, but for supervisor-level traps. Used by OS kernel.

```asm
# Kernel syscall handler
syscall_handler:
    csrr t0, sepc           # Get user PC (where ecall was)
    addi t0, t0, 4          # Skip the ecall instruction
    csrw sepc, t0           # Update return address
    # ... handle syscall ...
    sret                    # Return to user (sepc)
```

---

#### 10. **scause** - Supervisor Cause Register (0x142)

**Purpose:** Like mcause, but for supervisor-level traps.

```asm
# Kernel trap dispatcher
    csrr t0, scause
    bltz t0, s_interrupt
    # Handle exception
    li   t1, 8              # Syscall from user
    beq  t0, t1, do_syscall
```

---

#### 11. **stvec** - Supervisor Trap Vector (0x105)

**Purpose:** Like mtvec, but for supervisor-level trap handler.

```asm
# Kernel initialization
    la   t0, kernel_trap_entry
    csrw stvec, t0          # Set kernel trap handler
```

---

#### 12. **satp** - Supervisor Address Translation and Protection (0x180)

**Purpose:** Controls virtual memory (page table base and mode).

**Format (RV64):**
```
Bits 63-60: MODE (0=Bare, 8=Sv39, 9=Sv48, 10=Sv57)
Bits 59-44: ASID (Address Space ID)
Bits 43-0:  PPN (Physical Page Number of root page table)
```

**Virtual Memory Modes:**
```
MODE  Name   Description
----  ----   -----------
0     Bare   No translation (physical addressing)
8     Sv39   39-bit virtual addressing (512GB address space)
9     Sv48   48-bit virtual addressing (256TB address space)
10    Sv57   57-bit virtual addressing (128PB address space)
```

```asm
# Enable paging
    li   t0, (8 << 60)      # Sv39 mode
    la   t1, page_table
    srli t1, t1, 12         # Convert to PPN
    or   t0, t0, t1
    csrw satp, t0
    sfence.vma              # Flush TLB
```

---

## Complete CSR Register Reference

### User-Level CSRs (Unprivileged)

**Floating-Point CSRs:**

| Address | Name    | Abbreviation | Description | R/W |
|---------|---------|--------------|-------------|-----|
| 0x001   | fflags  | FFLags       | Floating-Point Accrued Exceptions | RW |
| 0x002   | frm     | FRM          | Floating-Point Dynamic Rounding Mode | RW |
| 0x003   | fcsr    | FCSR         | Floating-Point Control and Status Register | RW |

**Counters and Timers:**

| Address | Name       | Abbreviation | Description | R/W |
|---------|------------|--------------|-------------|-----|
| 0xC00   | cycle      | CYCLE        | Cycle counter (lower 32/64 bits) | RO |
| 0xC01   | time       | TIME         | Real-time clock (lower 32/64 bits) | RO |
| 0xC02   | instret    | INSTRET      | Instructions retired counter (lower 32/64 bits) | RO |
| 0xC03   | hpmcounter3 | HPMCTR3     | Performance monitoring counter 3 | RO |
| 0xC04   | hpmcounter4 | HPMCTR4     | Performance monitoring counter 4 | RO |
| ...     | ...        | ...          | ... | ... |
| 0xC1F   | hpmcounter31 | HPMCTR31   | Performance monitoring counter 31 | RO |
| 0xC80   | cycleh     | CYCLEH       | Upper 32 bits of cycle (RV32 only) | RO |
| 0xC81   | timeh      | TIMEH        | Upper 32 bits of time (RV32 only) | RO |
| 0xC82   | instreth   | INSTRETH     | Upper 32 bits of instret (RV32 only) | RO |
| 0xC83   | hpmcounter3h | HPMCTR3H   | Upper 32 bits of hpmcounter3 (RV32) | RO |
| ...     | ...        | ...          | ... | ... |
| 0xC9F   | hpmcounter31h | HPMCTR31H | Upper 32 bits of hpmcounter31 (RV32) | RO |

**Example Usage:**
```asm
# Read cycle counter
    csrr a0, cycle          # Get current cycle count
    
# Read real-time clock
    csrr a0, time           # Get time value
    
# RV32: Read 64-bit cycle counter
    csrr a0, cycle          # Lower 32 bits
    csrr a1, cycleh         # Upper 32 bits
```

---

### Supervisor-Level CSRs (OS Kernel)

**Supervisor Trap Setup:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x100   | sstatus   | SSTATUS      | Supervisor status register | RW |
| 0x104   | sie       | SIE          | Supervisor interrupt-enable register | RW |
| 0x105   | stvec     | STVEC        | Supervisor trap handler base address | RW |
| 0x106   | scounteren | SCOUNTEREN  | Supervisor counter enable | RW |

**sie Bit Fields:**
```
Bit  Name   Description
---  ----   -----------
0    USIE   User Software Interrupt Enable
1    SSIE   Supervisor Software Interrupt Enable
4    UTIE   User Timer Interrupt Enable
5    STIE   Supervisor Timer Interrupt Enable
8    UEIE   User External Interrupt Enable
9    SEIE   Supervisor External Interrupt Enable
```

**Supervisor Trap Handling:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x140   | sscratch  | SSCRATCH     | Scratch register for supervisor trap handler | RW |
| 0x141   | sepc      | SEPC         | Supervisor exception program counter | RW |
| 0x142   | scause    | SCAUSE       | Supervisor trap cause | RW |
| 0x143   | stval     | STVAL        | Supervisor bad address or instruction | RW |
| 0x144   | sip       | SIP          | Supervisor interrupt pending | RW |

**Supervisor Protection and Translation:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x180   | satp      | SATP         | Supervisor address translation and protection | RW |

---

### Machine-Level CSRs (Highest Privilege)

**Machine Information Registers:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0xF11   | mvendorid | MVENDORID    | Vendor ID | RO |
| 0xF12   | marchid   | MARCHID      | Architecture ID | RO |
| 0xF13   | mimpid    | MIMPID       | Implementation ID | RO |
| 0xF14   | mhartid   | MHARTID      | Hardware thread ID | RO |
| 0xF15   | mconfigptr | MCONFIGPTR  | Pointer to configuration data structure | RO |

**Machine Trap Setup:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x300   | mstatus   | MSTATUS      | Machine status register | RW |
| 0x301   | misa      | MISA         | ISA and extensions | RW |
| 0x302   | medeleg   | MEDELEG      | Machine exception delegation register | RW |
| 0x303   | mideleg   | MIDELEG      | Machine interrupt delegation register | RW |
| 0x304   | mie       | MIE          | Machine interrupt-enable register | RW |
| 0x305   | mtvec     | MTVEC        | Machine trap-handler base address | RW |
| 0x306   | mcounteren | MCOUNTEREN  | Machine counter enable | RW |
| 0x310   | mstatush  | MSTATUSH     | Additional machine status (RV32 only) | RW |

**misa Bit Fields:**
```
Bits 25-0: Extensions (A=0, B=1, C=2, D=3, ..., Z=25)
  Bit 0 (A): Atomic extension
  Bit 2 (C): Compressed instruction extension
  Bit 3 (D): Double-precision floating-point
  Bit 5 (F): Single-precision floating-point
  Bit 8 (I): RV32I/64I/128I base ISA
  Bit 12 (M): Integer multiply/divide extension
  Bit 18 (S): Supervisor mode implemented
  Bit 20 (U): User mode implemented
  Bit 21 (V): Vector extension
Bits 63-62 (RV64): MXL (Machine XLEN)
  1 = 32-bit, 2 = 64-bit, 3 = 128-bit
```

**Machine Trap Handling:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x340   | mscratch  | MSCRATCH     | Scratch register for machine trap handlers | RW |
| 0x341   | mepc      | MEPC         | Machine exception program counter | RW |
| 0x342   | mcause    | MCAUSE       | Machine trap cause | RW |
| 0x343   | mtval     | MTVAL        | Machine bad address or instruction | RW |
| 0x344   | mip       | MIP          | Machine interrupt pending | RW |
| 0x34A   | mtinst    | MTINST       | Machine trap instruction (transformed) | RW |
| 0x34B   | mtval2    | MTVAL2       | Machine bad guest physical address | RW |

**Machine Memory Protection:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x3A0   | pmpcfg0   | PMPCFG0      | Physical memory protection config 0 | RW |
| 0x3A1   | pmpcfg1   | PMPCFG1      | Physical memory protection config 1 | RW |
| 0x3A2   | pmpcfg2   | PMPCFG2      | Physical memory protection config 2 | RW |
| 0x3A3   | pmpcfg3   | PMPCFG3      | Physical memory protection config 3 | RW |
| 0x3B0   | pmpaddr0  | PMPADDR0     | Physical memory protection address 0 | RW |
| 0x3B1   | pmpaddr1  | PMPADDR1     | Physical memory protection address 1 | RW |
| ...     | ...       | ...          | ... | ... |
| 0x3BF   | pmpaddr15 | PMPADDR15    | Physical memory protection address 15 | RW |

**Machine Counters/Timers:**

| Address | Name        | Abbreviation | Description | R/W |
|---------|-------------|--------------|-------------|-----|
| 0xB00   | mcycle      | MCYCLE       | Machine cycle counter | RW |
| 0xB02   | minstret    | MINSTRET     | Machine instructions-retired counter | RW |
| 0xB03   | mhpmcounter3 | MHPMCTR3    | Machine performance-monitoring counter 3 | RW |
| ...     | ...         | ...          | ... | ... |
| 0xB1F   | mhpmcounter31 | MHPMCTR31  | Machine performance-monitoring counter 31 | RW |
| 0xB80   | mcycleh     | MCYCLEH      | Upper 32 bits of mcycle (RV32 only) | RW |
| 0xB82   | minstreth   | MINSTRETH    | Upper 32 bits of minstret (RV32 only) | RW |

**Machine Counter Setup:**

| Address | Name        | Abbreviation | Description | R/W |
|---------|-------------|--------------|-------------|-----|
| 0x320   | mcountinhibit | MCNTINHBT  | Machine counter-inhibit register | RW |
| 0x323   | mhpmevent3  | MHPMEVT3     | Machine performance-monitoring event 3 | RW |
| ...     | ...         | ...          | ... | ... |
| 0x33F   | mhpmevent31 | MHPMEVT31    | Machine performance-monitoring event 31 | RW |

---

### Debug/Trace Registers

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x7A0   | tselect   | TSELECT      | Debug/Trace trigger register select | RW |
| 0x7A1   | tdata1    | TDATA1       | First Debug/Trace trigger data register | RW |
| 0x7A2   | tdata2    | TDATA2       | Second Debug/Trace trigger data register | RW |
| 0x7A3   | tdata3    | TDATA3       | Third Debug/Trace trigger data register | RW |
| 0x7B0   | dcsr      | DCSR         | Debug control and status register | RW |
| 0x7B1   | dpc       | DPC          | Debug PC | RW |
| 0x7B2   | dscratch0 | DSCRATCH0    | Debug scratch register 0 | RW |
| 0x7B3   | dscratch1 | DSCRATCH1    | Debug scratch register 1 | RW |

---

### Hypervisor Extension CSRs (H-extension)

**Hypervisor Trap Setup:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x600   | hstatus   | HSTATUS      | Hypervisor status register | RW |
| 0x602   | hedeleg   | HEDELEG      | Hypervisor exception delegation register | RW |
| 0x603   | hideleg   | HIDELEG      | Hypervisor interrupt delegation register | RW |
| 0x604   | hie       | HIE          | Hypervisor interrupt-enable register | RW |
| 0x606   | hcounteren | HCOUNTEREN  | Hypervisor counter enable | RW |
| 0x607   | hgeie     | HGEIE        | Hypervisor guest external interrupt enable | RW |

**Hypervisor Trap Handling:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x643   | htval     | HTVAL        | Hypervisor bad guest physical address | RW |
| 0x644   | hip       | HIP          | Hypervisor interrupt pending | RW |
| 0x645   | hvip      | HVIP         | Hypervisor virtual interrupt pending | RW |
| 0x64A   | htinst    | HTINST       | Hypervisor trap instruction (transformed) | RW |
| 0xE12   | hgeip     | HGEIP        | Hypervisor guest external interrupt pending | RO |

**Hypervisor Protection and Translation:**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x680   | hgatp     | HGATP        | Hypervisor guest address translation/protection | RW |

**Virtual Supervisor Registers (accessed from VS-mode):**

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x200   | vsstatus  | VSSTATUS     | Virtual supervisor status register | RW |
| 0x204   | vsie      | VSIE         | Virtual supervisor interrupt-enable | RW |
| 0x205   | vstvec    | VSTVEC       | Virtual supervisor trap handler base | RW |
| 0x240   | vsscratch | VSSCRATCH    | Virtual supervisor scratch register | RW |
| 0x241   | vsepc     | VSEPC        | Virtual supervisor exception PC | RW |
| 0x242   | vscause   | VSCAUSE      | Virtual supervisor trap cause | RW |
| 0x243   | vstval    | VSTVAL       | Virtual supervisor bad address | RW |
| 0x244   | vsip      | VSIP         | Virtual supervisor interrupt pending | RW |
| 0x280   | vsatp     | VSATP        | Virtual supervisor address translation | RW |

---

### Vector Extension CSRs (V-extension)

| Address | Name      | Abbreviation | Description | R/W |
|---------|-----------|--------------|-------------|-----|
| 0x008   | vstart    | VSTART       | Vector start position | RW |
| 0x009   | vxsat     | VXSAT        | Vector fixed-point saturate flag | RW |
| 0x00A   | vxrm      | VXRM         | Vector fixed-point rounding mode | RW |
| 0x00F   | vcsr      | VCSR         | Vector control and status register | RW |
| 0xC20   | vl        | VL           | Vector length | RO |
| 0xC21   | vtype     | VTYPE        | Vector data type register | RO |
| 0xC22   | vlenb     | VLENB        | Vector register length in bytes | RO |

---

### Custom CSRs (Implementation-Specific)

| Address Range | Description |
|---------------|-------------|
| 0x800-0x8FF   | Custom read-write (URW) |
| 0xCC0-0xCFF   | Custom read-only (URO) |
| 0x9C0-0x9FF   | Custom read-write supervisor (SRW) |
| 0xDC0-0xDFF   | Custom read-only supervisor (SRO) |
| 0xBC0-0xBFF   | Custom read-write machine (MRW) |
| 0xFC0-0xFFF   | Custom read-only machine (MRO) |

---

## System Call Mechanism

### Understanding System Call Number 93 (exit)

The number **93** is the system call number for `exit` in the RISC-V Linux ABI.

**What is a System Call Number?**
- System calls are kernel functions that user programs can invoke
- Each syscall has a unique number to identify it
- The kernel uses this number to dispatch to the correct handler
- Different architectures have different syscall numbers

**Why 93 for exit?**
- RISC-V Linux follows its own syscall numbering scheme
- On x86-64: exit = 60
- On ARM64: exit = 93
- On RISC-V: exit = 93 (matches ARM64 for compatibility)

**Common RISC-V Linux System Call Numbers:**
```c
#define __NR_io_setup              0
#define __NR_io_destroy            1
#define __NR_read                 63  // Read from file descriptor
#define __NR_write                64  // Write to file descriptor  
#define __NR_openat               56  // Open file (at directory)
#define __NR_close                57  // Close file descriptor
#define __NR_lseek                62  // Reposition file offset
#define __NR_exit                 93  // Exit process
#define __NR_exit_group           94  // Exit all threads in process
#define __NR_kill                129  // Send signal to process
#define __NR_getpid              172  // Get process ID
#define __NR_brk                 214  // Change data segment size
#define __NR_mmap                222  // Map memory
#define __NR_munmap              215  // Unmap memory
#define __NR_clone               220  // Create new process/thread
```

---

## Exception and Interrupt Handling

### What Happens During `ecall` Execution

When the CPU executes the `ecall` instruction, here's the complete flow:

**1. Hardware CPU State Transition (Atomic Operation):**
```
Before ecall:
- CPU is in User mode (U-mode)
- PC points to ecall instruction
- All registers have current values (a7=93, a0=0, etc.)

During ecall (hardware does this automatically):
- Privilege level changes: U-mode -> S-mode (Supervisor mode)
- mepc CSR ← PC + 4 (save return address - next instruction after ecall)
- mcause CSR ← 8 (exception code 8 = "Environment call from U-mode")
- mstatus.MPP ← current privilege mode (to remember where we came from)
- mstatus.MPIE ← mstatus.MIE (save interrupt enable state)
- mstatus.MIE ← 0 (disable interrupts during handler)
- PC ← mtvec (jump to trap/exception handler address)

After ecall:
- CPU is now in Supervisor/Machine mode
- PC now points to kernel's trap handler (set in mtvec register)
- All general-purpose registers (a0-a7, etc.) remain unchanged!
```

**2. Kernel Entry - Saving User Context:**
```asm
# This code is in linux/arch/riscv/kernel/entry.S
# CPU jumped here because mtvec points to this address

handle_exception:
    # At this point: a7=93, a0=0 (from user program)
    
    # Swap to kernel stack
    csrrw sp, sscratch, sp     # Swap sp with kernel stack pointer
    
    # Allocate space for pt_regs structure (all registers)
    addi sp, sp, -PT_SIZE      # PT_SIZE = 256+ bytes
    
    # Save ALL user registers to kernel stack
    sd   x1,  PT_RA(sp)        # Save return address
    sd   x2,  PT_SP(sp)        # Save user stack pointer
    sd   x3,  PT_GP(sp)        # Save global pointer
    sd   x5,  PT_T0(sp)        # Save temporaries t0-t6
    sd   x10, PT_A0(sp)        # Save a0 (exit code = 0)
    sd   x11, PT_A1(sp)        # Save a1-a7
    sd   x17, PT_A7(sp)        # Save a7 (syscall number = 93)
    # ... saves all 32 registers ...
    
    # Save CSR registers that have trap information
    csrr t0, sepc              # Exception PC (user's PC after ecall)
    csrr t1, sstatus           # Status before trap
    csrr t2, sbadaddr          # Faulting address (if any)
    csrr t3, scause            # Cause of trap
    sd   t0, PT_EPC(sp)
    sd   t1, PT_STATUS(sp)
    sd   t2, PT_BADADDR(sp)
    sd   t3, PT_CAUSE(sp)
    
    # Now user context is completely saved!
```

**3. Syscall Dispatch - Finding the Handler:**
```asm
    # Check if this is a syscall (scause should indicate syscall)
    csrr t0, scause
    li   t1, 8                 # 8 = syscall from U-mode
    bne  t0, t1, not_syscall   # If not syscall, handle differently
    
    # This IS a syscall!
    # Bounds check: is syscall number valid?
    li   t0, __NR_syscalls     # Total number of syscalls
    bgeu a7, t0, invalid_syscall
    
    # Load function pointer from sys_call_table
    la   t0, sys_call_table    # Base address of syscall table
    slli t1, a7, 3             # t1 = a7 * 8 (each pointer is 8 bytes)
    add  t0, t0, t1            # t0 = &sys_call_table[93]
    ld   t0, 0(t0)             # t0 = sys_call_table[93] (function pointer)
    
    # Call the syscall handler
    jalr t0                    # Jump to sys_exit(a0=0)
```

**4. Inside `sys_exit()` - C Function in Kernel:**
```c
// This is now C code in linux/kernel/exit.c

SYSCALL_DEFINE1(exit, int, error_code)  // Creates sys_exit
{
    // error_code = 0 (from a0 register)
    
    do_exit((error_code & 0xff) << 8);
    // This function:
    // 1. Releases all resources (memory, file descriptors, etc.)
    // 2. Sends SIGCHLD to parent process
    // 3. Reparents child processes to init
    // 4. Marks task_struct as TASK_DEAD
    // 5. Calls schedule() to switch to another process
    // 6. NEVER RETURNS (this process is done!)
}
```

**5. Process Termination - What Happens Next:**
```
At this point:
- Current process is marked TASK_DEAD
- All resources are freed
- Parent process is notified
- Kernel scheduler is invoked

Scheduler:
- Picks another process to run
- Loads that process's context (registers, PC, page tables)
- Returns from trap handler into that OTHER process
- The original process never gets CPU time again

Important: The user program that called ecall NEVER returns!
The "return" from the syscall handler goes to a DIFFERENT process.
This is why exit() is special - it's the only syscall that doesn't return.
```

---

## Privilege Modes

RISC-V defines three privilege modes:

### Privilege Mode Hierarchy

```
Machine Mode (M-mode)      - Highest privilege, full hardware access
    ↓ delegates to
Supervisor Mode (S-mode)   - OS kernel, virtual memory management
    ↓ delegates to
User Mode (U-mode)         - Applications, restricted access
```

**Privilege Levels:**
```
Mode  Encoding  Typical Use
----  --------  -----------
U     00        User applications
S     01        Operating system kernel
M     11        Bootloader, firmware, machine-level operations
```

### Complete Privilege Mode Transition Example

```asm
# Machine mode → Supervisor mode → User mode

# Starting in M-mode (bootloader)
machine_mode_init:
    # Setup PMP (allow all)
    li   t0, 0x0F           # R=1, W=1, X=1, A=01 (TOR)
    csrw pmpcfg0, t0
    li   t0, -1
    csrw pmpaddr0, t0
    
    # Delegate interrupts and exceptions to S-mode
    li   t0, 0xFFFF
    csrw medeleg, t0        # All exceptions to S-mode
    csrw mideleg, t0        # All interrupts to S-mode
    
    # Transition to S-mode
    li   t0, (1 << 11) | (1 << 5)  # MPP=S-mode, MPIE=1
    csrw mstatus, t0
    la   t0, supervisor_mode_init
    csrw mepc, t0
    mret                    # → S-mode

# Now in S-mode (kernel)
supervisor_mode_init:
    # Setup S-mode trap handler
    la   t0, kernel_trap
    csrw stvec, t0
    
    # Enable S-mode interrupts
    li   t0, 0x222          # SSIE, STIE, SEIE
    csrw sie, t0
    
    # Setup virtual memory
    la   t0, page_table
    srli t0, t0, 12
    li   t1, (8 << 60)      # Sv39
    or   t0, t0, t1
    csrw satp, t0
    sfence.vma
    
    # Transition to U-mode
    li   t0, (1 << 5)       # SPP=U-mode, SPIE=1
    csrw sstatus, t0
    la   t0, user_program
    csrw sepc, t0
    sret                    # → U-mode

# Now in U-mode (application)
user_program:
    # Can only access user-level CSRs
    csrr a0, cycle          # OK: Read cycle counter
    # csrr a0, mstatus      # ILLEGAL: Cannot access M-mode CSR
    
    # Make syscall
    li   a7, 64             # sys_write
    ecall                   # Trap to S-mode
```

---

## Memory Protection

### PMP (Physical Memory Protection)

PMP allows M-mode to restrict memory access for S-mode and U-mode.

**pmpcfg format (each entry is 8 bits):**
```
Bit 7:   L (Lock - prevent further modification)
Bits 4-3: A (Address matching mode)
  00 = OFF (disabled)
  01 = TOR (Top of range)
  10 = NA4 (Naturally aligned 4-byte)
  11 = NAPOT (Naturally aligned power-of-2)
Bit 2:   X (eXecute permission)
Bit 1:   W (Write permission)
Bit 0:   R (Read permission)
```

**Example - Allow all memory access:**
```asm
# Setup PMP to allow all (common in bootloader)
    li   t0, 0x1F1F1F1F     # R=1, W=1, X=1, A=11 (NAPOT) for all 4 entries
    csrw pmpcfg0, t0
    
    li   t0, -1             # All 1s = entire address space
    csrw pmpaddr0, t0       # Region 0 covers everything
```

---

## Virtual Memory

### Page Table Formats

**Sv39 (39-bit Virtual Addressing):**
```
Virtual Address: [38:30] VPN[2] | [29:21] VPN[1] | [20:12] VPN[0] | [11:0] Offset
- 512 GB address space
- 3-level page table
- Page size: 4KB
```

**Sv48 (48-bit Virtual Addressing):**
```
Virtual Address: [47:39] VPN[3] | [38:30] VPN[2] | [29:21] VPN[1] | [20:12] VPN[0] | [11:0] Offset
- 256 TB address space
- 4-level page table
- Page size: 4KB
```

**Page Table Entry (PTE) Format:**
```
Bits 63-54: Reserved
Bits 53-10: PPN (Physical Page Number)
Bits 9-8:   RSW (Reserved for software)
Bit 7:      D (Dirty - page has been written)
Bit 6:      A (Accessed - page has been accessed)
Bit 5:      G (Global - mapping exists in all address spaces)
Bit 4:      U (User - accessible by user mode)
Bit 3:      X (eXecutable)
Bit 2:      W (Writable)
Bit 1:      R (Readable)
Bit 0:      V (Valid)
```

---

## CSR Summary Tables

### CSR Quick Reference by Category

**Identification (Read-Only):**
- mvendorid, marchid, mimpid, mhartid, mconfigptr

**Status and Control:**
- mstatus/sstatus, misa, mstatush

**Trap Handling:**
- mtvec/stvec, mepc/sepc, mcause/scause, mtval/stval, mscratch/sscratch

**Interrupt Management:**
- mie/sie, mip/sip, mideleg, medeleg

**Memory Protection:**
- pmpcfg0-3, pmpaddr0-15 (Physical Memory Protection)
- satp (Virtual Memory Translation)

**Performance Monitoring:**
- mcycle, minstret, mhpmcounter3-31, mhpmevent3-31
- cycle, time, instret (user read-only views)

**Debugging:**
- dcsr, dpc, dscratch0-1, tselect, tdata1-3

**Hypervisor (H-extension):**
- hstatus, hedeleg, hideleg, hgatp, vsstatus, vsatp

**Vector (V-extension):**
- vstart, vl, vtype, vlenb, vcsr

---

## References

- **RISC-V Privileged Architecture Specification v20211203**
  - Complete CSR definitions and behavior
  - Exception and interrupt handling
  - Virtual memory specifications
  
- **RISC-V Unprivileged ISA Specification v20191213**
  - User-level registers and instructions
  - Counter/timer access
  
- **RISC-V ELF psABI (Platform-Specific ABI)**
  - Register calling conventions
  - Stack frame layouts
  
- **Linux Kernel Source Code**
  - `linux/arch/riscv/kernel/entry.S` - Exception handling implementation
  - `linux/arch/riscv/kernel/head.S` - Kernel boot code
  - `linux/include/uapi/asm-generic/unistd.h` - System call numbers
  
- **U-Boot Bootloader Source**
  - `u-boot/arch/riscv/cpu/start.S` - Boot initialization
  
- **RISC-V Assembly Programmer's Manual**
  - https://github.com/riscv-non-isa/riscv-asm-manual

---

*Last Updated: November 2025*
