    # RISC-V Assembly Language Learning Path: Scratch to Expert
    ## Goal: Understand and Debug U-Boot & Linux Kernel RISC-V Assembly

    ---

    ## Phase 1: Foundation (Week 1-2)

    ### 1.1 RISC-V Architecture Basics

    **Key Concepts:**
    - RISC-V is a modular ISA with base integer instruction set + optional extensions
    - RV32I (32-bit) and RV64I (64-bit) base integer instructions
    - Extensions: M (multiply/divide), A (atomic), F/D (floating point), C (compressed)

    **RISC-V Registers:**
    ```
    Register ABI Name  Description                      Saved by
    x0       zero      Hard-wired zero                  -
    x1       ra        Return address                   Caller
    x2       sp        Stack pointer                    Callee
    x3       gp        Global pointer                   -
    x4       tp        Thread pointer                   -
    x5-x7    t0-t2     Temporary registers              Caller
    x8       s0/fp     Saved register/Frame pointer     Callee
    x9       s1        Saved register                   Callee
    x10-x11  a0-a1     Function args/return values      Caller
    x12-x17  a2-a7     Function arguments               Caller
    x18-x27  s2-s11    Saved registers                  Callee
    x28-x31  t3-t6     Temporary registers              Caller
    ```

    **Simple Example - Hello World Concepts:**
    ```asm
    .section .text
    .global _start

    _start:
        # Load immediate value into register
        li a0, 42              # Load 42 into a0 (x10 - first argument register)
        li a1, 100             # Load 100 into a1 (x11 - second argument register)
        
        # Add two numbers
        add a2, a0, a1         # a2 = a0 + a1 = 142 (a2/x12 - third argument register)
        
        # Exit syscall (Linux)
        li a7, 93              # syscall number for exit (a7/x17 - syscall number register)
        li a0, 0               # exit code 0 (a0/x10 - return value/first arg)
        ecall                  # make syscall (environment call instruction)
    ```

    **Register Usage Breakdown:**
    ```
    a0 (x10): First function argument / syscall argument / return value
            - Used to pass value 42, then overwritten with exit code 0
            
    a1 (x11): Second function argument / syscall argument
            - Used to pass value 100
            
    a2 (x12): Third function argument / syscall argument
            - Used to store the result of addition (142)
            
    a7 (x17): Syscall number register
            - Tells kernel which system call to execute (93 = exit)
            
    Note: In Linux syscalls, a0-a5 hold arguments, a7 holds syscall number
    ```

    **Detailed Execution Flow:**
    ```
    1. li a0, 42        -> a0 = 42 (0x2A)
    2. li a1, 100       -> a1 = 100 (0x64)
    3. add a2, a0, a1   -> a2 = 42 + 100 = 142 (0x8E)
    4. li a7, 93        -> a7 = 93 (sys_exit syscall number)
    5. li a0, 0         -> a0 = 0 (exit code: success)
                        Note: a0 is reused here!
    6. ecall            -> Trap to kernel with:
                        - a7 = 93 (which syscall)
                        - a0 = 0 (exit code parameter)
                        Kernel calls sys_exit(0)
    ```

    **Understanding Syscall Number 93:**

    The number **93** is the system call number for `exit` in the RISC-V Linux ABI (Application Binary Interface).

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

    **How the Kernel Processes Syscalls:**
    ```
    User Space (your program):
        li a7, 93           # Set syscall number
        li a0, 0            # Set argument(s)
        ecall               # Trap to kernel
        
    Kernel Space (Linux kernel):
        1. CPU switches to supervisor/machine mode
        2. Saves user context (PC, registers, etc.)
        3. Reads a7 register -> sees 93
        4. Looks up sys_call_table[93] -> finds sys_exit function
        5. Calls sys_exit(a0) with a0=0 as argument
        6. sys_exit terminates the process
    ```

    **What Happens During `ecall` Execution - Deep Dive:**

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
        # Stack now contains struct pt_regs with snapshot of user state
    ```

    **3. Syscall Dispatch - Finding the Handler:**
    ```asm
        # Check if this is a syscall (scause should indicate syscall)
        csrr t0, scause
        li   t1, 8                 # 8 = syscall from U-mode
        bne  t0, t1, not_syscall   # If not syscall, handle differently
        
        # This IS a syscall!
        # a7 still contains 93 (the syscall number)
        # Let's find the function to call
        
        # Bounds check: is syscall number valid?
        li   t0, __NR_syscalls     # Total number of syscalls
        bgeu a7, t0, invalid_syscall
        
        # Load function pointer from sys_call_table
        la   t0, sys_call_table    # Base address of syscall table
        slli t1, a7, 3             # t1 = a7 * 8 (each pointer is 8 bytes)
        add  t0, t0, t1            # t0 = &sys_call_table[93]
        ld   t0, 0(t0)             # t0 = sys_call_table[93] (function pointer)
        
        # sys_call_table[93] points to sys_exit function
        # Now t0 contains address of sys_exit()
        
        # Arguments are already in a0-a5 registers!
        # For exit: a0 = 0 (exit code)
        
        # Call the syscall handler
        jalr t0                    # Jump to sys_exit(a0=0)
    ```

    **4. Inside `sys_exit()` - C Function in Kernel:**
    ```c
    // This is now C code in linux/kernel/exit.c

    SYSCALL_DEFINE1(exit, int, error_code)  // SYSCALL_DEFINE1 creates sys_exit
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
    - Its task_struct remains until parent calls wait()

    Important: The user program that called ecall NEVER returns!
    The "return" from the syscall handler goes to a DIFFERENT process.
    This is why exit() is special - it's the only syscall that doesn't return.
    ```

    **Complete Flow Diagram:**
    ```
    User Program                  Hardware                 Kernel
    -----------                   --------                 ------
    li a7, 93
    li a0, 0
    ecall         ─────────────>  [CPU traps]  ──────────> handle_exception:
                                - Save PC                  csrrw sp, sscratch, sp
                                - Set mcause=8             addi sp, sp, -256
                                - Switch to S-mode         sd x1, 0(sp)
                                - PC = mtvec               sd x10, 80(sp)  # a0
                                - Disable interrupts       sd x17, 136(sp) # a7
                                                            ...
                                                        
                                                        syscall_handler:
                                                            la t0, sys_call_table
                                                            slli t1, a7, 3
                                                            add t0, t0, t1
                                                            ld t0, 0(t0)
                                                            jalr t0  ────────> sys_exit(a0=0):
                                                                                do_exit(0);
                                                                                [cleanup]
                                                                                schedule();
                                                        
                                                        [Scheduler picks new process]
                                                        [Restores new process context]
                                                        sret ───────────> [Different process runs]

    [Original process GONE]
    ```

    **Key Takeaways:**
    1. `ecall` is just ONE instruction that triggers EVERYTHING
    2. Hardware automatically saves minimal state (PC, cause, status)
    3. Kernel software saves ALL registers for complete context
    4. Syscall number in a7 is used as index into function table
    5. Arguments in a0-a5 are passed directly to C function
    6. For exit(), the process never returns - it's gone forever
    7. The "return" actually switches to a completely different process

    **Example: Different Syscalls**
    ```asm
    # System call: write(1, "Hello\n", 6)
    li   a7, 64         # __NR_write = 64
    li   a0, 1          # fd = 1 (stdout)
    la   a1, msg        # buf = address of "Hello\n"
    li   a2, 6          # count = 6 bytes
    ecall               # Kernel executes sys_write(1, msg, 6)

    # System call: exit(0)
    li   a7, 93         # __NR_exit = 93
    li   a0, 0          # status = 0
    ecall               # Kernel executes sys_exit(0)
    ```

    **Reference:** The syscall numbers are defined in the Linux kernel source:
    - `linux/include/uapi/asm-generic/unistd.h` (generic definitions)
    - `linux/arch/riscv/include/uapi/asm/unistd.h` (RISC-V specific)
    - RISC-V follows the generic syscall ABI where possible

    ### 1.2 Basic Instruction Types

    **R-Type (Register-Register Operations):**
    ```asm
    add  rd, rs1, rs2      # rd = rs1 + rs2
    sub  rd, rs1, rs2      # rd = rs1 - rs2
    and  rd, rs1, rs2      # rd = rs1 & rs2
    or   rd, rs1, rs2      # rd = rs1 | rs2
    xor  rd, rs1, rs2      # rd = rs1 ^ rs2
    sll  rd, rs1, rs2      # rd = rs1 << rs2 (shift left logical)
    srl  rd, rs1, rs2      # rd = rs1 >> rs2 (shift right logical)
    sra  rd, rs1, rs2      # rd = rs1 >> rs2 (shift right arithmetic)
    slt  rd, rs1, rs2      # rd = (rs1 < rs2) ? 1 : 0 (signed)
    sltu rd, rs1, rs2      # rd = (rs1 < rs2) ? 1 : 0 (unsigned)
    ```

    **I-Type (Immediate Operations):**
    ```asm
    addi  rd, rs1, imm     # rd = rs1 + sign_extend(imm)
    andi  rd, rs1, imm     # rd = rs1 & imm
    ori   rd, rs1, imm     # rd = rs1 | imm
    xori  rd, rs1, imm     # rd = rs1 ^ imm
    slli  rd, rs1, imm     # rd = rs1 << imm
    srli  rd, rs1, imm     # rd = rs1 >> imm (logical)
    srai  rd, rs1, imm     # rd = rs1 >> imm (arithmetic)
    slti  rd, rs1, imm     # rd = (rs1 < imm) ? 1 : 0
    ```

    **Load/Store Instructions:**
    ```asm
    # Load instructions
    lb   rd, offset(rs1)   # Load byte (sign-extended)
    lh   rd, offset(rs1)   # Load halfword (sign-extended)
    lw   rd, offset(rs1)   # Load word (sign-extended in RV64)
    ld   rd, offset(rs1)   # Load doubleword (RV64 only)
    lbu  rd, offset(rs1)   # Load byte unsigned
    lhu  rd, offset(rs1)   # Load halfword unsigned
    lwu  rd, offset(rs1)   # Load word unsigned (RV64)

    # Store instructions
    sb   rs2, offset(rs1)  # Store byte
    sh   rs2, offset(rs1)  # Store halfword
    sw   rs2, offset(rs1)  # Store word
    sd   rs2, offset(rs1)  # Store doubleword (RV64)
    ```

    **Example - Array Access:**
    ```asm
    .data
    array: .word 10, 20, 30, 40, 50

    .text
    _start:
        la   a0, array         # Load address of array into a0 (x10)
        li   a1, 2             # Index 2 into a1 (x11)
        slli a1, a1, 2         # a1 = a1 << 2 = 2 * 4 = 8 (offset in bytes)
        add  a0, a0, a1        # a0 = base + offset (address of array[2])
        lw   a2, 0(a0)         # Load word from memory[a0] into a2 (x12) -> a2 = 30
    ```

    **Register Usage:**
    ```
    a0 (x10): Base address of array, then address of array[2]
    a1 (x11): Array index (2), then byte offset (8)
    a2 (x12): Result - stores array[2] value (30)
    ```

    ### 1.3 Control Flow

    **Branch Instructions:**
    ```asm
    beq  rs1, rs2, label   # Branch if rs1 == rs2
    bne  rs1, rs2, label   # Branch if rs1 != rs2
    blt  rs1, rs2, label   # Branch if rs1 < rs2 (signed)
    bge  rs1, rs2, label   # Branch if rs1 >= rs2 (signed)
    bltu rs1, rs2, label   # Branch if rs1 < rs2 (unsigned)
    bgeu rs1, rs2, label   # Branch if rs1 >= rs2 (unsigned)
    ```

    **Jump Instructions:**
    ```asm
    jal  rd, label         # Jump and link: rd = PC+4, PC = label
    jalr rd, offset(rs1)   # Jump and link register: rd = PC+4, PC = rs1+offset
    ```

    **Example - If-Else:**
    ```asm
        li   a0, 10            # a0 (x10) = 10
        li   a1, 20            # a1 (x11) = 20
        
        blt  a0, a1, less_than # Branch if 10 < 20 (signed comparison)
        # a0 >= a1 path (not taken)
        li   a2, 0             # a2 (x12) = 0
        j    done              # Jump to done
        
    less_than:
        # a0 < a1 path (taken since 10 < 20)
        li   a2, 1             # a2 (x12) = 1
        
    done:
        # continue... a2 now contains 1
    ```

    **Register Usage:**
    ```
    a0 (x10): First comparison value (10)
    a1 (x11): Second comparison value (20)
    a2 (x12): Result - 1 if a0 < a1, else 0
    ```

    **Example - Loop:**
    ```asm
        li   a0, 0             # a0 (x10) = counter = 0
        li   a1, 10            # a1 (x11) = max = 10
        
    loop:
        addi a0, a0, 1         # a0 = a0 + 1 (increment counter)
        blt  a0, a1, loop      # if counter < 10, jump to loop
        
        # continue after loop (a0 = 10)
    ```

    **Register Usage & Execution:**
    ```
    a0 (x10): Loop counter (0 -> 1 -> 2 -> ... -> 10)
    a1 (x11): Loop limit (10)

    Iteration trace:
    Iteration 1: a0=1, blt 1<10 -> branch taken
    Iteration 2: a0=2, blt 2<10 -> branch taken
    ...
    Iteration 9: a0=9, blt 9<10 -> branch taken
    Iteration 10: a0=10, blt 10<10 -> branch NOT taken, exit loop
    ```

    ---

    ## Phase 2: Intermediate (Week 3-4)

    ### 2.1 Function Calls and Stack

    **Calling Convention:**
    - Arguments: a0-a7 (x10-x17)
    - Return values: a0-a1
    - Saved registers: s0-s11 must be preserved by callee
    - Temporary registers: t0-t6 may be clobbered

    **Stack Frame Example:**
    ```asm
    # Function prologue
    function:
        addi sp, sp, -16       # sp (x2) = sp - 16 (allocate 16 bytes on stack)
        sd   ra, 8(sp)         # Store ra (x1) at sp+8 (save return address)
        sd   s0, 0(sp)         # Store s0 (x8) at sp+0 (save frame pointer)
        addi s0, sp, 16        # s0 (x8/fp) = sp + 16 (set new frame pointer)
        
        # Function body
        # ... your code here ...
        
        # Function epilogue
        ld   ra, 8(sp)         # ra (x1) = memory[sp+8] (restore return address)
        ld   s0, 0(sp)         # s0 (x8) = memory[sp+0] (restore frame pointer)
        addi sp, sp, 16        # sp (x2) = sp + 16 (deallocate stack frame)
        ret                    # jalr zero, ra, 0 (return to caller)
    ```

    **Register Usage:**
    ```
    sp (x2):  Stack pointer - points to current stack top
    ra (x1):  Return address - where to return after function completes
    s0 (x8):  Saved register/Frame pointer - preserved across calls

    Stack Layout (grows downward):
    [sp+16] <- old sp (before allocation)
    [sp+8]  <- saved ra
    [sp+0]  <- saved s0  <- current sp
    ```

    **Example - Recursive Fibonacci:**
    ```asm
    # int fibonacci(int n)
    fibonacci:
        addi sp, sp, -16       # Allocate stack
        sd   ra, 8(sp)         # Save return address
        sd   s0, 0(sp)         # Save s0
        
        li   t0, 2
        blt  a0, t0, base_case # if n < 2, return n
        
        # Save n
        mv   s0, a0            # s0 = n
        
        # Call fib(n-1)
        addi a0, s0, -1        # a0 = n - 1
        call fibonacci
        mv   t1, a0            # t1 = fib(n-1)
        
        # Call fib(n-2)
        addi a0, s0, -2        # a0 = n - 2
        call fibonacci
        add  a0, a0, t1        # a0 = fib(n-2) + fib(n-1)
        j    epilogue
        
    base_case:
        # Return n as-is
        
    epilogue:
        ld   ra, 8(sp)         # Restore return address
        ld   s0, 0(sp)         # Restore s0
        addi sp, sp, 16        # Deallocate stack
        ret
    ```

    ### 2.2 Pseudo-Instructions

    RISC-V assembler provides many pseudo-instructions that expand to real instructions:

    ```asm
    # Pseudo-instruction          -> Actual instruction(s)
    nop                           -> addi zero, zero, 0
    li   rd, immediate            -> Multiple instructions for large immediates
    la   rd, symbol               -> auipc + addi
    mv   rd, rs                   -> addi rd, rs, 0
    not  rd, rs                   -> xori rd, rs, -1
    neg  rd, rs                   -> sub  rd, zero, rs
    seqz rd, rs                   -> sltiu rd, rs, 1
    snez rd, rs                   -> sltu rd, zero, rs
    j    offset                   -> jal zero, offset
    jr   rs                       -> jalr zero, rs, 0
    ret                           -> jalr zero, ra, 0
    call offset                   -> auipc + jalr
    ```

    ### 2.3 CSR (Control and Status Registers)

    **Common CSRs:**
    ```asm
    # Machine mode CSRs
    mstatus    # Machine status register
    mtvec      # Machine trap-handler base address
    mepc       # Machine exception program counter
    mcause     # Machine trap cause
    mie        # Machine interrupt enable
    mip        # Machine interrupt pending
    mhartid    # Hardware thread ID
    misa       # ISA and extensions

    # Supervisor mode CSRs
    sstatus    # Supervisor status
    stvec      # Supervisor trap vector
    sepc       # Supervisor exception PC
    scause     # Supervisor trap cause
    satp       # Supervisor address translation and protection
    ```

    **CSR Instructions:**
    ```asm
    csrrw  rd, csr, rs1    # Atomic Read/Write: rd = csr; csr = rs1
    csrrs  rd, csr, rs1    # Atomic Read/Set: rd = csr; csr = csr | rs1
    csrrc  rd, csr, rs1    # Atomic Read/Clear: rd = csr; csr = csr & ~rs1

    # Immediate versions
    csrrwi rd, csr, imm
    csrrsi rd, csr, imm
    csrrci rd, csr, imm
    ```

    **Example - Reading Hart ID:**
    ```asm
        csrr a0, mhartid       # Read hardware thread ID into a0
    ```

    ---

    ## RISC-V Control and Status Registers (CSRs) - Complete Reference

    ### Overview
    CSRs are special registers that control CPU behavior, handle exceptions/interrupts, and manage privilege modes. They are accessed using special CSR instructions and are critical for bootloaders, kernels, and exception handling.

    ---

    ### Machine Mode CSRs (Highest Privilege Level)

    #### 1. **mstatus** - Machine Status Register

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
    13           FS[1:0]      Floating-point unit Status
    15           XS[1:0]      Extension Status
    17           MPRV         Modify PRiVilege (memory access uses MPP privilege)
    18-19        XS           User extension status
    63           SD           State Dirty (FS or XS is dirty)
    ```

    **Detailed Field Explanations:**

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
    1. **Bootloader (U-Boot, OpenSBI):**
    - Initialize privilege modes
    - Transition from M-mode to S-mode
    - Enable/disable interrupts during hardware setup

    2. **Kernel initialization:**
    - Setup initial privilege state
    - Configure exception handling
    - Enable floating-point unit

    3. **Exception handlers:**
    - Save/restore interrupt state
    - Track privilege transitions

    ---

    #### 2. **mepc** - Machine Exception Program Counter

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

    #### 3. **mcause** - Machine Cause Register

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

    # Example 2: C handler receiving cause
    trap_handler:
        csrr a0, mcause         # Pass cause as argument
        csrr a1, mepc           # Pass fault PC
        csrr a2, mtval          # Pass additional info
        call do_trap_c          # Call C function

    # Example 3: Checking for specific syscall
    handle_syscall:
        csrr t0, mcause
        li   t1, 8
        bne  t0, t1, not_syscall    # Verify it's U-mode ecall
        # ... process syscall ...
    ```

    **Where mcause is used:**
    1. **Exception dispatchers** - Route to correct handler
    2. **System call handlers** - Verify it's actually a syscall
    3. **Debuggers** - Display exception type to user
    4. **Error logging** - Record what caused the fault
    5. **Kernel panic** - Print cause before halting

    ---

    #### 4. **mtvec** - Machine Trap Vector

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

    **Usage Examples:**

    ```asm
    # Example 1: Simple setup with direct mode
    setup_trap_handler:
        la   t0, trap_entry
        csrw mtvec, t0          # Direct mode (all traps go here)

    trap_entry:
        # Save context
        # Check mcause to dispatch
        # Handle trap
        # Restore context
        mret

    # Example 2: Vectored mode for fast interrupts
    setup_vectored:
        la   t0, trap_vectors
        ori  t0, t0, 1          # Set vectored mode
        csrw mtvec, t0

    # Example 3: Bootloader changing trap handler
        # Start with simple handler
        la   t0, early_trap
        csrw mtvec, t0
        
        # ... initialize system ...
        
        # Switch to full kernel handler
        la   t0, kernel_trap_handler
        csrw mtvec, t0

    # Example 4: Reading current handler
        csrr t0, mtvec          # Read handler address
        andi t1, t0, 3          # Extract MODE bits
        andi t0, t0, ~3         # Extract BASE address
    ```

    **Where mtvec is used:**
    1. **Bootloader initialization** - First setup of exception handling
    2. **Kernel boot** - Switch from bootloader handler to kernel handler
    3. **Context switching** - Some OSes change per-process
    4. **Debugging** - Temporary handler for testing
    5. **Interrupt controller setup** - Configure vectored mode

    ---

    #### 5. **mie** - Machine Interrupt Enable Register

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
    # Example 1: Enable timer interrupts
        li   t0, (1 << 7)       # Bit 7 = MTIE
        csrs mie, t0            # Set MTIE (enable timer)
        csrsi mstatus, 0x8      # Set MIE (global enable)

    # Example 2: Disable only external interrupts (keep others)
        li   t0, (1 << 11)      # Bit 11 = MEIE
        csrc mie, t0            # Clear MEIE (disable external)

    # Example 3: Enable all machine-mode interrupts
        li   t0, 0x888          # Bits 3, 7, 11 (MSI, MTI, MEI)
        csrs mie, t0            # Enable all three
        csrsi mstatus, 0x8      # Global enable

    # Example 4: Critical section (disable all interrupts temporarily)
        csrci mstatus, 0x8      # Clear MIE (fast disable)
        # ... critical code ...
        csrsi mstatus, 0x8      # Set MIE (re-enable)

    # Example 5: Reading current enables
        csrr t0, mie            # Read mie
        andi t1, t0, 0x80       # Check if timer enabled (bit 7)
        bnez t1, timer_is_enabled
    ```

    **Where mie is used:**
    1. **Timer initialization** - Enable timer for scheduling
    2. **Interrupt controller setup** - Configure which sources are active
    3. **Power management** - Disable unused interrupt sources
    4. **Critical sections** - Fine-grained interrupt control
    5. **Device drivers** - Enable/disable device interrupts

    ---

    #### 6. **mip** - Machine Interrupt Pending Register

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
    # Example 1: Check if timer interrupt is pending
        csrr t0, mip            # Read pending interrupts
        andi t0, t0, 0x80       # Check bit 7 (MTIP)
        bnez t0, timer_pending

    # Example 2: Poll for any interrupt
        csrr t0, mip
        andi t0, t0, 0x888      # Check all three sources
        beqz t0, no_interrupts_pending

    # Example 3: Wait for specific interrupt (polling)
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

    ### Supervisor Mode CSRs (Used by Operating System)

    #### 7. **sstatus** - Supervisor Status Register

    **Purpose:** Supervisor-level view of mstatus (subset of fields).

    **Key fields:**
    ```
    Bit  Name   Description
    ---  ----   -----------
    1    SIE    Supervisor Interrupt Enable
    5    SPIE   Supervisor Previous Interrupt Enable
    8    SPP    Supervisor Previous Privilege (0=U, 1=S)
    ```

    Similar to mstatus.MIE/MPIE/MPP but for supervisor mode.

    ---

    #### 8. **sepc** - Supervisor Exception Program Counter

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

    #### 9. **scause** - Supervisor Cause Register

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

    #### 10. **stvec** - Supervisor Trap Vector

    **Purpose:** Like mtvec, but for supervisor-level trap handler.

    ```asm
    # Kernel initialization
        la   t0, kernel_trap_entry
        csrw stvec, t0          # Set kernel trap handler
    ```

    ---

    #### 11. **satp** - Supervisor Address Translation and Protection

    **Purpose:** Controls virtual memory (page table base and mode).

    **Format (RV64):**
    ```
    Bits 63-60: MODE (0=Bare, 8=Sv39, 9=Sv48, 10=Sv57)
    Bits 59-44: ASID (Address Space ID)
    Bits 43-0:  PPN (Physical Page Number of root page table)
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

    ### CSR Usage in Complete Boot Flow

    ```asm
    # Machine mode initialization (Bootloader/OpenSBI)
    _start:
        # 1. Disable interrupts
        csrw mie, zero          # Clear all interrupt enables
        csrci mstatus, 0x8      # Clear MIE (global disable)
        
        # 2. Setup trap handler
        la   t0, m_trap_handler
        csrw mtvec, t0          # Set trap vector
        
        # 3. Configure delegation (pass some traps to supervisor)
        li   t0, 0xffff
        csrw medeleg, t0        # Delegate exceptions to S-mode
        csrw mideleg, t0        # Delegate interrupts to S-mode
        
        # 4. Setup timer
        li   t0, (1 << 7)       # MTIE
        csrw mie, t0            # Enable timer interrupt
        
        # 5. Prepare to enter supervisor mode
        li   t0, (1 << 11) | (1 << 5)  # MPP=01 (S-mode), MPIE=1
        csrw mstatus, t0
        
        la   t0, supervisor_start
        csrw mepc, t0           # Set entry point
        
        mret                    # Enter S-mode, PC=mepc

    # Supervisor mode (Kernel)
    supervisor_start:
        # Now in S-mode
        # Setup kernel trap handler
        la   t0, kernel_trap
        csrw stvec, t0
        
        # Enable supervisor interrupts
        csrsi sstatus, 0x2      # SIE=1
        
        # Setup paging
        la   t0, kernel_page_table
        srli t0, t0, 12
        li   t1, (8 << 60)      # Sv39 mode
        or   t0, t0, t1
        csrw satp, t0
        sfence.vma
        
        # Jump to C kernel
        tail start_kernel
    ```

    ---

    ### CSR Summary Table

    | CSR      | Mode | Purpose | When Used |
    |----------|------|---------|-----------|
    | mstatus  | M    | Machine status/control | Boot, exception entry/exit, privilege changes |
    | mepc     | M    | Exception return address | Every trap handler |
    | mcause   | M    | Trap reason | Exception dispatch |
    | mtvec    | M    | Trap handler address | Boot, handler setup |
    | mie      | M    | Interrupt enables | Interrupt initialization |
    | mip      | M    | Interrupt pending | Interrupt checking |
    | mhartid  | M    | CPU core ID | Multi-core sync |
    | sstatus  | S    | Supervisor status | OS kernel operations |
    | sepc     | S    | Supervisor exception PC | Syscalls, kernel traps |
    | scause   | S    | Supervisor trap cause | Kernel exception dispatch |
    | stvec    | S    | Supervisor trap vector | Kernel initialization |
    | satp     | S    | Page table control | Virtual memory setup |

    **Reference:** RISC-V Privileged Architecture Specification v20211203

    ---

    ## Complete RISC-V CSR Register Reference

    ### CSR Address Map and Abbreviations

    RISC-V CSRs have 12-bit addresses. The address encoding indicates privilege level and read/write permissions:

    **Address Format:**
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

    ---

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

    **sstatus Bit Fields:**
    ```
    Bit  Name   Description
    ---  ----   -----------
    0    UIE    User Interrupt Enable
    1    SIE    Supervisor Interrupt Enable
    4    UPIE   User Previous Interrupt Enable
    5    SPIE   Supervisor Previous Interrupt Enable
    8    SPP    Supervisor Previous Privilege (0=U, 1=S)
    13-14 FS    Floating-point Status (00=Off, 01=Initial, 10=Clean, 11=Dirty)
    15-16 XS    Extension Status
    18   SUM    Permit Supervisor User Memory access
    19   MXR    Make eXecutable Readable
    32   UXL    User mode XLEN (1=32-bit, 2=64-bit) [RV64 only]
    63   SD     State Dirty (FS or XS is dirty)
    ```

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

    **scause Exception/Interrupt Codes:** (Same as mcause but supervisor-level)

    **Supervisor Protection and Translation:**

    | Address | Name      | Abbreviation | Description | R/W |
    |---------|-----------|--------------|-------------|-----|
    | 0x180   | satp      | SATP         | Supervisor address translation and protection | RW |

    **satp Format (RV64):**
    ```
    Bits 63-60: MODE
    0 = Bare (no translation)
    8 = Sv39 (39-bit virtual addressing)
    9 = Sv48 (48-bit virtual addressing)
    10 = Sv57 (57-bit virtual addressing)
    Bits 59-44: ASID (Address Space IDentifier)
    Bits 43-0:  PPN (Physical Page Number of root page table)
    ```

    **Example Usage:**
    ```asm
    # Setup supervisor trap handler
        la   t0, kernel_trap
        csrw stvec, t0          # Set trap vector
        
    # Enable supervisor interrupts
        li   t0, (1 << 1) | (1 << 5) | (1 << 9)  # SSIE, STIE, SEIE
        csrw sie, t0            # Enable S-mode interrupts
        csrsi sstatus, 0x2      # Set SIE bit in sstatus
        
    # Setup virtual memory (Sv39)
        la   t0, page_table_root
        srli t0, t0, 12         # Convert to PPN
        li   t1, (8 << 60)      # MODE = Sv39
        or   t0, t0, t1
        csrw satp, t0           # Enable paging
        sfence.vma              # Flush TLB
    ```

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

    **Example:**
    ```asm
    # Get CPU core ID (for multi-core systems)
        csrr a0, mhartid        # a0 = 0, 1, 2, ... (core number)
        
    # Check vendor ID
        csrr a0, mvendorid      # Implementation-specific value
    ```

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
    Bit 4 (E): RV32E base ISA (embedded)
    Bit 5 (F): Single-precision floating-point
    Bit 8 (I): RV32I/64I/128I base ISA
    Bit 12 (M): Integer multiply/divide extension
    Bit 13 (N): User-level interrupts
    Bit 16 (Q): Quad-precision floating-point
    Bit 18 (S): Supervisor mode implemented
    Bit 20 (U): User mode implemented
    Bit 21 (V): Vector extension
    Bits 63-62 (RV64): MXL (Machine XLEN)
    1 = 32-bit, 2 = 64-bit, 3 = 128-bit
    ```

    **medeleg/mideleg:**
    These registers delegate exceptions/interrupts to lower privilege levels:
    ```
    Bit = 1: Delegate to supervisor mode
    Bit = 0: Handle in machine mode

    Common delegation pattern (delegate most to supervisor):
        li   t0, 0xFFFF
        csrw medeleg, t0        # Delegate all exceptions
        csrw mideleg, t0        # Delegate all interrupts
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

    **mtval (Machine Trap Value):**
    ```
    Provides additional information about the trap:
    - Instruction address misaligned: Faulting address
    - Instruction access fault: Faulting address
    - Illegal instruction: The instruction bits
    - Breakpoint: Program counter
    - Load/store misaligned: Faulting address
    - Load/store access fault: Faulting address
    - Page fault: Faulting virtual address
    ```

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

    **PMP (Physical Memory Protection):**
    ```
    Allows M-mode to restrict memory access for S-mode and U-mode.

    pmpcfg format (each entry is 8 bits):
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
    | 0xB83   | mhpmcounter3h | MHPMCTR3H  | Upper 32 bits of mhpmcounter3 (RV32) | RW |
    | ...     | ...         | ...          | ... | ... |
    | 0xB9F   | mhpmcounter31h | MHPMCTR31H | Upper 32 bits of mhpmcounter31 (RV32) | RW |

    **Machine Counter Setup:**

    | Address | Name        | Abbreviation | Description | R/W |
    |---------|-------------|--------------|-------------|-----|
    | 0x320   | mcountinhibit | MCNTINHBT  | Machine counter-inhibit register | RW |
    | 0x323   | mhpmevent3  | MHPMEVT3     | Machine performance-monitoring event 3 | RW |
    | ...     | ...         | ...          | ... | ... |
    | 0x33F   | mhpmevent31 | MHPMEVT31    | Machine performance-monitoring event 31 | RW |

    **mcountinhibit:**
    ```
    Controls which counters are active:
    Bit 0: Inhibit mcycle
    Bit 2: Inhibit minstret
    Bit 3-31: Inhibit mhpmcounter3-31

    Example - Stop cycle counting during debugging:
        csrsi mcountinhibit, 0x1    # Set bit 0, stop mcycle
        # ... debug code ...
        csrci mcountinhibit, 0x1    # Clear bit 0, resume mcycle
    ```

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

    RISC-V reserves address ranges for custom extensions:

    | Address Range | Description |
    |---------------|-------------|
    | 0x800-0x8FF   | Custom read-write (URW) |
    | 0xCC0-0xCFF   | Custom read-only (URO) |
    | 0x9C0-0x9FF   | Custom read-write supervisor (SRW) |
    | 0xDC0-0xDFF   | Custom read-only supervisor (SRO) |
    | 0xBC0-0xBFF   | Custom read-write machine (MRW) |
    | 0xFC0-0xFFF   | Custom read-only machine (MRO) |

    ---

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

    ### CSR Access Instructions Summary

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

    **Common Patterns:**
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

    **Reference:** 
    - RISC-V Privileged Architecture Specification v20211203
    - RISC-V User-Level ISA Specification v20191213
    - RISC-V Vector Extension Specification v1.0
    - RISC-V Hypervisor Extension Specification v1.0

    ---

    ## Phase 3: U-Boot Assembly Analysis (Week 5-6)

    ### 3.1 U-Boot Entry Point

    **Location:** `u-boot/arch/riscv/cpu/start.S`

    **Key Sections to Study:**

    ```asm
    # Simplified U-Boot entry point structure
    .section .text
    .globl _start
    _start:
        # Disable interrupts
        csrw mie, zero
        csrw mip, zero
        
        # Get CPU ID
        csrr a0, mhartid
        
        # Only hart 0 continues, others wait
        bnez a0, secondary_hart_loop
        
    primary_hart:
        # Setup stack pointer
        la   sp, __stack_top
        
        # Clear BSS section
        la   t0, __bss_start
        la   t1, __bss_end
    clear_bss:
        sd   zero, 0(t0)
        addi t0, t0, 8
        bltu t0, t1, clear_bss
        
        # Jump to C code
        j    board_init_f
        
    secondary_hart_loop:
        # Wait for interrupt
        wfi
        j    secondary_hart_loop
    ```

    **Real U-Boot Example from your u-boot directory:**
    ```bash
    # View actual U-Boot RISC-V startup code
    cat u-boot/arch/riscv/cpu/start.S
    ```

    ### 3.2 Important U-Boot Assembly Patterns

    **1. Relocation Code:**
    ```asm
    relocate_code:
        # Save arguments
        mv   s0, a0            # s0 = gd (global data pointer)
        mv   s1, a1            # s1 = dest_addr
        
        # Calculate relocation offset
        la   t0, _start        # t0 = link address
        sub  s2, s1, t0        # s2 = relocation offset
        
        # Copy code
        la   t0, __image_copy_start
        la   t1, __image_copy_end
    copy_loop:
        ld   t2, 0(t0)
        sd   t2, 0(s1)
        addi t0, t0, 8
        addi s1, s1, 8
        bltu t0, t1, copy_loop
    ```

    **2. Exception/Interrupt Handler Setup:**
    ```asm
        # Set trap vector
        la   t0, trap_entry
        csrw mtvec, t0
        
    trap_entry:
        # Save all registers
        addi sp, sp, -256
        sd   x1, 0(sp)
        sd   x2, 8(sp)
        # ... save all registers ...
        
        # Call C handler
        csrr a0, mcause        # First arg: trap cause
        csrr a1, mepc          # Second arg: exception PC
        mv   a2, sp            # Third arg: register context
        call handle_trap
        
        # Restore all registers
        ld   x1, 0(sp)
        # ... restore all ...
        addi sp, sp, 256
        mret                   # Return from machine mode
    ```

    ### 3.3 U-Boot Debugging Techniques

    **GDB Commands for RISC-V:**
    ```bash
    # Start GDB with U-Boot ELF
    riscv64-unknown-elf-gdb u-boot

    # Common GDB commands
    (gdb) target remote :1234           # Connect to QEMU
    (gdb) break _start                  # Break at start
    (gdb) break *0x80000000             # Break at address
    (gdb) info registers                # Show all registers
    (gdb) info register pc sp ra        # Show specific registers
    (gdb) x/10i $pc                     # Disassemble 10 instructions at PC
    (gdb) x/10gx $sp                    # Show 10 doublewords at stack pointer
    (gdb) stepi                         # Step one instruction
    (gdb) nexti                         # Next instruction (skip calls)
    (gdb) backtrace                     # Show call stack
    ```

    **QEMU with U-Boot:**
    ```bash
    # Run U-Boot in QEMU with debugging
    qemu-system-riscv64 \
        -machine virt \
        -bios u-boot.bin \
        -nographic \
        -s -S                           # -s: GDB on port 1234, -S: pause at start

    # In another terminal
    riscv64-unknown-elf-gdb u-boot
    (gdb) target remote :1234
    (gdb) break _start
    (gdb) continue
    ```

    ---

    ## Phase 4: Linux Kernel Assembly Analysis (Week 7-8)

    ### 4.1 Kernel Entry Point

    **Location:** `linux/arch/riscv/kernel/head.S`

    **Key Code Patterns:**

    ```asm
    # Simplified kernel entry
    .section .head.text
    .global _start
    _start:
        # Disable interrupts
        csrw CSR_IE, zero
        
        # Setup per-hart (CPU) stack
        csrr a0, CSR_HARTID
        la   sp, init_thread_union
        li   t0, THREAD_SIZE
        add  sp, sp, t0
        mul  t1, a0, t0
        sub  sp, sp, t1
        
        # Clear BSS
        la   t0, __bss_start
        la   t1, __bss_stop
    clear_bss_loop:
        sd   zero, (t0)
        addi t0, t0, 8
        bltu t0, t1, clear_bss_loop
        
        # Setup trap vector
        la   t0, handle_exception
        csrw CSR_TVEC, t0
        
        # Jump to C code
        tail start_kernel          # Tail call optimization
    ```

    ### 4.2 Context Switch Assembly

    **Location:** `linux/arch/riscv/kernel/entry.S`

    ```asm
    # Simplified context switch
    switch_to:
        # Save callee-saved registers of prev task
        sd   ra, TASK_THREAD_RA(a0)
        sd   sp, TASK_THREAD_SP(a0)
        sd   s0, TASK_THREAD_S0(a0)
        sd   s1, TASK_THREAD_S1(a0)
        # ... save s2-s11 ...
        
        # Restore callee-saved registers of next task
        ld   ra, TASK_THREAD_RA(a1)
        ld   sp, TASK_THREAD_SP(a1)
        ld   s0, TASK_THREAD_S0(a1)
        ld   s1, TASK_THREAD_S1(a1)
        # ... restore s2-s11 ...
        
        # Switch page table if needed
        ld   t0, TASK_THREAD_SATP(a1)
        csrw CSR_SATP, t0
        sfence.vma                 # Flush TLB
        
        ret
    ```

    ### 4.3 System Call Entry

    ```asm
    # Simplified syscall handler
    handle_syscall:
        # Save user context
        csrw CSR_SCRATCH, tp       # Save thread pointer
        csrrw tp, CSR_SCRATCH, tp  # Get kernel tp
        
        sd   x1, PT_RA(tp)
        sd   x2, PT_SP(tp)
        # ... save all caller-saved registers ...
        
        # Get syscall number and call handler
        # a7 contains syscall number
        # a0-a5 contain arguments
        
        la   t0, sys_call_table
        slli t1, a7, 3             # Multiply by 8 (pointer size)
        add  t0, t0, t1
        ld   t0, 0(t0)             # Load function pointer
        jalr t0                    # Call syscall handler
        
        # Restore user context
        ld   x1, PT_RA(tp)
        ld   x2, PT_SP(tp)
        # ... restore all registers ...
        
        sret                       # Return to user mode
    ```

    ### 4.4 Exception/Interrupt Handling

    ```asm
    handle_exception:
        # Save entire register context
        csrrw sp, CSR_SCRATCH, sp  # Swap sp with scratch
        
        addi sp, sp, -PT_SIZE      # Allocate pt_regs on stack
        sd   x1, PT_RA(sp)
        sd   x3, PT_GP(sp)
        # ... save all 32 registers ...
        
        # Save CSR state
        csrr t0, CSR_STATUS
        csrr t1, CSR_EPC
        csrr t2, CSR_TVAL
        csrr t3, CSR_CAUSE
        sd   t0, PT_STATUS(sp)
        sd   t1, PT_EPC(sp)
        sd   t2, PT_BADADDR(sp)
        sd   t3, PT_CAUSE(sp)
        
        # Call C exception handler
        mv   a0, sp                # Pass pt_regs as argument
        call do_trap
        
        # Restore context
        ld   t0, PT_STATUS(sp)
        ld   t1, PT_EPC(sp)
        csrw CSR_STATUS, t0
        csrw CSR_EPC, t1
        
        ld   x1, PT_RA(sp)
        # ... restore all registers ...
        addi sp, sp, PT_SIZE
        
        sret
    ```

    ---

    ## Phase 5: Advanced Debugging (Week 9-10)

    ### 5.1 Debugging Kernel Assembly with QEMU + GDB

    **Setup:**
    ```bash
    # Compile kernel with debug symbols
    cd linux
    make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- defconfig
    make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j$(nproc)

    # Run in QEMU with GDB
    qemu-system-riscv64 \
        -machine virt \
        -kernel arch/riscv/boot/Image \
        -append "console=ttyS0 nokaslr" \
        -nographic \
        -s -S

    # In another terminal
    riscv64-linux-gnu-gdb vmlinux
    (gdb) target remote :1234
    (gdb) break start_kernel
    (gdb) continue
    ```

    **Advanced GDB Techniques:**
    ```bash
    # Break on specific instruction
    (gdb) break *_start
    (gdb) break *handle_exception

    # Watch memory address
    (gdb) watch *(long*)0x80000000

    # Examine assembly with source
    (gdb) set disassemble-next-line on
    (gdb) display/i $pc

    # Show register changes
    (gdb) display/x $pc
    (gdb) display/x $sp
    (gdb) display/x $ra

    # Custom commands for RISC-V CSRs
    (gdb) define show_csrs
    > info register mstatus
    > info register mtvec
    > info register mepc
    > info register mcause
    > end

    # Trace execution
    (gdb) commands
    > silent
    > printf "PC=%p SP=%p RA=%p\n", $pc, $sp, $ra
    > continue
    > end
    ```

    ### 5.2 Using OpenOCD for Real Hardware

    **OpenOCD Configuration:**
    ```bash
    # Connect to VisionFive2 board
    openocd -f interface/ftdi/jh7110.cfg -f target/riscv.cfg

    # In another terminal
    riscv64-linux-gnu-gdb vmlinux
    (gdb) target remote :3333
    (gdb) monitor reset halt
    (gdb) load                     # Upload kernel
    (gdb) break _start
    (gdb) continue
    ```

    ### 5.3 Analyzing Disassembly

    **Useful Commands:**
    ```bash
    # Disassemble specific function
    riscv64-linux-gnu-objdump -d vmlinux | grep -A 50 "handle_exception"

    # Disassemble with source intermixed
    riscv64-linux-gnu-objdump -S vmlinux > kernel.asm

    # Show all sections
    riscv64-linux-gnu-readelf -S vmlinux

    # Show symbols
    riscv64-linux-gnu-nm vmlinux | grep syscall

    # Examine specific address range
    riscv64-linux-gnu-objdump -d --start-address=0x80000000 --stop-address=0x80000100 vmlinux
    ```

    ### 5.4 Tracing with ftrace

    **Enable Kernel Function Tracing:**
    ```bash
    # In running kernel
    mount -t tracefs none /sys/kernel/tracing
    cd /sys/kernel/tracing

    # Trace specific function
    echo function > current_tracer
    echo handle_syscall > set_ftrace_filter
    echo 1 > tracing_on

    # View trace
    cat trace

    # Function graph tracing
    echo function_graph > current_tracer
    echo do_fork > set_graph_function
    ```

    ---

    ## Phase 6: Expert Level - Writing Assembly (Week 11-12)

    ### 6.1 Optimized Memory Copy

    ```asm
    # Fast memcpy for RISC-V (64-byte cache line optimized)
    .global memcpy_fast
    memcpy_fast:
        # Arguments: a0 = dest, a1 = src, a2 = length
        mv   t0, a0                # Save dest for return
        
        # Check alignment
        or   t1, a0, a1
        andi t1, t1, 7
        bnez t1, byte_copy         # Not 8-byte aligned
        
        # 8-byte aligned copy
        srli t1, a2, 3             # t1 = length / 8
        beqz t1, remainder
        
    aligned_loop:
        ld   t2, 0(a1)
        sd   t2, 0(a0)
        addi a0, a0, 8
        addi a1, a1, 8
        addi t1, t1, -1
        bnez t1, aligned_loop
        
    remainder:
        andi a2, a2, 7             # a2 = length % 8
        beqz a2, done
        
    byte_copy:
        lb   t2, 0(a1)
        sb   t2, 0(a0)
        addi a0, a0, 1
        addi a1, a1, 1
        addi a2, a2, -1
        bnez a2, byte_copy
        
    done:
        mv   a0, t0                # Return original dest
        ret
    ```

    ### 6.2 Atomic Operations (AMO Extension)

    ```asm
    # Atomic add
    atomic_add:
        # a0 = address, a1 = value
        amoadd.w.aq zero, a1, (a0) # Atomic add with acquire semantics
        ret

    # Atomic compare-and-swap
    atomic_cas:
        # a0 = address, a1 = expected, a2 = desired
        # Returns old value in a0
    cas_retry:
        lr.w.aq a3, (a0)           # Load-reserved with acquire
        bne  a3, a1, cas_fail      # If not expected, fail
        sc.w.rl a4, a2, (a0)       # Store-conditional with release
        bnez a4, cas_retry         # If failed, retry
        mv   a0, a3                # Return old value
        ret
    cas_fail:
        mv   a0, a3
        ret

    # Spinlock implementation
    spinlock_acquire:
        # a0 = lock address
        li   t0, 1
    spin:
        amoswap.w.aq t1, t0, (a0)  # Atomic swap with acquire
        bnez t1, spin              # If was locked, spin
        ret

    spinlock_release:
        # a0 = lock address
        amoswap.w.rl zero, zero, (a0)  # Atomic swap with release
        ret
    ```

    ### 6.3 Custom Bootloader Example

    ```asm
    # Minimal RISC-V bootloader
    .section .text.boot
    .global _boot
    _boot:
        # Disable all interrupts
        csrw mie, zero
        csrw mip, zero
        
        # Setup machine mode trap vector
        la   t0, m_trap_vector
        csrw mtvec, t0
        
        # Get hart ID
        csrr a0, mhartid
        bnez a0, park_hart         # Only hart 0 continues
        
        # Setup stack
        la   sp, _stack_top
        
        # Clear BSS
        la   t0, _bss_start
        la   t1, _bss_end
    bss_clear:
        sd   zero, 0(t0)
        addi t0, t0, 8
        bltu t0, t1, bss_clear
        
        # Setup PMP (Physical Memory Protection) - allow all
        li   t0, -1
        csrw pmpcfg0, t0
        li   t0, 0x3fffffffffffff
        csrw pmpaddr0, t0
        
        # Delegate interrupts to supervisor mode
        li   t0, 0xffff
        csrw mideleg, t0
        csrw medeleg, t0
        
        # Setup supervisor mode entry point
        la   t0, kernel_entry
        csrw mepc, t0
        
        # Set next mode to supervisor
        li   t0, (1 << 11) | (1 << 5) # MPP=1 (S-mode), MPIE=1
        csrw mstatus, t0
        
        # Jump to kernel in supervisor mode
        mret
        
    park_hart:
        wfi
        j park_hart
        
    m_trap_vector:
        # Simple trap handler
        csrr t0, mcause
        bgez t0, m_exception
        j m_interrupt
        
    m_exception:
        # Handle exception
        j park_hart
        
    m_interrupt:
        # Handle interrupt
        mret
    ```

    ---

    ## Phase 7: Real-World Examples from Your Workspace

    ### 7.1 Analyze Your U-Boot Code

    **Key Files to Study:**
    ```bash
    # Entry point
    cat u-boot/arch/riscv/cpu/start.S

    # Low-level initialization
    cat u-boot/arch/riscv/cpu/cpu.c

    # Board-specific code
    find u-boot -name "*visionfive*" -type f

    # Examine compiled assembly
    riscv64-linux-gnu-objdump -d u-boot/u-boot > u-boot.asm
    ```

    ### 7.2 Analyze Your Linux Kernel

    **Key Files:**
    ```bash
    # Kernel entry
    cat linux/arch/riscv/kernel/head.S

    # Exception handling
    cat linux/arch/riscv/kernel/entry.S

    # System calls
    cat linux/arch/riscv/kernel/syscall_table.c

    # Context switching
    cat linux/arch/riscv/kernel/process.c

    # Generate full disassembly
    riscv64-linux-gnu-objdump -d linux/vmlinux > vmlinux.asm
    ```

    ### 7.3 Debug Your VisionFive2 Boot

    **Based on your existing scripts:**
    ```bash
    # Build with debug symbols
    cd /Users/bpullann/My_project/vision
    ./build_visionfive2.sh

    # Debug U-Boot
    qemu-system-riscv64 -machine virt -bios u-boot/u-boot.bin -nographic -s -S

    # In another terminal
    riscv64-linux-gnu-gdb u-boot/u-boot
    (gdb) target remote :1234
    (gdb) break _start
    (gdb) break board_init_f
    (gdb) continue
    ```

    ---

    ## Learning Resources

    ### Books (From Your Collection)
    1. **Linux Device Drivers (ldd3.pdf)** - Chapter 2 covers module assembly
    2. **Linux Kernel Development** - Chapter on low-level details
    3. **Computer Architecture - Structured Computer Organization**

    ### Online Resources
    1. **RISC-V ISA Specification:** https://riscv.org/technical/specifications/
    2. **RISC-V Assembly Programmer's Manual:** https://github.com/riscv-non-isa/riscv-asm-manual
    3. **SiFive Documentation:** https://www.sifive.com/documentation

    ### Primary References for This Guide

    **1. Linux Kernel Source Code:**
    - **System call definitions:** `linux/include/uapi/asm-generic/unistd.h`
    - **RISC-V syscall table:** `linux/arch/riscv/include/uapi/asm/unistd.h`
    - **Kernel entry/boot:** `linux/arch/riscv/kernel/head.S`
    - **Exception handling:** `linux/arch/riscv/kernel/entry.S`
    - **Context switching:** `linux/arch/riscv/kernel/process.c`
    - Source: https://github.com/torvalds/linux

    **2. U-Boot Bootloader Source:**
    - **RISC-V startup:** `u-boot/arch/riscv/cpu/start.S`
    - **CPU initialization:** `u-boot/arch/riscv/cpu/cpu.c`
    - **Board support:** `u-boot/board/` (various vendors)
    - Source: https://github.com/u-boot/u-boot

    **3. Official RISC-V Specifications:**
    - **RISC-V Unprivileged ISA Specification v20191213**
    - Defines base integer instructions (RV32I, RV64I)
    - Extension specifications (M, A, F, D, C)
    - https://riscv.org/technical/specifications/
    
    - **RISC-V Privileged Architecture Specification v20211203**
    - Machine mode, Supervisor mode, User mode
    - CSR registers (mstatus, mtvec, mepc, etc.)
    - Exception and interrupt handling
    - https://riscv.org/technical/specifications/

    **4. RISC-V Assembly Programmer's Manual:**
    - Pseudo-instructions and their expansions
    - Assembler directives
    - ABI (Application Binary Interface) conventions
    - Source: https://github.com/riscv-non-isa/riscv-asm-manual

    **5. RISC-V ELF psABI (Platform-Specific ABI):**
    - Calling conventions (argument passing, return values)
    - Register usage (caller-saved vs callee-saved)
    - Stack frame layout
    - Source: https://github.com/riscv-non-isa/riscv-elf-psabi-doc

    **6. Linux Syscall Documentation:**
    - **syscalls(2) man page:** System call interface description
    - **Generic syscall numbers:** Based on asm-generic/unistd.h
    - ARM64 and RISC-V share the same syscall numbering (both use 93 for exit)
    - Source: Linux man pages and kernel documentation

    **7. Books Used:**
    - **"The Linux Programming Interface"** by Michael Kerrisk
    - Comprehensive coverage of Linux system calls and programming
    - Chapter 3: System Programming Concepts
    
    - **"Linux Kernel Development" (3rd Edition)** by Robert Love
    - Low-level kernel architecture
    - System call implementation details
    
    - **"Linux Device Drivers" (3rd Edition)** by Corbet, Rubini, Kroah-Hartman
    - Module programming with assembly examples
    - Hardware interaction patterns

    **8. Additional Technical Resources:**
    - **SiFive Freedom Unleashed Documentation**
    - Real-world RISC-V SoC implementation
    - https://www.sifive.com/boards/hifive-unleashed
    
    - **QEMU RISC-V Documentation**
    - Virtual machine emulation and debugging
    - https://www.qemu.org/docs/master/system/target-riscv.html
    
    - **OpenOCD RISC-V Support**
    - On-chip debugging for real hardware
    - https://openocd.org/

    **9. Community Resources:**
    - **RISC-V International Forums:** https://riscv.org/community/
    - **RISC-V Software mailing lists:** Discussion of toolchain, OS ports
    - **Stack Overflow:** Tagged questions on RISC-V assembly

    **10. Your Workspace Resources:**
    - Your cloned Linux kernel: `/Users/bpullann/My_project/linux`
    - Your U-Boot source: `/Users/bpullann/My_project/u-boot`
    - VisionFive2 board scripts: `/Users/bpullann/My_project/vision`
    - Books repository: `/Users/bpullann/My_project/books`

    **Verification Methods Used:**
    - Cross-referenced syscall numbers with actual Linux kernel headers
    - Tested assembly examples with RISC-V toolchain
    - Verified register conventions against RISC-V psABI specification
    - Compared with working code in Linux kernel and U-Boot sources

    **Note on Syscall Number 93:**
    The exit syscall being numbered 93 on RISC-V comes from the generic Linux syscall table 
    (`include/uapi/asm-generic/unistd.h`). RISC-V adopted the same numbering as ARM64 for 
    consistency across 64-bit architectures. You can verify this by examining:
    ```bash
    grep -r "define __NR_exit" linux/include/uapi/asm-generic/unistd.h
    grep -r "define __NR3264_exit" linux/include/uapi/asm-generic/unistd.h
    ```

    ### Tools You Need
    ```bash
    # Install RISC-V toolchain
    brew install riscv-gnu-toolchain  # macOS

    # Or build from source
    git clone https://github.com/riscv/riscv-gnu-toolchain
    cd riscv-gnu-toolchain
    ./configure --prefix=/opt/riscv
    make linux -j$(nproc)
    ```

    ---

    ## Practice Exercises

    ### Week 1-2: Basics
    1. Write "Hello World" in pure RISC-V assembly
    2. Implement basic arithmetic operations
    3. Write a function to find max of array
    4. Implement strlen, strcpy, strcmp

    ### Week 3-4: Intermediate
    1. Implement recursive factorial
    2. Write a bubble sort in assembly
    3. Create a simple linked list manipulation
    4. Implement printf-style integer formatting

    ### Week 5-6: U-Boot Focus
    1. Trace U-Boot execution from _start to board_init
    2. Add custom assembly routine to U-Boot
    3. Modify U-Boot to print register values at boot
    4. Implement a simple memory test in assembly

    ### Week 7-8: Kernel Focus
    1. Trace kernel boot from head.S to start_kernel
    2. Analyze syscall path with GDB
    3. Add custom syscall with assembly entry
    4. Instrument context switch code

    ### Week 9-10: Debugging
    1. Debug kernel panic with GDB
    2. Trace interrupt handling
    3. Analyze page fault handling
    4. Profile function execution with ftrace

    ### Week 11-12: Advanced
    1. Write optimized memcpy for your platform
    2. Implement atomic primitives
    3. Create a minimal kernel module in pure assembly
    4. Write a custom exception handler

    ---

    ## Debugging Cheat Sheet

    ### GDB Commands for RISC-V
    ```bash
    # Start debugging
    riscv64-linux-gnu-gdb <binary>
    (gdb) target remote :1234

    # Breakpoints
    break _start
    break *0x80000000
    break filename.c:42

    # Stepping
    stepi / si              # Step one instruction
    nexti / ni              # Next instruction (skip calls)
    step / s                # Step one line
    next / n                # Next line

    # Registers
    info registers          # All registers
    info register pc sp ra  # Specific registers
    p/x $a0                 # Print register in hex

    # Memory
    x/10i $pc              # Disassemble 10 instructions
    x/10gx $sp             # 10 doublewords at stack
    x/s 0x80000000         # String at address

    # Call stack
    backtrace / bt
    frame <n>
    info frame

    # Watchpoints
    watch *(long*)0x80000000
    rwatch / awatch        # Read / access watchpoint
    ```

    ### QEMU Monitor Commands
    ```bash
    # Press Ctrl+A C to enter QEMU monitor
    info registers         # Show all registers
    info mem              # Show memory map
    info tlb              # Show TLB entries
    x /10i $pc            # Disassemble at PC
    gdbserver             # Start GDB server
    system_reset          # Reset system
    quit                  # Exit QEMU
    ```

    ---

    ## Next Steps

    1. **Start with Phase 1** - Don't skip basics
    2. **Practice daily** - Write small assembly programs
    3. **Read actual code** - Your U-Boot and kernel sources
    4. **Debug everything** - Use GDB extensively
    5. **Document your learning** - Keep notes on patterns you discover
    6. **Contribute** - Once confident, contribute to U-Boot/Linux RISC-V

    ## Success Metrics

    - Week 4: Comfortable reading simple RISC-V assembly
    - Week 6: Can trace U-Boot execution without getting lost
    - Week 8: Understand kernel entry and basic exception handling
    - Week 10: Can debug kernel issues using GDB effectively
    - Week 12: Can write optimized assembly and modify kernel/U-Boot

    Good luck with your RISC-V assembly journey!
