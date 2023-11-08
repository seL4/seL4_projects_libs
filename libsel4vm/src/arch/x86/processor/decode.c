/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/* x86 fetch/decode/emulate code

Author: W.A.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_ram.h>
#include <sel4vm/arch/guest_x86_context.h>

#include "sel4vm/guest_memory.h"

#include "processor/platfeature.h"
#include "processor/decode.h"
#include "processor/msr.h"
#include "guest_state.h"

/* TODO are these defined elsewhere? */
#define IA32_PDE_SIZE(pde) (pde & BIT(7))
#define IA32_PDE_PRESENT(pde) (pde & BIT(0))
#define IA32_PTE_ADDR(pte) (pte & 0xFFFFF000)
#define IA32_PDPTE_ADDR(pdpte) (pdpte & 0xC0000000)
#define IA32_PDE_ADDR(pde) (pde & 0xFFE00000)
#define IA32_PSE_ADDR(pse) (pse & 0xFFC00000)

#define IA32_OPCODE_S(op) (op & BIT(0))
#define IA32_OPCODE_D(op) (op & BIT(1))
#define IA32_OPCODY_BODY(op) (op & 0b11111100)
#define IA32_MODRM_REG(m) ((m & 0b00111000) >> 3)

#define SEG_MULT (0x10)

#define EXTRACT_BITS(num, x, y) ((MASK(x) & ((num) >> (y))))

enum decode_instr {
    DECODE_INSTR_MOV,
    DECODE_INSTR_MOVQ,
    DECODE_INSTR_INVALID
};

enum decode_prefix {
    ES_SEG_OVERRIDE = 0x26,
    CS_SEG_OVERRIDE = 0x2e,
    SS_SEG_OVERRIDE = 0x36,
    DS_SEG_OVERRIDE = 0x3e,
    REX_PREFIX_START = 0x40,
    REX_PREFIX_END = 0x4f,
    FS_SEG_OVERRIDE = 0x64,
    GS_SEG_OVERRIDE = 0x65,
    OP_SIZE_OVERRIDE = 0x66,
    ADDR_SIZE_OVERRIDE = 0x67
};

struct x86_op {
    int reg;
    uint32_t val;
    size_t len;
    size_t reg_mod;
};

struct decode_op {
    int curr_byte;
    uint8_t *instr;
    size_t instr_len;
    struct x86_op op;
};

struct decode_table {
    enum decode_instr instr;
    void (*decode_fn)(struct decode_op *);
};

static void debug_print_instruction(uint8_t *instr, int instr_len)
{
    printf("instruction dump: ");
    for (int j = 0; j < instr_len; j++) {
        printf("%2x ", instr[j]);
    }
    printf("\n");
}

static void decode_modrm_reg_op(struct decode_op *decode_op)
{
    /* Mov with register */
    uint8_t modrm = decode_op->instr[decode_op->curr_byte];
    decode_op->curr_byte++;
    decode_op->op.reg = IA32_MODRM_REG(modrm) + decode_op->op.reg_mod;
    return;
}

static void decode_imm_op(struct decode_op *decode_op)
{
    /* Mov with immediate */
    decode_op->op.reg = -1;
    uint32_t immediate = 0;
    for (int j = 0; j < decode_op->op.len; j++) {
        immediate <<= 8;
        immediate |= decode_op->instr[decode_op->instr_len - j - 1];
    }
    decode_op->op.val = immediate;
    return;
}

static void decode_invalid_op(struct decode_op *decode_op)
{
    ZF_LOGE("can't emulate instruction!");
    debug_print_instruction(decode_op->instr, decode_op->instr_len);
    assert(0);
}

static const struct decode_table decode_table_1op[] = {
    [0 ... MAX_INSTR_OPCODES] = {DECODE_INSTR_INVALID, decode_invalid_op},
    [0x88] = {DECODE_INSTR_MOV, decode_modrm_reg_op},
    [0x89] = {DECODE_INSTR_MOV, decode_modrm_reg_op},
    [0x8a] = {DECODE_INSTR_MOV, decode_modrm_reg_op},
    [0x8b] = {DECODE_INSTR_MOV, decode_modrm_reg_op},
    [0x8c] = {DECODE_INSTR_MOV, decode_modrm_reg_op},
    [0xc6] = {DECODE_INSTR_MOV, decode_imm_op},
    [0xc7] = {DECODE_INSTR_MOV, decode_imm_op}
};

static const struct decode_table decode_table_2op[] = {
    [0 ... MAX_INSTR_OPCODES] = {DECODE_INSTR_INVALID, decode_invalid_op},
    [0x6f] = {DECODE_INSTR_MOVQ, decode_modrm_reg_op}
};

/* Get a word from a guest physical address */
inline static seL4_Word guest_get_phys_word(vm_t *vm, uintptr_t addr)
{
    seL4_Word val;

    vm_ram_touch(vm, addr, sizeof(seL4_Word),
                 vm_guest_ram_read_callback, &val);

    return val;
}

/* Fetch a guest's instruction */
int vm_fetch_instruction(vm_vcpu_t *vcpu, uintptr_t eip, uintptr_t cr3,
                         int len, uint8_t *buf)
{
    /* Walk page tables to get physical address of instruction */
    uintptr_t instr_phys = 0;
    uintptr_t cr4 = vm_guest_state_get_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);

    /* ensure that PAE is not enabled */
#ifndef CONFIG_X86_64_VTX_64BIT_GUESTS
    if (cr4 & X86_CR4_PAE) {
        ZF_LOGE("Do not support walking PAE paging structures");
        return -1;
    }
#endif /* not CONFIG_X86_64_VTX_64BIT_GUESTS */

    int extra_instr = 0;
    int read_instr = len;

    if ((eip >> seL4_PageBits) != ((eip + len) >> seL4_PageBits)) {
        extra_instr = (eip + len) % BIT(seL4_PageBits);
        read_instr -= extra_instr;
    }

    if (cr4 & X86_CR4_PAE) {
        /* assert that pcid is off  */
        assert(!(cr4 & X86_CR4_PCIDE));

        uint64_t eip_47_39 = EXTRACT_BITS(eip, 9, 39);  /* Bits 47:39 of linear address */
        uint64_t eip_38_30 = EXTRACT_BITS(eip, 9, 30);  /* Bits 38:30 of linear address */
        uint64_t eip_29_21 = EXTRACT_BITS(eip, 9, 21);  /* Bits 29:21 of linear address */
        uint64_t eip_20_0 = EXTRACT_BITS(eip, 21, 0);   /* Bits 20:0 of linear address */

        uint64_t pml4e = guest_get_phys_word(vcpu->vm, cr3 | (eip_47_39 << 3));

        assert(IA32_PDE_PRESENT(pml4e));

        uint64_t pdpte = guest_get_phys_word(vcpu->vm, IA32_PTE_ADDR(pml4e) | (eip_38_30 << 3));

        assert(IA32_PDE_PRESENT(pdpte));

        /* If this maps a 1GB page, then we can fetch the instruction now. */
        if (IA32_PDE_SIZE(pdpte)) {
            instr_phys = IA32_PDPTE_ADDR(pdpte) + EXTRACT_BITS(eip, 29, 0);
            goto fetch;
        }

        uint64_t pde = guest_get_phys_word(vcpu->vm, IA32_PTE_ADDR(pdpte) | (eip_29_21 << 3));

        assert(IA32_PDE_PRESENT(pde));

        /* If this maps a 2MB page, then we can fetch the instruction now. */
        if (IA32_PDE_SIZE(pde)) {
            instr_phys = IA32_PDE_ADDR(pde) + eip_20_0;
            goto fetch;
        }

        uint64_t pte = guest_get_phys_word(vcpu->vm, IA32_PTE_ADDR(pde) | (eip_20_0 << 3));

        /* If this maps a 4KB page, then we can fetch the instruction now. */
        if (IA32_PDE_SIZE(pte)) {
            instr_phys = IA32_PTE_ADDR(pte) + EXTRACT_BITS(eip, 11, 0);
            goto fetch;
        }

        return -1;
    } else {
        // TODO implement page-boundary crossing properly
        assert((eip >> 12) == ((eip + len) >> 12));

        uint32_t pdi = eip >> 22;
        uint32_t pti = (eip >> 12) & 0x3FF;

        uint32_t pde = guest_get_phys_word(vcpu->vm, cr3 + pdi * 4);

        assert(IA32_PDE_PRESENT(pde)); /* WTF? */

        if (IA32_PDE_SIZE(pde)) {
            /* PSE is used, 4M pages */
            instr_phys = (uintptr_t)IA32_PSE_ADDR(pde) + (eip & 0x3FFFFF);
        } else {
            /* 4k pages */
            uint32_t pte = guest_get_phys_word(vcpu->vm,
                                               (uintptr_t)IA32_PTE_ADDR(pde) + pti * 4);

            assert(IA32_PDE_PRESENT(pte));

            instr_phys = (uintptr_t)IA32_PTE_ADDR(pte) + (eip & 0xFFF);
        }
    }

fetch:
    /* Fetch instruction */
    vm_ram_touch(vcpu->vm, instr_phys, read_instr,
                 vm_guest_ram_read_callback, buf);

    if (extra_instr > 0) {
        vm_fetch_instruction(vcpu, eip + read_instr, cr3, extra_instr, buf + read_instr);
    }

    return 0;
}

/* Returns 1 if this byte is an x86 instruction prefix */
static int is_prefix(uint8_t byte)
{
    switch (byte) {
    case ES_SEG_OVERRIDE:
    case CS_SEG_OVERRIDE:
    case SS_SEG_OVERRIDE:
    case DS_SEG_OVERRIDE:
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    case REX_PREFIX_START ... REX_PREFIX_END:
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
    case FS_SEG_OVERRIDE:
    case GS_SEG_OVERRIDE:
    case ADDR_SIZE_OVERRIDE:
    case OP_SIZE_OVERRIDE:
        return 1;
    }

    return 0;
}

static int is_high_reg_prefix(uint8_t byte)
{
    switch (byte) {
    case 0x44:
    case 0x4c:
    case 0x4d:
        return 1;
    }
    return 0;
}


/* Partial support to decode an instruction for a memory access
   This is very crude. It can break in many ways. */
int vm_decode_instruction(uint8_t *instr, int instr_len, int *reg, seL4_Word *imm, int *op_len)
{
    struct decode_op dec_op;
    dec_op.instr = instr;
    dec_op.instr_len = instr_len;
    dec_op.op.len = 1;
    dec_op.op.reg_mod = 0;
    /* First loop through and check prefixes */
    int i;
    for (i = 0; i < instr_len; i++) {
        if (is_prefix(instr[i])) {
            if (instr[i] == OP_SIZE_OVERRIDE) {
                /* 16 bit modifier */
                dec_op.op.len = 2;
            }
            if (is_high_reg_prefix(instr[i])) {
                dec_op.op.reg_mod = 8;
            }
        } else {
            /* We've hit the opcode */
            break;
        }
    }

    dec_op.curr_byte = i;
    assert(dec_op.curr_byte < instr_len); /* We still need an opcode */

    uint8_t opcode = instr[dec_op.curr_byte];
    dec_op.curr_byte++;
    if (opcode == OP_ESCAPE) {
        opcode = instr[dec_op.curr_byte];
        dec_op.curr_byte++;
        decode_table_2op[opcode].decode_fn(&dec_op);
    } else {
        decode_table_1op[opcode].decode_fn(&dec_op);
    }

    if (dec_op.op.len != 2 && IA32_OPCODE_S(opcode)) {
        dec_op.op.len = 4;
    }

    *reg = dec_op.op.reg;
    *imm = dec_op.op.val;
    *op_len = dec_op.op.len;
    return 0;
}

void vm_decode_ept_violation(vm_vcpu_t *vcpu, int *reg, seL4_Word *imm, int *size)
{
    /* Decode instruction */
    uint8_t ibuf[15];
    int instr_len = vm_guest_exit_get_int_len(vcpu->vcpu_arch.guest_state);
    vm_fetch_instruction(vcpu,
                         vm_guest_state_get_eip(vcpu->vcpu_arch.guest_state),
                         vm_guest_state_get_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
                         instr_len, ibuf);

    vm_decode_instruction(ibuf, instr_len, reg, imm, size);
}

/*
   Useful information: The GDT loaded by the Linux SMP trampoline looks like:
0x00: 00 00 00 00 00 00 00 00
0x08: 00 00 00 00 00 00 00 00
0x10: ff ff 00 00 00 9b cf 00 <- Executable 0x00000000-0xffffffff
0x18: ff ff 00 00 00 93 cf 00 <- RW data    0x00000000-0xffffffff
*/

/* Interpret just enough virtual 8086 instructions to run trampoline code.
   Returns the final jump address

   For 64-bit guests, this function first emulates the 8086 instructions, and then
   also emulates the 32-bit instructions before returning the final jump address.
   NOTE: This function does not emulate the "call verify_cpu" function, since in
         order to get this far, a 64-bit guest would have to make it through init
         code, thus verifying the cpu.
*/
uintptr_t vm_emulate_realmode(vm_vcpu_t *vcpu, uint8_t *instr_buf,
                              uint16_t *segment, uintptr_t eip, uint32_t len, guest_state_t *gs,
                              int m66_set)
{
    /* We only track one segment, and assume that code and data are in the same
       segment, which is valid for most trampoline and bootloader code */
    uint8_t *instr = instr_buf;
    assert(segment);

    while (instr - instr_buf < len) {
        uintptr_t mem = 0;
        uint32_t lit = 0;
        /* Since 64-bit guests emulate two sections, the second section is already in 32-bit mode,
         * thus every memory read/write will automatically be 4 bytes. This allows the caller to
         * pass in an operating mode
         */
        int m66 = m66_set;

        uint32_t base = 0;
        uint32_t limit = 0;

        if (*instr == 0x66) {
            m66 = 1;
            instr++;
        }

        if (*instr == 0x0f) {
            instr++;
            if (*instr == 0x01) {
                instr++;
                if (*instr == 0x1e) {
                    // lidtl
                    instr++;
                    memcpy(&mem, instr, 2);
                    mem += *segment * SEG_MULT;
                    instr += 2;

                    /* Limit is first 2 bytes, base is next 4 bytes */
                    vm_ram_touch(vcpu->vm, mem,
                                 2, vm_guest_ram_read_callback, &limit);
                    vm_ram_touch(vcpu->vm, mem + 2,
                                 4, vm_guest_ram_read_callback, &base);
                    ZF_LOGD("lidtl %p", (void *)mem);

                    vm_guest_state_set_idt_base(gs, base);
                    vm_guest_state_set_idt_limit(gs, limit);
                } else if (*instr == 0x16) {
                    // lgdtl
                    instr++;
                    memcpy(&mem, instr, 2);
                    mem += *segment * SEG_MULT;
                    instr += 2;

                    /* Limit is first 2 bytes, base is next 4 bytes */
                    vm_ram_touch(vcpu->vm, mem,
                                 2, vm_guest_ram_read_callback, &limit);
                    vm_ram_touch(vcpu->vm, mem + 2,
                                 4, vm_guest_ram_read_callback, &base);
                    ZF_LOGD("lgdtl %p; base = %x, limit = %x", (void *)mem,
                            base, limit);

                    vm_guest_state_set_gdt_base(gs, base);
                    vm_guest_state_set_gdt_limit(gs, limit);
                } else {
                    //ignore
                    instr++;
                }
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
            } else if (*instr == 0x22) {
                // mov eax crX
                instr++;
                seL4_Word eax;
                vm_get_thread_context_reg(vcpu, USER_CONTEXT_EAX, &eax);

                if (*instr == 0xc0) {
                    vm_guest_state_set_cr0(gs, eax);
                    ZF_LOGD("cr0 %lx", (long unsigned int)eax);
                }
                if (*instr == 0xd8) {
                    vm_guest_state_set_cr3(gs, eax);
                    ZF_LOGD("cr3 %lx", (long unsigned int)eax);
                }
                if (*instr == 0xe0) {
                    vm_guest_state_set_cr4(gs, eax);
                    ZF_LOGD("cr4 %lx", (long unsigned int)eax);
                }
            } else if (*instr == 0x30) {
                // wrmsr
                instr++;
                seL4_Word eax;
                seL4_Word ecx;
                seL4_Word edx;

                vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &eax);
                vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, &ecx);
                vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, &edx);
                if (MSR_EFER == ecx) {
                    vm_set_vmcs_field(vcpu, VMX_GUEST_EFER, (edx << 32) | eax);
                    ZF_LOGD("wrmsr %lx %lx", ecx, (edx << 32) | eax);
                }
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
            } else {
                //ignore
                instr++;
            }
        } else if (*instr == 0xea) {
            /* Absolute jmp */
            instr++;
            uint32_t base = 0;
            uintptr_t jmp_addr = 0;
            if (m66) {
                // base is 4 bytes
                /* Make the wild assumptions that we are now in protected mode
                   and the relevant GDT entry just covers all memory. Therefore
                   the base address is our absolute address. This just happens
                   to work with Linux and probably other modern systems that
                   don't use the GDT much. */
                memcpy(&base, instr, 4);
                instr += 4;
                jmp_addr = base;
                memcpy(segment, instr, 2);
            } else {
                memcpy(&base, instr, 2);
                instr += 2;
                memcpy(segment, instr, 2);
                jmp_addr = *segment * SEG_MULT + base;
            }
            instr += 2;
            ZF_LOGD("absolute jmpf $%p, cs now %04x", (void *)jmp_addr, *segment);
            if (((int64_t)jmp_addr - (int64_t)(len + eip)) >= 0) {
                vm_guest_state_set_cs_selector(gs, *segment);
                return jmp_addr;
            } else {
                instr = jmp_addr - eip + instr_buf;
            }
        } else {
            switch (*instr) {
            case 0xa1:
                /* mov offset memory to eax */
                instr++;
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
                memcpy(&mem, instr, 4);
                instr += 4;
#else
                memcpy(&mem, instr, 2);
                instr += 2;
                mem += *segment * SEG_MULT;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
                ZF_LOGD("mov %p, eax", (void *)mem);
                uint32_t eax;
                vm_ram_touch(vcpu->vm, mem,
                             4, vm_guest_ram_read_callback, &eax);
                vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, eax);
                break;
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
            case 0xb8:
                /* mov const to eax */
                instr++;
                memcpy(&mem, instr, 4);
                instr += 4;
                ZF_LOGD("mov %lx, eax", mem);
                vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, mem);
                break;
            case 0xb9:
                /* mov const to ecx */
                instr++;
                memcpy(&mem, instr, 4);
                instr += 4;
                ZF_LOGD("mov %lx, ecx", mem);
                vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, mem);
                break;
            case 0x8b:
                /* mov offset memory to edx */
                instr++;
                if (*instr == 0x15) {
                    instr++;
                    memcpy(&mem, instr, 4);
                    instr += 4;
                    uint32_t edx;
                    vm_ram_touch(vcpu->vm, mem,
                                 4, vm_guest_ram_read_callback, &edx);
                    ZF_LOGD("mov %x, edx", edx);
                    vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, mem);
                }
                break;
            case 0x81:
                instr++;
                if (*instr = 0xc4) {
                    /* add lit to rsp */
                    instr++;
                    memcpy(&mem, instr, 4);
                    instr += 4;
                    seL4_Word esp = vm_guest_state_get_esp(gs, mem);
                    esp += mem;
                    vm_guest_state_set_esp(gs, esp);
                    ZF_LOGD("add %lx, rsp", mem);
                }
                break;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
            case 0xc7:
                instr++;
                if (*instr == 0x06) { // modrm
                    int size;
                    instr++;
                    /* mov literal to memory */
                    memcpy(&mem, instr, 2);
                    mem += *segment * SEG_MULT;
                    instr += 2;
                    if (m66) {
                        memcpy(&lit, instr, 4);
                        size = 4;
                    } else {
                        memcpy(&lit, instr, 2);
                        size = 2;
                    }
                    instr += size;
                    ZF_LOGD("mov $0x%x, %p", lit, (void *)mem);
                    vm_ram_touch(vcpu->vm, mem,
                                 size, vm_guest_ram_write_callback, &lit);
                }
                break;
            case 0xba:
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
                /* mov const to edx */
                instr++;
                memcpy(&mem, instr, 4);
                instr += 4;
                ZF_LOGD("mov %lx, edx", mem);
                vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, mem);
#else
                //?????mov literal to dx
                /* ignore */
                instr += 2;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
                break;
            case 0xbc:
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
                // mov lit esp
                instr++;
                memcpy(&mem, instr, 4);
                instr += 4;
                ZF_LOGD("mov %lx, esp", mem);
                vm_guest_state_set_esp(gs, mem);
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
                break;
            case 0x8c:
                /* mov to/from sreg. ignore */
                instr += 2;
                break;
            case 0x8e:
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
                // mov eax/edx -> segment register
                instr++;

                seL4_Word val = 0;

                if ((*instr == 0xc0) || (*instr == 0xd0) || (*instr == 0xd8)) {
                    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &val);
                } else if ((*instr == 0xc2) || (*instr == 0xd2) || (*instr == 0xda)
                           || (*instr == 0xe2) || (*instr == 0xea)) {
                    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, &val);
                }

                /* Mask everything but lowest 16 bits */
                val &= 0xffff;

                if ((*instr == 0xd0) || (*instr == 0xd2)) {
                    vm_guest_state_set_ss_selector(gs, val);
                    ZF_LOGD("ss %lx", (long unsigned int)val);
                } else if ((*instr == 0xd8) || (*instr == 0xda)) {
                    vm_guest_state_set_ds_selector(gs, val);
                    ZF_LOGD("ds %lx", (long unsigned int)val);
                } else if ((*instr == 0xc0) || (*instr == 0xc2)) {
                    vm_guest_state_set_es_selector(gs, val);
                    ZF_LOGD("es %lx", (long unsigned int)val);
                } else if (*instr == 0xe2) {
                    vm_guest_state_set_fs_selector(gs, val);
                    ZF_LOGD("fs %lx", (long unsigned int)val);
                } else if (*instr == 0xea) {
                    vm_guest_state_set_gs_selector(gs, val);
                    ZF_LOGD("gs %lx", (long unsigned int)val);
                }

                instr++;
#else
                /* mov to/from sreg. ignore */
                instr += 2;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
                break;
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
            case 0x75:
            /* jne */
            case 0x85:
                /* test eax, eax */
                instr += 2;
                break;
            case 0xe8:
                /* call rel */
                instr += 3;
                break;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
            default:
                /* Assume this is a single byte instruction we can ignore */
                instr++;
            }
        }

        ZF_LOGI("read %zu bytes", (size_t)(instr - instr_buf));
    }

    return 0;
}
