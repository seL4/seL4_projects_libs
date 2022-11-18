/*
 * Local APIC virtualization
 *
 * Copyright (C) 2006 Qumranet, Inc.
 * Copyright (C) 2007 Novell
 * Copyright (C) 2007 Intel
 * Copyright 2009 Red Hat, Inc. and/or its affiliates.
 *
 * Authors:
 *   Dor Laor <dor.laor@qumranet.com>
 *   Gregory Haskins <ghaskins@novell.com>
 *   Yaozu (Eddie) Dong <eddie.dong@intel.com>
 *
 * Based on Xen 3.1 code, Copyright (c) 2004, Intel Corporation.
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

// SPDX-License-Identifier: GPL-2.0-only

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <utils/util.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/arch/guest_x86_irq_controller.h>
#include <platsupport/arch/tsc.h>

#include "processor/lapic.h"
#include "processor/apicdef.h"
#include "processor/msr.h"
#include "i8259.h"
#include "interrupt.h"

#define APIC_BUS_CYCLE_NS 1

#define APIC_DEBUG 0
#define apic_debug(lvl,...) do{ if(lvl < APIC_DEBUG){printf(__VA_ARGS__);fflush(stdout);}}while (0)

#define mod_64(x, y) ((x) - (y) * ((x) / (y)))

#define APIC_LVT_NUM            6
/* 14 is the version for Xeon and Pentium 8.4.8*/
#define APIC_VERSION            (0x14UL | ((APIC_LVT_NUM - 1) << 16))
#define LAPIC_MMIO_LENGTH       (BIT(12))
/* followed define is not in apicdef.h */
#define APIC_SHORT_MASK         0xc0000
#define APIC_DEST_NOSHORT       0x0
#define APIC_DEST_MASK          0x800
#define MAX_APIC_VECTOR         256
#define APIC_VECTORS_PER_REG        32

/* This needs to be generalised to per-cpu for SMP support */
irq_info_t irq_info[I8259_NR_IRQS + LAPIC_NR_IRQS];

/* id used to identify this timer in the timer_server component */
static int ts_id;
static uint64_t tsc_frequency = 0;

static int64_t current_time_ns()
{
    return (int64_t) muldivu64(rdtsc_pure(), NS_IN_S, tsc_frequency);
}

static inline uint64_t timer_tsc_freq(vm_lapic_t *apic)
{
    return apic->lapic_timer.timer_emul->tsc_freq();
}

static inline int timer_oneshot_absolute(vm_lapic_t *apic, uint64_t ns)
{
    return apic->lapic_timer.timer_emul->oneshot_absolute(ns);
}

static inline int timer_stop(vm_lapic_t *apic)
{
    return apic->lapic_timer.timer_emul->stop();
}

inline static int pic_get_interrupt(vm_t *vm)
{
    return i8259_get_interrupt(vm);
}

inline static int pic_has_interrupt(vm_t *vm)
{
    return i8259_has_interrupt(vm);
}

struct vm_lapic_irq {
    uint32_t vector;
    uint32_t delivery_mode;
    uint32_t dest_mode;
    uint32_t level;
    uint32_t trig_mode;
    uint32_t shorthand;
    uint32_t dest_id;
};

/* Generic bit operations; TODO move these elsewhere */
static inline int fls(int x)
{
    int r = 32;

    if (!x) {
        return 0;
    }

    if (!(x & 0xffff0000u)) {
        x <<= 16;
        r -= 16;
    }
    if (!(x & 0xff000000u)) {
        x <<= 8;
        r -= 8;
    }
    if (!(x & 0xf0000000u)) {
        x <<= 4;
        r -= 4;
    }
    if (!(x & 0xc0000000u)) {
        x <<= 2;
        r -= 2;
    }
    if (!(x & 0x80000000u)) {
        x <<= 1;
        r -= 1;
    }
    return r;
}

static uint32_t hweight32(unsigned int w)
{
    uint32_t res = w - ((w >> 1) & 0x55555555);
    res = (res & 0x33333333) + ((res >> 2) & 0x33333333);
    res = (res + (res >> 4)) & 0x0F0F0F0F;
    res = res + (res >> 8);
    return (res + (res >> 16)) & 0x000000FF;
}
/* End generic bit ops */

void vm_lapic_reset(vm_vcpu_t *vcpu);

// Returns whether the irq delivery mode is lowest prio
inline static bool vm_is_dm_lowest_prio(struct vm_lapic_irq *irq)
{
    return irq->delivery_mode == APIC_DM_LOWEST;
}

// Access register field
static inline void apic_set_reg(vm_lapic_t *apic, int reg_off, uint32_t val)
{
    *((uint32_t *)(apic->regs + reg_off)) = val;
}

static inline uint32_t vm_apic_get_reg(vm_lapic_t *apic, int reg_off)
{
    return *((uint32_t *)(apic->regs + reg_off));
}

static inline int apic_test_vector(int vec, void *bitmap)
{
    return ((1UL << (vec & 31)) & ((uint32_t *)bitmap)[vec >> 5]) != 0;
}

bool vm_apic_pending_eoi(vm_vcpu_t *vcpu, int vector)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;

    return apic_test_vector(vector, apic->regs + APIC_ISR) ||
           apic_test_vector(vector, apic->regs + APIC_IRR);
}

static inline void apic_set_vector(int vec, void *bitmap)
{
    ((uint32_t *)bitmap)[vec >> 5] |= 1UL << (vec & 31);
}

static inline void apic_clear_vector(int vec, void *bitmap)
{
    ((uint32_t *)bitmap)[vec >> 5] &= ~(1UL << (vec & 31));
}

static inline int vm_apic_sw_enabled(vm_lapic_t *apic)
{
    return vm_apic_get_reg(apic, APIC_SPIV) & APIC_SPIV_APIC_ENABLED;
}

static inline int vm_apic_hw_enabled(vm_lapic_t *apic)
{
    return apic->apic_base & MSR_IA32_APICBASE_ENABLE;
}

inline int vm_apic_enabled(vm_lapic_t *apic)
{
    return vm_apic_sw_enabled(apic) && vm_apic_hw_enabled(apic);
}

#define LVT_MASK    \
    (APIC_LVT_MASKED | APIC_SEND_PENDING | APIC_VECTOR_MASK)

#define LINT_MASK   \
    (LVT_MASK | APIC_MODE_MASK | APIC_INPUT_POLARITY | \
     APIC_LVT_REMOTE_IRR | APIC_LVT_LEVEL_TRIGGER)

static inline int vm_apic_id(vm_lapic_t *apic)
{
    return (vm_apic_get_reg(apic, APIC_ID) >> 24) & 0xff;
}

static inline void apic_set_spiv(vm_lapic_t *apic, uint32_t val)
{
    apic_set_reg(apic, APIC_SPIV, val);
}

static const unsigned int apic_lvt_mask[APIC_LVT_NUM] = {
    LVT_MASK,       /* part LVTT mask, timer mode mask added at runtime */
    LVT_MASK | APIC_MODE_MASK,  /* LVTTHMR */
    LVT_MASK | APIC_MODE_MASK,  /* LVTPC */
    LINT_MASK, LINT_MASK,   /* LVT0-1 */
    LVT_MASK        /* LVTERR */
};

static inline void vm_apic_set_id(vm_lapic_t *apic, uint8_t id)
{
    apic_set_reg(apic, APIC_ID, id << 24);
}

static inline void vm_apic_set_ldr(vm_lapic_t *apic, uint32_t id)
{
    apic_set_reg(apic, APIC_LDR, id);
}

static inline int apic_lvt_enabled(vm_lapic_t *apic, int lvt_type)
{
    return !(vm_apic_get_reg(apic, lvt_type) & APIC_LVT_MASKED);
}

static inline int apic_lvt_vector(vm_lapic_t *apic, int lvt_type)
{
    return vm_apic_get_reg(apic, lvt_type) & APIC_VECTOR_MASK;
}

static inline int vm_vcpu_is_bsp(vm_vcpu_t *vcpu)
{
    return vcpu->vcpu_id == BOOT_VCPU;
}

static inline int apic_lvt_nmi_mode(uint32_t lvt_val)
{
    return (lvt_val & (APIC_MODE_MASK | APIC_LVT_MASKED)) == APIC_DM_NMI;
}

int vm_apic_compare_prio(vm_vcpu_t *vcpu1, vm_vcpu_t *vcpu2)
{
    return vcpu1->vcpu_arch.lapic->arb_prio - vcpu2->vcpu_arch.lapic->arb_prio;
}

static void UNUSED dump_vector(const char *name, void *bitmap)
{
    int vec;
    uint32_t *reg = bitmap;

    printf("%s = 0x", name);

    for (vec = MAX_APIC_VECTOR - APIC_VECTORS_PER_REG;
         vec >= 0; vec -= APIC_VECTORS_PER_REG) {
        printf("%08x", reg[vec >> 5]);
    }

    printf("\n");
}

static inline int apic_lvtt_oneshot(vm_lapic_t *apic)
{
    return apic->lapic_timer.timer_mode == APIC_LVT_TIMER_ONESHOT;
}

static inline int apic_lvtt_period(vm_lapic_t *apic)
{
    return apic->lapic_timer.timer_mode == APIC_LVT_TIMER_PERIODIC;
}

static inline int apic_lvtt_tscdeadline(vm_lapic_t *apic)
{
    return apic->lapic_timer.timer_mode == APIC_LVT_TIMER_TSCDEADLINE;
}

static int find_highest_vector(void *bitmap)
{
    int vec;
    uint32_t *reg = bitmap;

    for (vec = MAX_APIC_VECTOR - APIC_VECTORS_PER_REG;
         vec >= 0; vec -= APIC_VECTORS_PER_REG) {
        if (reg[vec >> 5]) {
            return fls(reg[vec >> 5]) - 1 + vec;
        }
    }

    return -1;
}

static uint8_t UNUSED count_vectors(void *bitmap)
{
    int vec;
    uint32_t *reg = bitmap;
    uint8_t count = 0;

    for (vec = 0; vec < MAX_APIC_VECTOR; vec += APIC_VECTORS_PER_REG) {
        count += hweight32(reg[vec >> 5]);
    }

    return count;
}

static inline int apic_search_irr(vm_lapic_t *apic)
{
    return find_highest_vector(apic->regs + APIC_IRR);
}

static inline int apic_find_highest_irr(vm_lapic_t *apic)
{
    int result;

    if (!apic->irr_pending) {
        return -1;
    }

    result = apic_search_irr(apic);
    assert(result == -1 || result >= 16);

    return result;
}

static inline void apic_set_irr(int vec, vm_lapic_t *apic)
{
    if (vec != 0x30) {
        apic_debug(5, "!setting irr 0x%x\n", vec);
    }

    apic->irr_pending = true;
    apic_set_vector(vec, apic->regs + APIC_IRR);
}

static inline void apic_clear_irr(int vec, vm_lapic_t *apic)
{
    apic_clear_vector(vec, apic->regs + APIC_IRR);

    vec = apic_search_irr(apic);
    apic->irr_pending = (vec != -1);
}

static inline void apic_set_isr(int vec, vm_lapic_t *apic)
{
    if (apic_test_vector(vec, apic->regs + APIC_ISR)) {
        return;
    }
    apic_set_vector(vec, apic->regs + APIC_ISR);

    ++apic->isr_count;
    /*
     * ISR (in service register) bit is set when injecting an interrupt.
     * The highest vector is injected. Thus the latest bit set matches
     * the highest bit in ISR.
     */
}

static inline int apic_find_highest_isr(vm_lapic_t *apic)
{
    int result;

    /*
     * Note that isr_count is always 1, and highest_isr_cache
     * is always -1, with APIC virtualization enabled.
     */
    if (!apic->isr_count) {
        return -1;
    }
    if (apic->highest_isr_cache != -1) {
        return apic->highest_isr_cache;
    }

    result = find_highest_vector(apic->regs + APIC_ISR);
    assert(result == -1 || result >= 16);

    return result;
}

static inline void apic_clear_isr(int vec, vm_lapic_t *apic)
{
    if (!apic_test_vector(vec, apic->regs + APIC_ISR)) {
        return;
    }
    apic_clear_vector(vec, apic->regs + APIC_ISR);

    --apic->isr_count;
    apic->highest_isr_cache = -1;
}

int vm_lapic_find_highest_irr(vm_vcpu_t *vcpu)
{
    int highest_irr;

    highest_irr = apic_find_highest_irr(vcpu->vcpu_arch.lapic);

    return highest_irr;
}

static int __apic_accept_irq(vm_vcpu_t *vcpu, int delivery_mode,
                             int vector, int level, int trig_mode,
                             unsigned long *dest_map);

int vm_apic_set_irq(vm_vcpu_t *vcpu, struct vm_lapic_irq *irq,
                    unsigned long *dest_map)
{
    return __apic_accept_irq(vcpu, irq->delivery_mode, irq->vector,
                             irq->level, irq->trig_mode, dest_map);
}

void vm_apic_update_tmr(vm_vcpu_t *vcpu, uint32_t *tmr)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    int i;

    for (i = 0; i < 8; i++) {
        apic_set_reg(apic, APIC_TMR + 0x10 * i, tmr[i]);
    }
}

static void apic_update_ppr(vm_vcpu_t *vcpu)
{
    /* Intel SDM 10.8.3.1 Task and Processor Priorities */
    uint32_t tpr, isrv, ppr, old_ppr;
    int isr;
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;

    old_ppr = vm_apic_get_reg(apic, APIC_PROCPRI);
    tpr = vm_apic_get_reg(apic, APIC_TASKPRI);
    isr = apic_find_highest_isr(apic);
    isrv = (isr != -1) ? isr : 0;

    if ((tpr & 0xf0) >= (isrv & 0xf0)) {
        ppr = tpr & 0xff;
    } else {
        ppr = isrv & 0xf0;
    }

    apic_debug(6, "vlapic %p, ppr 0x%x, isr 0x%x, isrv 0x%x\n",
               apic, ppr, isr, isrv);

    if (old_ppr != ppr) {
        apic_set_reg(apic, APIC_PROCPRI, ppr);
        if (ppr < old_ppr) {
            /* Might have unmasked some pending interrupts */
            vm_vcpu_accept_interrupt(vcpu);
        }
    }
}

static void apic_set_tpr(vm_vcpu_t *vcpu, uint32_t tpr)
{
    apic_set_reg(vcpu->vcpu_arch.lapic, APIC_TASKPRI, tpr);
    apic_debug(3, "vcpu %d lapic TPR set to %d\n", vcpu->vcpu_id, tpr);
    apic_update_ppr(vcpu);
}

int vm_apic_match_physical_addr(vm_lapic_t *apic, uint16_t dest)
{
    return dest == 0xff || vm_apic_id(apic) == dest;
}

int vm_apic_match_logical_addr(vm_lapic_t *apic, uint8_t mda)
{
    int result = 0;
    uint32_t logical_id;

    logical_id = GET_APIC_LOGICAL_ID(vm_apic_get_reg(apic, APIC_LDR));

    switch (vm_apic_get_reg(apic, APIC_DFR)) {
    case APIC_DFR_FLAT:
        if (logical_id & mda) {
            result = 1;
        }
        break;
    case APIC_DFR_CLUSTER:
        if (((logical_id >> 4) == (mda >> 0x4))
            && (logical_id & mda & 0xf)) {
            result = 1;
        }
        break;
    default:
        apic_debug(1, "Bad DFR: %08x\n", vm_apic_get_reg(apic, APIC_DFR));
        break;
    }

    return result;
}

int vm_apic_match_dest(vm_vcpu_t *vcpu, vm_lapic_t *source,
                       int short_hand, int dest, int dest_mode)
{
    int result = 0;
    vm_lapic_t *target = vcpu->vcpu_arch.lapic;

    assert(target);
    switch (short_hand) {
    case APIC_DEST_NOSHORT:
        if (dest_mode == 0)
            /* Physical mode. */
        {
            result = vm_apic_match_physical_addr(target, dest);
        } else
            /* Logical mode. */
        {
            result = vm_apic_match_logical_addr(target, dest);
        }
        break;
    case APIC_DEST_SELF:
        result = (target == source);
        break;
    case APIC_DEST_ALLINC:
        result = 1;
        break;
    case APIC_DEST_ALLBUT:
        result = (target != source);
        break;
    default:
        apic_debug(2, "apic: Bad dest shorthand value %x\n",
                   short_hand);
        break;
    }

    apic_debug(4, "target %p, source %p, dest 0x%x, "
               "dest_mode 0x%x, short_hand 0x%x",
               target, source, dest, dest_mode, short_hand);
    if (result) {
        apic_debug(4, " MATCH\n");
    } else {
        apic_debug(4, "\n");
    }

    return result;
}

int vm_irq_delivery_to_apic(vm_vcpu_t *src_vcpu, struct vm_lapic_irq *irq, unsigned long *dest_map)
{
    int i, r = -1;
    vm_lapic_t *src = src_vcpu->vcpu_arch.lapic;
    vm_t *vm = src_vcpu->vm;

    vm_vcpu_t *lowest = NULL;

    if (irq->shorthand == APIC_DEST_SELF) {
        return vm_apic_set_irq(src_vcpu, irq, dest_map);
    }

    vm_vcpu_t *dest_vcpu = vm->vcpus[BOOT_VCPU];

    if (!vm_apic_hw_enabled(dest_vcpu->vcpu_arch.lapic)) {
        return r;
    }

    if (!vm_apic_match_dest(dest_vcpu, src, irq->shorthand,
                            irq->dest_id, irq->dest_mode)) {
        return r;
    }

    if (!vm_is_dm_lowest_prio(irq) || (vm_apic_enabled(dest_vcpu->vcpu_arch.lapic))) {
        r = vm_apic_set_irq(dest_vcpu, irq, dest_map);
    }

    return r;
}

/*
 * Add a pending IRQ into lapic.
 * Return 1 if successfully added and 0 if discarded.
 */
static int __apic_accept_irq(vm_vcpu_t *vcpu, int delivery_mode,
                             int vector, int level, int trig_mode,
                             unsigned long *dest_map)
{
    int result = 0;
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;

    switch (delivery_mode) {
    case APIC_DM_LOWEST:
        apic->arb_prio++;
    case APIC_DM_FIXED:
        /* FIXME add logic for vcpu on reset */
        if (!vm_apic_enabled(apic)) {
            break;
        }

        /** Note: lapic.c currently assumes all trigger mode for lowest/fixed interrupt
         * to be edge triggered, which is why the APIC_TMR register is not cleared.
        */

        apic_debug(4, "###fixed int 0x%x to vcpu %d\n", vector, vcpu->vcpu_id);

        result = 1;
        apic_set_irr(vector, apic);
        vm_vcpu_accept_interrupt(vcpu);
        break;

    case APIC_DM_NMI:
    case APIC_DM_REMRD:
        result = 1;
        vm_vcpu_accept_interrupt(vcpu);
        break;

    case APIC_DM_SMI:
        apic_debug(2, "Ignoring guest SMI\n");
        break;

    case APIC_DM_INIT:
        apic_debug(2, "Got init ipi on vcpu %d\n", vcpu->vcpu_id);
        if (!trig_mode || level) {
            if (apic->state == LAPIC_STATE_RUN) {
                /* Already running, ignore inits */
                break;
            }
            result = 1;
            vm_lapic_reset(vcpu);
            apic->arb_prio = vm_apic_id(apic);
            apic->state = LAPIC_STATE_WAITSIPI;
        } else {
            apic_debug(2, "Ignoring de-assert INIT to vcpu %d\n",
                       vcpu->vcpu_id);
        }
        break;

    case APIC_DM_STARTUP:
        if (apic->state != LAPIC_STATE_WAITSIPI) {
            apic_debug(1, "Received SIPI while processor was not in wait for SIPI state\n");
        } else {
            apic_debug(2, "SIPI to vcpu %d vector 0x%02x\n",
                       vcpu->vcpu_id, vector);
            result = 1;
            apic->sipi_vector = vector;
            apic->state = LAPIC_STATE_RUN;

            /* Start the VCPU thread. */
            vm_start_ap_vcpu(vcpu, vector);
        }
        break;

    case APIC_DM_EXTINT:
        /* extints are handled by vm_apic_consume_extints */
        printf("extint should not come to this function. vcpu %d\n", vcpu->vcpu_id);
        assert(0);
        break;

    default:
        printf("TODO: unsupported lapic ipi delivery mode %x", delivery_mode);
        assert(0);
        break;
    }
    return result;
}

static int apic_set_eoi(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    int vector = apic_find_highest_isr(apic);

    /*
     * Not every write EOI will has corresponding ISR,
     * one example is when Kernel check timer on setup_IO_APIC
     */
    if (vector == -1) {
        return vector;
    }

    if (vector < NR_IRQS && irq_info[vector].callback) {
        /* These callbacks are only setup for the PIC and APIC. For MSIs, the
         * guest will choose what vector the interrupts head to and typically
         * above 32 (our max supported irqs). Plus I don't think the MSIs need
         * an explicit EOI signal.
         * TODO: investigate */
        irq_info[vector].callback(vcpu, vector, (void *) irq_info[vector].cookie);
    }

    apic_clear_isr(vector, apic);
    apic_update_ppr(vcpu);

    /* If another interrupt is pending, raise it */
    vm_vcpu_accept_interrupt(vcpu);

    return vector;
}

static void apic_send_ipi(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    uint32_t icr_low = vm_apic_get_reg(apic, APIC_ICR);
    uint32_t icr_high = vm_apic_get_reg(apic, APIC_ICR2);
    struct vm_lapic_irq irq;

    irq.vector = icr_low & APIC_VECTOR_MASK;
    irq.delivery_mode = icr_low & APIC_MODE_MASK;
    irq.dest_mode = icr_low & APIC_DEST_MASK;
    irq.level = icr_low & APIC_INT_ASSERT;
    irq.trig_mode = icr_low & APIC_INT_LEVELTRIG;
    irq.shorthand = icr_low & APIC_SHORT_MASK;
    irq.dest_id = GET_APIC_DEST_FIELD(icr_high);

    apic_debug(3, "icr_high 0x%x, icr_low 0x%x, "
               "short_hand 0x%x, dest 0x%x, trig_mode 0x%x, level 0x%x, "
               "dest_mode 0x%x, delivery_mode 0x%x, vector 0x%x\n",
               icr_high, icr_low, irq.shorthand, irq.dest_id,
               irq.trig_mode, irq.level, irq.dest_mode, irq.delivery_mode,
               irq.vector);

    vm_irq_delivery_to_apic(vcpu, &irq, NULL);
}

static uint32_t apic_get_tmcct(vm_lapic_t *apic)
{
    int64_t remaining, now, ns;
    uint32_t tmcct;

    /* if initial count is 0, current count should also be 0 */
    if (vm_apic_get_reg(apic, APIC_TMICT) == 0 ||
        apic->lapic_timer.period == 0) {
        return 0;
    }

    now = current_time_ns();
    remaining = apic->lapic_timer.target_expiration - now;
    if (remaining < 0) {
        remaining = 0;
    }

    ns = mod_64(remaining, apic->lapic_timer.period);
    tmcct = ns / (APIC_BUS_CYCLE_NS * apic->divide_count);

    return tmcct;
}

static uint32_t __apic_read(vm_lapic_t *apic, unsigned int offset)
{
    uint32_t val = 0;

    if (offset >= LAPIC_MMIO_LENGTH) {
        return 0;
    }

    switch (offset) {
    case APIC_ID:
        val = vm_apic_id(apic) << 24;
        break;
    case APIC_ARBPRI:
        apic_debug(2, "Access APIC ARBPRI register which is for P6\n");
        break;

    case APIC_TMCCT:
        if (apic_lvtt_tscdeadline(apic)) {
            /* Shouldn't even get to here but just in case */
            return 0;
        }
        // val = apic_get_tmcct(apic);
        break;
    case APIC_PROCPRI:
        /* TODO: unknown if this is needed, Xen does not update PPR
         * while KVM does when reading the PROCPRI register */
        // apic_update_ppr(apic->vcpu);
        val = vm_apic_get_reg(apic, offset);
        break;
    default:
        val = vm_apic_get_reg(apic, offset);
        break;
    }

    return val;
}

static void vm_apic_inject_pending_timer_irqs(vm_lapic_t *apic)
{
    vm_apic_local_deliver(apic->vcpu, APIC_LVTT);
    if (apic_lvtt_oneshot(apic)) {
        apic->lapic_timer.tscdeadline = 0; // no clue why this is set to 0 tbh
        apic->lapic_timer.target_expiration = 0;
    }
}

static void apic_timer_expired(vm_lapic_t *apic, bool from_timer_fn)
{
    vm_vcpu_t *vcpu = apic->vcpu;

    if (apic->lapic_timer.pending) {
        // ZF_LOGE("Current pending timer IRQ");
        return;
    }

    if (!from_timer_fn && vm_apic_enabled(apic)) {
        vm_apic_inject_pending_timer_irqs(apic);
        return;
    }

    vm_apic_inject_pending_timer_irqs(apic);
    apic->lapic_timer.pending++;
    if (from_timer_fn) {
        vm_vcpu_accept_interrupt(vcpu);
    }
}

static void update_divide_count(vm_lapic_t *apic)
{
    uint32_t tmp1, tmp2, tdcr;

    tdcr = vm_apic_get_reg(apic, APIC_TDCR);
    tmp1 = tdcr & 0xf;
    tmp2 = ((tmp1 & 0x3) | ((tmp1 & 0x8) >> 1)) + 1;
    apic->divide_count = 0x1 << (tmp2 & 0x7);
}


static void limit_periodic_timer_frequency(vm_lapic_t *apic)
{
    /*
     * Do not allow the guest to program periodic timers with small
     * interval (cuz that's what KVM does lmfao).
     */
    if (apic_lvtt_period(apic) && apic->lapic_timer.period) {
        int64_t min_period = MIN_TIMER_PERIOD_US * 1000LL;

        if (apic->lapic_timer.period < min_period) {
            apic->lapic_timer.period = min_period;
        }
    }
}

static inline int64_t tmict_to_ns(vm_lapic_t *apic, uint32_t tmict)
{
    return (int64_t) tmict * APIC_BUS_CYCLE_NS * (int64_t) apic->divide_count;
}

static void update_target_expiration(vm_lapic_t *apic, uint32_t old_divisor)
{
    int64_t now, remaining, remaining_new;

    apic->lapic_timer.period =
            tmict_to_ns(apic, vm_apic_get_reg(apic, APIC_TMICT));
    limit_periodic_timer_frequency(apic);

    now = current_time_ns();
    remaining = apic->lapic_timer.target_expiration - now;
    if (remaining < 0) {
        remaining = 0;
    }

    remaining_new = muldivu64(remaining, (uint64_t) apic->divide_count, old_divisor);
    apic->lapic_timer.target_expiration = now + remaining_new;
}

static bool set_target_expiration(vm_lapic_t *apic, uint32_t count_reg)
{
    int64_t now = current_time_ns();
    int64_t deadline;

    apic->lapic_timer.period =
            tmict_to_ns(apic, vm_apic_get_reg(apic, APIC_TMICT));
    
    if (!apic->lapic_timer.period) {
        apic->lapic_timer.tscdeadline = 0;
        return false;
    }

    limit_periodic_timer_frequency(apic);
    deadline = apic->lapic_timer.period;

    if (apic_lvtt_period(apic) || apic_lvtt_oneshot(apic)) {
        if (unlikely(count_reg != APIC_TMICT)) {
            deadline = tmict_to_ns(apic, vm_apic_get_reg(apic, count_reg));
            if (unlikely(deadline <= 0)) {
                deadline = apic->lapic_timer.period;
            } else if (unlikely(deadline > apic->lapic_timer.period)) {
                apic_set_reg(apic, count_reg, 0);
                deadline = apic->lapic_timer.period;
            }
        }
    }

    /* KVM code for TSC deadline but we don't support. It's here
     * in case anyone wants to try in the future. */
#if 0
    apic->lapic_timer.tscdeadline = kvm_read_l1_tsc(apic->vcpu, tscl) +
        nsec_to_cycles(apic->vcpu, deadline);
#endif
    apic->lapic_timer.target_expiration = now + deadline;

    return true;
}

static void apic_cancel_timer(vm_lapic_t *apic)
{
    timer_stop(apic);
    apic->lapic_timer.pending = 0;
}

static void apic_update_lvtt(vm_lapic_t *apic)
{
    uint32_t timer_mode = vm_apic_get_reg(apic, APIC_LVTT) &
            apic->lapic_timer.timer_mode_mask;

    if (apic->lapic_timer.timer_mode != timer_mode) {
        if (apic_lvtt_tscdeadline(apic) != (timer_mode ==
                APIC_LVT_TIMER_TSCDEADLINE)) {
            apic_cancel_timer(apic);
            apic_set_reg(apic, APIC_TMICT, 0);
            apic->lapic_timer.period = 0;
            apic->lapic_timer.tscdeadline = 0;
        }
        apic->lapic_timer.timer_mode = timer_mode;
        limit_periodic_timer_frequency(apic);
    }
}

static void advance_periodic_target_expiration(vm_lapic_t *apic)
{
    /* KVM syncs the periodic and tsc deadline counters in this function
     * but I've ommitted for brevity's sake. */
    apic->lapic_timer.target_expiration += apic->lapic_timer.period;
}

static void start_period(vm_lapic_t *apic)
{
    if (!apic->lapic_timer.period)
        return;

    if (current_time_ns() > apic->lapic_timer.target_expiration) {
        apic_timer_expired(apic, false);

        if (apic_lvtt_oneshot(apic))
            return;

        advance_periodic_target_expiration(apic);
    }

    ZF_LOGE("Starting timer of period %ull", apic->lapic_timer.period);

    timer_oneshot_absolute(apic, apic->lapic_timer.target_expiration);
}

static void start_timer(vm_lapic_t *apic)
{
    if (!apic_lvtt_period(apic) && apic->lapic_timer.period) {
        return;
    }

    if (apic_lvtt_period(apic) || apic_lvtt_oneshot(apic)) {
        start_period(apic);
    } else {
        /* TSC deadline unimplemented */
    }
}

static void apic_restart_timer(vm_lapic_t *apic)
{
    if (!apic_lvtt_period(apic) && apic->lapic_timer.pending) {
        return;
    }

    start_timer(apic);
}

static void __start_apic_timer(vm_lapic_t *apic, uint32_t count_reg)
{
    apic->lapic_timer.pending = 0;

    if ((apic_lvtt_period(apic) || apic_lvtt_oneshot(apic))
        && !set_target_expiration(apic, count_reg))
        return;

    apic_restart_timer(apic);
}


static void start_apic_timer(vm_lapic_t *apic)
{
    __start_apic_timer(apic, APIC_TMICT);
}

static void apic_manage_nmi_watchdog(vm_lapic_t *apic, uint32_t lvt0_val)
{
    int nmi_wd_enabled = apic_lvt_nmi_mode(vm_apic_get_reg(apic, APIC_LVT0));

    if (apic_lvt_nmi_mode(lvt0_val)) {
        if (!nmi_wd_enabled) {
            apic_debug(4, "Receive NMI setting on APIC_LVT0 \n");
        }
    }
}

static int apic_reg_write(vm_vcpu_t *vcpu, uint32_t reg, uint32_t val)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    int ret = 0;

    switch (reg) {
    case APIC_ID:       /* Local APIC ID */
        vm_apic_set_id(apic, val >> 24);
        break;

    case APIC_TASKPRI:
        apic_set_tpr(vcpu, val & 0xff);
        break;

    case APIC_EOI:
        apic_set_eoi(vcpu);
        break;

    case APIC_LDR:
        vm_apic_set_ldr(apic, val & APIC_LDR_MASK);
        break;

    case APIC_DFR:
        apic_set_reg(apic, APIC_DFR, val | 0x0FFFFFFF);
        break;

    case APIC_SPIV: {
        uint32_t mask = 0x3ff;
        if (vm_apic_get_reg(apic, APIC_LVR) & APIC_LVR_DIRECTED_EOI) {
            mask |= APIC_SPIV_DIRECTED_EOI;
        }
        apic_set_spiv(apic, val & mask);
        if (!(val & APIC_SPIV_APIC_ENABLED)) {
            int i;
            uint32_t lvt_val;

            for (i = 0; i < APIC_LVT_NUM; i++) {
                lvt_val = vm_apic_get_reg(apic,
                                          APIC_LVTT + 0x10 * i);
                apic_set_reg(apic, APIC_LVTT + 0x10 * i,
                             lvt_val | APIC_LVT_MASKED);
            }
            apic->lapic_timer.pending = 0;
        }
        break;
    }
    case APIC_ICR:
        /* No delay here, so we always clear the pending bit */
        apic_set_reg(apic, APIC_ICR, val & ~(BIT(12)));
        apic_send_ipi(vcpu);
        break;

    case APIC_ICR2:
        val &= 0xff000000;
        apic_set_reg(apic, APIC_ICR2, val);
        break;

    case APIC_LVT0:
        apic_manage_nmi_watchdog(apic, val);
    case APIC_LVTTHMR:
    case APIC_LVTPC:
    case APIC_LVT1:
    case APIC_LVTERR:
        /* KVM TODO: Check vector */
        if (!vm_apic_sw_enabled(apic)) {
            val |= APIC_LVT_MASKED;
        }

        val &= apic_lvt_mask[(reg - APIC_LVTT) >> 4];
        apic_set_reg(apic, reg, val);

        break;

    case APIC_LVTT:
        /* Timer LVT */
        ZF_LOGE("APIC_LVTT");
        if (!vm_apic_sw_enabled(apic)) {
            val |= APIC_LVT_MASKED;
        }
        val &= (apic_lvt_mask[0] | apic->lapic_timer.timer_mode_mask);
        apic_set_reg(apic, APIC_LVTT, val);
        apic_update_lvtt(apic);
        break;

    case APIC_TMICT:
        /* Timer initial count */
        ZF_LOGE("APIC_TMICT");
        if (apic_lvtt_tscdeadline(apic)) {
            ZF_LOGE("TSC deadline should never be in use");
            break;
        }
        apic_cancel_timer(apic);
        apic_set_reg(apic, APIC_TMICT, val);
        start_apic_timer(apic);
        break;

    case APIC_TDCR:
        /* Timer divide config */
        ZF_LOGE("APIC_TDCR");
        uint32_t old_divisor = apic->divide_count;
        apic_set_reg(apic, APIC_TDCR, val);
        update_divide_count(apic);
        if (apic->divide_count != old_divisor &&
                apic->lapic_timer.period) {
            timer_stop(apic);
            update_target_expiration(apic, old_divisor);
            apic_restart_timer(apic);
        }
        break;
    
    case APIC_ESR:
        /**
         * Writing to the APIC ESR clears the ESR according to Pentium errata 3AP.
         * Linux does it on purpose (lmao) so we must support, I guess.
        */
        apic_set_reg(apic, APIC_ESR, 0);
        break;

    default:
        ret = 1;
        break;
    }
    if (ret) {
        apic_debug(2, "Local APIC Write to read-only register %x\n", reg);
    }
    return ret;
}

void vm_apic_mmio_write(vm_vcpu_t *vcpu, void *cookie, uint32_t offset,
                        int len, const uint32_t data)
{
    (void)cookie;

    /*
     * APIC register must be aligned on 128-bits boundary.
     * 32/64/128 bits registers must be accessed thru 32 bits.
     * Refer SDM 8.4.1
     */
    if (len != 4 || (offset & 0xf)) {
        apic_debug(1, "apic write: bad size=%d %x\n", len, offset);
        return;
    }

    /* too common printing */
    if (offset != APIC_EOI)
        apic_debug(6, "lapic mmio write at %s: offset 0x%x with length 0x%x, and value is "
                    "0x%x\n", __func__, offset, len, data);

    apic_reg_write(vcpu, offset & 0xff0, data);
}

static int apic_reg_read(vm_lapic_t *apic, uint32_t offset, int len,
                         void *data)
{
    unsigned char alignment = offset & 0xf;
    uint32_t result;
    /* this bitmask has a bit cleared for each reserved register */
    static const uint64_t rmask = 0x43ff01ffffffe70cULL;

    if ((alignment + len) > 4) {
        apic_debug(2, "APIC READ: alignment error %x %d\n",
                   offset, len);
        return 1;
    }

    if (offset > 0x3f0 || !(rmask & (1ULL << (offset >> 4)))) {
        apic_debug(2, "APIC_READ: read reserved register %x\n",
                   offset);
        return 1;
    }

    result = __apic_read(apic, offset & ~0xf);

    switch (len) {
    case 1:
    case 2:
    case 4:
        memcpy(data, (char *)&result + alignment, len);
        break;
    default:
        apic_debug(2, "Local APIC read with len = %x, "
                   "should be 1,2, or 4 instead\n", len);
        break;
    }
    return 0;
}

void vm_apic_mmio_read(vm_vcpu_t *vcpu, void *cookie, uint32_t offset,
                       int len, uint32_t *data)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    (void)cookie;

    apic_reg_read(apic, offset, len, data);

    apic_debug(6, "lapic mmio read on vcpu %d, reg 0x%x = 0x%x\n", vcpu->vcpu_id, offset, *data);

    return;
}

memory_fault_result_t apic_fault_callback(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                          void *cookie)
{
    uint32_t data;
    if (is_vcpu_read_fault(vcpu)) {
        vm_apic_mmio_read(vcpu, cookie, fault_addr - APIC_DEFAULT_PHYS_BASE, fault_length, &data);
        set_vcpu_fault_data(vcpu, data);
    } else {
        data = get_vcpu_fault_data(vcpu);
        vm_apic_mmio_write(vcpu, cookie, fault_addr - APIC_DEFAULT_PHYS_BASE, fault_length, data);
    }
    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}

void vm_free_lapic(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;

    if (!apic) {
        return;
    }

    if (apic->regs) {
        free(apic->regs);
    }

    free(apic);
}

void vm_lapic_set_base_msr(vm_vcpu_t *vcpu, uint32_t value)
{
    apic_debug(2, "IA32_APIC_BASE MSR set to %08x on vcpu %d\n", value, vcpu->vcpu_id);

    if (!(value & MSR_IA32_APICBASE_ENABLE)) {
        printf("Warning! Local apic has been disabled by MSR on vcpu %d. "
               "This will probably not work!\n", vcpu->vcpu_id);
    }

    vcpu->vcpu_arch.lapic->apic_base = value;
}

uint32_t vm_lapic_get_base_msr(vm_vcpu_t *vcpu)
{
    uint32_t value = vcpu->vcpu_arch.lapic->apic_base;

    if (vm_vcpu_is_bsp(vcpu)) {
        value |= MSR_IA32_APICBASE_BSP;
    } else {
        value &= ~MSR_IA32_APICBASE_BSP;
    }

    apic_debug(2, "Read from IA32_APIC_BASE MSR returns %08x on vcpu %d\n", value, vcpu->vcpu_id);

    return value;
}

void vm_lapic_reset(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic;
    int i;

    apic_debug(4, "%s\n", __func__);

    assert(vcpu);
    apic = vcpu->vcpu_arch.lapic;
    assert(apic != NULL);

    /* Stop the timer in case it's a reset to an active apic */

    vm_apic_set_id(apic, vcpu->vcpu_id); /* In agreement with ACPI code */
    apic_set_reg(apic, APIC_LVR, APIC_VERSION);

    for (i = 0; i < 8; i++) {
        apic_set_reg(apic, APIC_IRR + 0x10 * i, 0);
        apic_set_reg(apic, APIC_ISR + 0x10 * i, 0);
        apic_set_reg(apic, APIC_TMR + 0x10 * i, 0);
    }
    apic_set_reg(apic, APIC_ICR, 0);
    apic_set_reg(apic, APIC_ICR2, 0);
    apic_set_reg(apic, APIC_ESR, 0);

    /* Clear the LDR (unless in x2APIC mode) */
    apic_set_reg(apic, APIC_LDR, 0);
    apic_set_reg(apic, APIC_TASKPRI, 0);
    apic_set_reg(apic, APIC_TMICT, 0);
    apic_set_reg(apic, APIC_TMCCT, 0);
    apic_set_reg(apic, APIC_TDCR, 0);

    apic_set_reg(apic, APIC_DFR, 0xffffffffU);

    for (i = 0; i < APIC_LVT_NUM; i++) {
        apic_set_reg(apic, APIC_LVTT + 0x10 * i, APIC_LVT_MASKED);
    }

    apic_set_spiv(apic, 0xff);

    apic->irr_pending = 0;
    apic->isr_count = 0;
    apic->highest_isr_cache = -1;
    apic_update_ppr(vcpu);
    update_divide_count(apic);
    apic->lapic_timer.pending = 0;

    apic->arb_prio = 0;

    /* AMD does not implement the TSC deadline timer, so until we have AMD cpu
     * virtualization, just don't implement the TSC */
    apic->lapic_timer.timer_mode_mask = BIT(17);

    apic_debug(4, "%s: vcpu=%p, id=%d, base_msr="
               "0x%016x\n", __func__,
               vcpu, vm_apic_id(apic),
               apic->apic_base);

    if (vcpu->vcpu_id == BOOT_VCPU) {
        /* Bootstrap boot vcpu lapic in virtual wire mode */
        apic_set_reg(apic, APIC_LVT0,
                     SET_APIC_DELIVERY_MODE(0, APIC_MODE_EXTINT));
        apic_set_reg(apic, APIC_SPIV, APIC_SPIV_APIC_ENABLED);

        assert(vm_apic_sw_enabled(apic));
    } else {
        apic_set_reg(apic, APIC_SPIV, 0);
    }
}

int vm_create_lapic(vm_vcpu_t *vcpu, int enabled)
{
    vm_lapic_t *apic;

    assert(vcpu != NULL);
    apic_debug(2, "apic_init %d\n", vcpu->vcpu_id);

    apic = calloc(1, sizeof(*apic));
    if (!apic) {
        goto nomem;
    }

    vcpu->vcpu_arch.lapic = apic;

    apic->regs = calloc(1, PAGE_SIZE_4K);
    if (!apic->regs) {
        printf("calloc apic regs error for vcpu %x\n",
               vcpu->vcpu_id);
        goto nomem_free_apic;
    }

    if (enabled) {
        vm_lapic_set_base_msr(vcpu, APIC_DEFAULT_PHYS_BASE | MSR_IA32_APICBASE_ENABLE);
    } else {
        vm_lapic_set_base_msr(vcpu, APIC_DEFAULT_PHYS_BASE);
    }

    /* mainly init registers */
    vm_lapic_reset(vcpu);

    apic->vcpu = vcpu;

    return 0;
nomem_free_apic:
    free(apic);
nomem:
    return -1;
}

/* Return 1 if this vcpu should accept a PIC interrupt */
int vm_apic_accept_pic_intr(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    uint32_t lvt0 = vm_apic_get_reg(apic, APIC_LVT0);

    return ((lvt0 & APIC_LVT_MASKED) == 0 &&
            GET_APIC_DELIVERY_MODE(lvt0) == APIC_MODE_EXTINT &&
            vm_apic_sw_enabled(vcpu->vcpu_arch.lapic));
}

/* Service an interrupt */
int vm_apic_get_interrupt(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    int vector = vm_apic_has_interrupt(vcpu);

    if (vector == 1) {
        return pic_get_interrupt(vcpu->vm);
    } else if (vector == -1) {
        return -1;
    }

    apic_set_isr(vector, apic);
    apic_update_ppr(vcpu);
    apic_clear_irr(vector, apic);
    return vector;
}

/* Return which vector is next up for servicing */
int vm_apic_has_interrupt(vm_vcpu_t *vcpu)
{
    /* Early boot sequence apic might not be set up yet */
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    if (!apic) {
        return -1;
    }

    int highest_irr;

    if (vm_apic_accept_pic_intr(vcpu) && pic_has_interrupt(vcpu->vm)) {
        return 1;
    }

    highest_irr = apic_find_highest_irr(apic);
    if (highest_irr == -1) {
        return -1;
    }

    uint32_t ppr = vm_apic_get_reg(apic, APIC_PROCPRI);
    if ((highest_irr & 0xF0) <= vm_apic_get_reg(apic, APIC_PROCPRI)) {
        return -1;
    }

    return highest_irr;
}

int vm_apic_local_deliver(vm_vcpu_t *vcpu, int lvt_type)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    uint32_t reg = vm_apic_get_reg(apic, lvt_type);
    int vector, mode, trig_mode;

    if (!(reg & APIC_LVT_MASKED)) {
        vector = reg & APIC_VECTOR_MASK;
        mode = reg & APIC_MODE_MASK;
        trig_mode = reg & APIC_LVT_LEVEL_TRIGGER;
        return __apic_accept_irq(vcpu, mode, vector, 1, trig_mode, NULL);
    }
    return 0;
}

int vm_inject_timer_irq(vm_vcpu_t *vcpu)
{
    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;
    apic_timer_expired(apic, true);
    apic->lapic_timer.pending = 0;

    /* if period timer, slap that mf back alive */
    if (apic_lvtt_period(apic)) {
        advance_periodic_target_expiration(apic);
        timer_oneshot_absolute(apic, apic->lapic_timer.target_expiration);
    } else {
        timer_stop(apic);
    }

    return 0;
}

void vm_apic_set_timer_and_update(vm_lapic_t *apic, struct timer_functions *timer_emul)
{
    apic->lapic_timer.timer_emul = timer_emul;
    tsc_frequency = timer_tsc_freq(apic);
}

void vm_irq_set_msi_data(int irq, pci_msi_data_t *msi_data)
{
    memcpy(&irq_info[irq].cookie->msi_cookie.data, msi_data, sizeof(*msi_data));
}

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    /* if legacy irq send to PIC */
    if (irq < I8259_NR_IRQS) {
        return i8259_inject_irq(vcpu, irq);
    }

    int vector, delivery_mode, level, trig_mode;

    if (irq_info[irq].cookie->is_msi) {
        /* If MSI we need to patch in the values the guest programmed originally */
        pci_msi_data_t msi_data = irq_info[irq].cookie->msi_cookie.data;

        /* The MSI data register and APIC ISR are practically the same.
         * Even if they aren't MSIs are always fixed and edge triggered.*/
        vector = msi_data.value & APIC_VECTOR_MASK;
        delivery_mode = msi_data.value & APIC_DM_FIXED_MASK;
        trig_mode = msi_data.value & APIC_INT_LEVELTRIG;
        level = msi_data.value & APIC_INT_ASSERT; /* Ignored if edge trig */
    } else {
        /* Not sure what this should be programmed, just leaving it as fixed
         * edge triggered for now. */
        vector = irq;
        delivery_mode = APIC_DM_FIXED;
        trig_mode = APIC_INT_EDGETRIG;
        level = 0; /* Ignored if edge trig */
    }

    vm_lapic_t *apic = vcpu->vcpu_arch.lapic;

    int ret = __apic_accept_irq(vcpu, delivery_mode, vector, level, trig_mode, NULL);
    return (ret) ? 0 : 1;
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t fn, void *cookie)
{
    /* if legacy irq send to PIC to deal with*/
    if (irq < I8259_NR_IRQS) {
        return i8259_register_irq(vcpu, irq, fn, cookie);
    }
    irq_info_t *info = &irq_info[irq];
    info->callback = fn;
    info->cookie = (x86_irq_cookie_t *) cookie;
    return 0;
}
