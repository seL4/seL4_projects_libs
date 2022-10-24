/*
 * Copyright 2019, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/boot.h>

#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/plat/device_map.h>
#include <sel4vmmplatsupport/plat/vuart.h>
#include <sel4vmmplatsupport/plat/devices.h>

#include <ringbuffer/ringbuffer.h>

#define VUART_BUFLEN 256

#define CR         0x00 /* Control Register */
#define MR         0x04 /* Mode Register */
#define IER        0x08 /* Interrupt Enable Register */
#define IDR        0x0C /* Interrupt Disable Register */
#define IMR        0x10 /* Interrupt Mask Register */
#define ISR        0x14 /* Channel Interrupt Status Register */
#define BAUDGEN    0x18 /* Baud Rate Generator Register */
#define RXTOUT     0x1C /* Receiver Timeout Register */
#define RXWM       0x20 /* Receiver FIFO Trigger Level Register */
#define MODEMCR    0x24 /* Modem Control Register */
#define MODEMSR    0x28 /* Modem Status Register */
#define SR         0x2C /* Channel Status Register */
#define FIFO       0x30 /* Transmit and Receive FIFO */
#define BAUDDIV    0x34 /* Baud Rate Divider Register */
#define FLOWDEL    0x38 /* Flow Control Delay Register */
#define PAD1       0x3C
#define PAD2       0x40
#define TXWM       0x44 /* Transmitter FIFO Trigger Level Register */
#define UART_SIZE  0x48

struct zynq_uart_regs {
    uint32_t cr;            /* 0x00 Control Register */
    uint32_t mr;            /* 0x04 Mode Register */
    uint32_t ier;           /* 0x08 Interrupt Enable Register */
    uint32_t idr;           /* 0x0C Interrupt Disable Register */
    uint32_t imr;           /* 0x10 Interrupt Mask Register */
    uint32_t isr;           /* 0x14 Channel Interrupt Status Register */
    uint32_t baudgen;       /* 0x18 Baud Rate Generator Register */
    uint32_t rxtout;        /* 0x1C Receiver Timeout Register */
    uint32_t rxwm;          /* 0x20 Receiver FIFO Trigger Level Register */
    uint32_t modemcr;       /* 0x24 Modem Control Register */
    uint32_t modemsr;       /* 0x28 Modem Status Register */
    uint32_t sr;            /* 0x2C Channel Status Register */
    uint32_t fifo;          /* 0x30 Transmit and Receive FIFO */
    uint32_t bauddiv;       /* 0x34 Baud Rate Divider Register */
    uint32_t flowdel;       /* 0x38 Flow Control Delay Register */
    uint32_t pad[2];
    uint32_t txwm;          /* 0x44 Transmitter FIFO Trigger Level Register */
};
typedef volatile struct zynq_uart_regs zynq_uart_regs_t;

#define UART_SR_RTRIG           BIT( 0)
#define UART_SR_REMPTY          BIT( 1)
#define UART_SR_RFUL            BIT( 2)
#define UART_SR_TEMPTY          BIT( 3)
#define UART_SR_TFUL            BIT( 4)
#define UART_SR_RACTIVE         BIT(10)
#define UART_SR_TACTIVE         BIT(11)
#define UART_SR_FDELT           BIT(12)
#define UART_SR_TTRIG           BIT(13)
#define UART_SR_TNFUL           BIT(14)

#define UART_ISR_RTRIG          BIT( 0)
#define UART_ISR_REMPTY         BIT( 1)
#define UART_ISR_RFUL           BIT( 2)
#define UART_ISR_TEMPTY         BIT( 3)
#define UART_ISR_TFUL           BIT( 4)
#define UART_ISR_ROVR           BIT( 5)
#define UART_ISR_FRAME          BIT( 6)
#define UART_ISR_PARE           BIT( 7)
#define UART_ISR_TIMEOUT        BIT( 8)
#define UART_ISR_DMSI           BIT( 9)
#define UART_ISR_TTRIG          BIT(10)
#define UART_ISR_TNFUL          BIT(11)
#define UART_ISR_TOVR           BIT(12)
#define UART_ISR_MASK           (BIT(13)-1)

#define UART_CR_SELF_CLEARING_BITS  (0x43)

#define COLOR_BUF_SZ      6
#define NAME_BUF_SZ       64

struct vuart_priv {
    void *regs;
    char buffer[VUART_BUFLEN];
    int virq;
    int buf_pos;
    int int_pending;
    vm_t *vm;
    print_func_t callback;
};

static struct vuart_priv *vuart_data;
static ringbuffer_t *send_ring;
static ringbuffer_t *recv_ring;

static inline void *vuart_priv_get_regs(struct device *d)
{
    return ((struct vuart_priv *)d->priv)->regs;
}

static void vuart_data_reset(struct device *d)
{
    void *uart_regs = vuart_priv_get_regs(d);

    /* Default UART registers as defined in the ZUS+ TRM. Since
     * we are emulating the device, we want the VM to see the
     * registers with the values it would expect on reset.
     */
    const uint32_t reset_data[] = { 0x00000128,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000208,
                                    0x0000028B,
                                    0x00000000,
                                    0x00000020,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x0000000F,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000020
                                  };
    memcpy(uart_regs, reset_data, sizeof(reset_data));
}

/* Called by the VM to ACK a virtual IRQ */
static void vuart_ack(vm_vcpu_t *vcpu, int irq, void *cookie)
{
    struct vuart_priv *vuart_data = cookie;
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_data->regs;
    if (uart_regs->isr & uart_regs->imr) {
        /* Another IRQ occured */
        vm_inject_irq(vuart_data->vm->vcpus[BOOT_VCPU], vuart_data->virq);
    } else {
        vuart_data->int_pending = 0;
    }
}

static void vuart_inject_irq(struct vuart_priv *vuart)
{
    if (vuart->int_pending == 0) {
        vuart->int_pending = 1;
        vm_inject_irq(vuart->vm->vcpus[BOOT_VCPU], vuart->virq);
    }
}

void vuart_handle_irq(int c)
{
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_data->regs;

    rb_transmit_byte(send_ring, (unsigned char)c);

    if (!rb_has_data(recv_ring)) {
        uart_regs->isr |= UART_ISR_RTRIG;
        vuart_inject_irq(vuart_data);
    }
}

static void flush_vconsole_device(struct device *d)
{
    struct vuart_priv *vuart_data;
    char *buf;

    vuart_data = (struct vuart_priv *)d->priv;
    assert(d->priv);
    buf = vuart_data->buffer;

    for (int i = 0; i < vuart_data->buf_pos; i++) {
        vuart_data->callback(buf[i]);
    }

    vuart_data->buf_pos = 0;
}

static void vuart_putchar(struct device *d, char c)
{
    struct vuart_priv *vuart_data;
    assert(d->priv);
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_priv_get_regs(d);
    vuart_data = (struct vuart_priv *)d->priv;

    assert(vuart_data->buf_pos < VUART_BUFLEN);
    vuart_data->buffer[vuart_data->buf_pos++] = c;

    /* We flush after every character is sent instead of only at newlines. This is so typing in characters on the
     * console doesn't look weird. This can be slow when displaying a lot of information quickly.
     *
     * We could probably implement some SW timeout that flushes every so often if there is data available.
     */
    flush_vconsole_device(d);

    vuart_inject_irq(vuart_data);
}

static memory_fault_result_t handle_vuart_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                                void *cookie)
{
    uint32_t *reg;
    int offset;
    uint32_t mask;
    struct device *dev;
    dev = (struct device *)cookie;
    UNUSED uint32_t v;
    UNUSED int data;
    zynq_uart_regs_t *uart_regs;

    uart_regs = (zynq_uart_regs_t *)vuart_priv_get_regs(dev);

    /* Gather fault information */
    offset = fault_addr - dev->pstart;
    reg = (uint32_t *)(vuart_priv_get_regs(dev) + offset - (offset % 4));
    mask = get_vcpu_fault_data_mask(vcpu);

    /* Handle the fault */
    if (offset < 0 || UART_SIZE <= offset) {
        /* Out of range, treat as SBZ */
        set_vcpu_fault_data(vcpu, 0);
        return FAULT_IGNORE;

    } else if (is_vcpu_read_fault(vcpu)) {
        switch (offset) {
        case SR:
            data = 0;
            if (rb_has_data(recv_ring)) {
                data |= UART_SR_REMPTY;
            }
            data |= UART_SR_TEMPTY;
            set_vcpu_fault_data(vcpu, data);
            break;
        case ISR:
            set_vcpu_fault_data(vcpu, uart_regs->isr);
            break;
        case FIFO:
            if (!rb_has_data(recv_ring)) {
                data = rb_receive_byte(recv_ring);
                set_vcpu_fault_data(vcpu, data);
            }
            if (rb_has_data(recv_ring)) {
                uart_regs->isr &= ~UART_ISR_RTRIG;
            }
            break;
        default:
            /* Blindly read out data */
            set_vcpu_fault_data(vcpu, *reg);
        }
        advance_vcpu_fault(vcpu);

    } else { /* if(fault_is_write(fault))*/
        switch (offset) {
        case IER:
            /* Set bits get set in Interrupt Mask */
            v = (get_vcpu_fault_data(vcpu) & mask);
            uart_regs->imr |= v;
            break;
        case IDR:
            /* Set bits get cleared in Interrupt Mask */
            v = ~(get_vcpu_fault_data(vcpu) & mask);
            uart_regs->imr &= v;
            break;
        case ISR:
            /* Only clear set bits */
            v = uart_regs->isr & ~mask;
            v &= ~(get_vcpu_fault_data(vcpu)& mask);
            v |= UART_ISR_TEMPTY;
            uart_regs->isr = v;
            break;
        case BAUDGEN:
        case RXTOUT:
        case RXWM:
        case MODEMCR:
        case MODEMSR:
        case BAUDDIV:
        case FLOWDEL:
        case MR:
        case TXWM:
            /* Blindly write to the device */
            v = *reg & ~mask;
            v |= get_vcpu_fault_data(vcpu) & mask;
            *reg = v;
            break;
        case FIFO:
            vuart_putchar(dev, get_vcpu_fault_data(vcpu));
            break;
        case CR:
            v = *reg & ~mask;
            v |= get_vcpu_fault_data(vcpu) & mask;
            /* Always make sure self clearing bits are cleared
             * since we don't actually let the VM control the UART
             */
            v &= ~(UART_CR_SELF_CLEARING_BITS);
            *reg = v;
            break;
        default:
            return FAULT_IGNORE;
        }
        advance_vcpu_fault(vcpu);
    }
    return FAULT_HANDLED;
}

const struct device dev_uart0 = {
    .name = "uart0",
    .pstart = UART0_PADDR,
    .size = 0x1000,
    .handle_device_fault = NULL,
    .priv = NULL
};

const struct device dev_uart1 = {
    .name = "uart1",
    .pstart = UART1_PADDR,
    .size = 0x1000,
    .handle_device_fault = NULL,
    .priv = NULL
};

int vm_install_vconsole(vm_t *vm, print_func_t func)
{
    static int once = 0;

    ZF_LOGF_IF(once, "Only install vconsole once\n");

    struct device *d;
    int err;

    d = (struct device *)calloc(1, sizeof(struct device));
    if (!d) {
        return -1;
    }

    *d = dev_vconsole;

    /* Initialise the virtual device */
    vuart_data = calloc(1, sizeof(struct vuart_priv));
    ZF_LOGF_IF(NULL == vuart_data, "Failed to malloc vconsole device\n");

    vuart_data->vm = vm;
    vuart_data->int_pending = 0;
    vuart_data->callback = func;

    vuart_data->regs = calloc(1, UART_SIZE);
    if (vuart_data->regs == NULL) {
        assert(vuart_data->regs);
        return -1;
    }

    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d->pstart, d->size,
                                                                handle_vuart_fault, (void *)d);
    if (!reservation) {
        return -1;
    }

    d->priv = vuart_data;

    vuart_data_reset(d);

    /* Initialise virtual IRQ */
    vuart_data->virq = VCONSOLE_IRQ;
    err = vm_register_irq(vm->vcpus[BOOT_VCPU], VCONSOLE_IRQ, &vuart_ack, vuart_data);
    ZF_LOGF_IF(err, "Failed to initialize vconsole virq\n");

    /* Initialize input ring buffer */
    void *ring_buf_base = (char *)malloc(sizeof(char) * VUART_BUFLEN);
    ZF_LOGF_IF(NULL == ring_buf_base, "Failed to initialize input ring buffer\n");

    send_ring = rb_new(ring_buf_base, VUART_BUFLEN);
    recv_ring = rb_new(ring_buf_base, VUART_BUFLEN);

    once = 1;

    return 0;
}

int vm_uninstall_vconsole(vm_t *vm)
{
    return 0;
}
