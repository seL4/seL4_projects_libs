/*
 * Copyright 2024, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DORNERWORKS_BSD)
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
#include <sel4vmmplatsupport/plat/pl011_vuart.h>
#include <sel4vmmplatsupport/plat/devices.h>
#include <ringbuffer/ringbuffer.h>

#include <libfdt.h>
#include <fdtgen.h>

/*** VUART FDT MACRO ***/
#define VUART_FDT_OP(op)                                       \
    do {                                                       \
        int err = (op);                                        \
        ZF_LOGF_IF(err < 0, "Vuart: FDT operation failed");    \
    } while(0);

struct vuart_irqs {
    uint32_t type;
    uint32_t num;
    uint32_t flags;
} PACKED;
typedef struct vuart_irqs vuart_irqs_t;

struct vuart_dev {
  uint64_t address;
  uint64_t size;
  uint32_t current_speed;
  uint32_t port;
  vuart_irqs_t irqs;
};

#ifndef PL011_VUART_PLAT_SUPPORT
#warning "Compiling Pl011 Vuart for unsupported platform"
#define PL011_VUART_ADDR        0xA0A0A0A0
#define PL011_VUART_SIZE        0x1000
#define PL011_VUART_SPEED       115200
#define PL011_VUART_PORT        0x1
#define PL011_VUART_IRQ_TYPE     0x4
#define PL011_VUART_IRQ_NUM      0xfff
#define PL011_VUART_IRQ_FLAGS    0x4
#endif

struct vuart_dev vuart_info = {
    .address = PL011_VUART_ADDR,
    .size = PL011_VUART_SIZE,
    .current_speed = PL011_VUART_SPEED,
    .port = PL011_VUART_PORT,
    .irqs = {
        .type = PL011_VUART_IRQ_TYPE,
        .num = PL011_VUART_IRQ_NUM,
        .flags = PL011_VUART_IRQ_FLAGS,
    },
};

/* UART MACROS*/
#define PL011_VUART_PADDR     PL011_VUART_ADDR
#define VUART_BUFLEN            32
#define VUART_SIZE              PL011_VUART_SIZE
#define VUART_COMPAT            "arm,sbsa-uart" /*Amba Pl011 SBSA compat string*/
#define PL011_VCONSOLE_IRQ    (vuart_info.irqs.num + 32)

/* UART REGISTER OFFSETS*/
#define DATA_REG                0x00
#define STAT_ERR_CLR_REG        0x04
#define FLAG_REG                0x18
#define LOW_PWR_CTR_REG         0x020
#define INT_BAUD_REG            0x024
#define FRACT_BAUD_REG          0x028
#define LINE_CTRL_REG           0x02C
#define CONTROL_REG             0x030
#define IRQ_FIFO_SELECT_REG     0x034
#define IRQ_MASK_SET_REG        0x038
#define RAW_IRQ_STAT_REG        0x03C
#define MASK_IRQ_STAT_REG       0x040
#define IRQ_CLR_REG             0x044
#define DMA_CTRL_REG            0x048
#define PERIPH_ID_START         0xFE0
#define PERIPH_ID_END           0xFEC
#define PERIPH_CELL_ID_START    0xFF0
#define PERIPH_CELL_ID_END      0xFFC
#define PERIPHIDN(N)            (0x0FE0 + (4 * N))
#define PERIPHCELLIDN(N)        (0x0FF0 + (4 * N))

#define CR_ENABLE_BIT           BIT(0)
#define LCR_FEN_BIT             BIT(4)

#define RX_IRS                  BIT(4)
#define RX_FE                   BIT(4)
#define TX_FE                   BIT(7)

/* UART RESET VALUES */
#define CR_RESET_VALUE          0x301 // bits 9 and 8 are 1
#define FR_RESET_VALUE          0x90 //  bits 7 and 4 are 1
#define IFLS_RESET_VALUE        0x12
#define RIS_RESET_VALUE         0xf
#define MIS_RESET_VALUE         0xf


/** VUART structs*/
const struct device dev_pl011_vuart = {
    .name = "pl011_vuart",
    .pstart = PL011_VUART_PADDR,
    .size = VUART_SIZE,
    .handle_device_fault = NULL,
    .priv = NULL
};
/*Shown in Table 3-1 of the PrimeCell UART Technical Reference Manual (PL011)*/
struct common_uart_regs {
    uint32_t dr;                /* 0x00: data */
    uint32_t sr_ecr;            /* 0x04: status register / error clear */
    uint32_t res0[3];           /* 0x008 - 0x014: reserved */
    uint32_t fr;                /* 0x018: flag */
    uint32_t res1;              /* 0x01c: reserved*/
    uint32_t lpr;               /* 0x020: lower power counter register*/
    uint32_t ibrd;              /* 0x024: integer baud rate */
    uint32_t fbrd;              /* 0x028: fractional baud rate*/
    uint32_t lcr_h;             /* 0x02c: line control */
    uint32_t cr;                /* 0x030: control */
    uint32_t ifls;              /* 0x034: interrupt fifo select */
    uint32_t imsc;              /* 0x038: interrupt mask set/clear */
    uint32_t ris;               /* 0x03C: raw interrupt status */
    uint32_t mis;               /* 0x040: masked interrupt status */
    uint32_t icr;               /* 0x044: interrupt clear*/
    uint32_t dmacr;             /* 0x048: dma control */
    uint32_t res2[12];          /* 0x04C-0x07C: reserved */
    uint32_t res3[3];           /* 0x080 - 0x08C: reserved for test */
    uint32_t res4[975];         /* 0x90 - 0xFCC: reserved */
    uint32_t res5[3];           /* 0xFD0 - 0xFDC: reserved for future ids*/
    uint32_t periphidn[4];      /* 0xFE0 - 0xFEC: periph id n regs*/
    uint32_t pcellidn[4];       /* 0xFF0 - 0xFFC: pcellidn */
};

/** Data struct that ends up holding our chars before they get flushed out **/
struct vuart_priv {
    void *regs;
    char buffer[VUART_BUFLEN];
    int virq;
    int buf_pos;
    int int_pending;
    vm_t *vm;
    print_func_t callback;
};
typedef volatile struct common_uart_regs common_uart_regs_t;
static struct vuart_priv *vuart_data;
static ringbuffer_t *send_ring;
static ringbuffer_t *recv_ring;

static inline void * pl011_vuart_priv_get_regs(struct device *d)
{
    return ((struct vuart_priv *)d->priv)->regs;
}

int fdt_generate_vuart_node(vm_t* vm, void* fdt, int gic_phandle)
{
    int root_offset = fdt_path_offset(fdt, "/");
    int address_cells = fdt_address_cells(fdt, root_offset);
    int size_cells = fdt_size_cells(fdt, root_offset);

    int vuart_node = fdt_add_subnode(fdt, root_offset, "pl011_vuart@a000a000");
    if(vuart_node < 0){
        return vuart_node;
    }

    vuart_info.irqs.num  = cpu_to_fdt32(vuart_info.irqs.num);
    vuart_info.irqs.flags = cpu_to_fdt32(vuart_info.irqs.flags);

    VUART_FDT_OP(fdt_appendprop_uint(fdt, vuart_node, "reg", vuart_info.address, address_cells));
    VUART_FDT_OP(fdt_appendprop_uint(fdt, vuart_node, "reg", vuart_info.size, size_cells));
    VUART_FDT_OP(fdt_appendprop_string(fdt, vuart_node, "compatible", VUART_COMPAT));
    VUART_FDT_OP(fdt_appendprop_string(fdt, vuart_node, "status", "okay"));
    VUART_FDT_OP(fdt_appendprop_u32(fdt, vuart_node, "current-speed", vuart_info.current_speed));
    VUART_FDT_OP(fdt_appendprop_u32(fdt, vuart_node, "port-number", vuart_info.port));
    VUART_FDT_OP(fdt_appendprop_u32(fdt, vuart_node, "interrupt-parent", gic_phandle));
    VUART_FDT_OP(fdt_appendprop(fdt, vuart_node, "interrupts", &vuart_info.irqs, sizeof(vuart_info.irqs)));

    return 0;
}

static void reset_common_vuart(struct device *dev)
{
    common_uart_regs_t *uart_regs = (common_uart_regs_t *) pl011_vuart_priv_get_regs(dev);
    assert(uart_regs);
    //only set specific non 0 bits here since calloc should clear everything initially
    uart_regs->cr = CR_RESET_VALUE;
    uart_regs->fr = FR_RESET_VALUE;
    uart_regs->ifls = IFLS_RESET_VALUE;
    uart_regs->ris = RIS_RESET_VALUE;
    uart_regs->mis = MIS_RESET_VALUE;
}

static void pl011_vuart_ack(vm_vcpu_t *vcpu, int irq, void *cookie)
{
    struct vuart_priv *vuart_data = cookie;
    common_uart_regs_t *uart_regs = (common_uart_regs_t *)vuart_data->regs;
    if (uart_regs->ris & uart_regs->mis) {
        /* Another IRQ occured */
        vm_inject_irq(vuart_data->vm->vcpus[BOOT_VCPU], vuart_data->virq);
    } else {
        vuart_data->int_pending = 0;
    }
}

static void pl011_vuart_inject_irq(struct vuart_priv *vuart)
{
    if (vuart->int_pending == 0) {
        vuart->int_pending = 1;
        vm_inject_irq(vuart->vm->vcpus[BOOT_VCPU], vuart->virq);
    }
}

void pl011_vuart_handle_irq(int c)
{
    common_uart_regs_t *uart_regs = (common_uart_regs_t *)vuart_data->regs;

    rb_transmit_byte(send_ring, (unsigned char)c);

    if (!rb_has_data(recv_ring)) {
        uart_regs->ris |= RX_IRS;
        pl011_vuart_inject_irq(vuart_data);
    }
}

static void flush_pl011_vconsole_device(struct device *d)
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

static void pl011_vuart_putchar(struct device *d, char c)
{
    struct vuart_priv *vuart_data;
    assert(d->priv);
    common_uart_regs_t *uart_regs = (common_uart_regs_t *)pl011_vuart_priv_get_regs(d);
    vuart_data = (struct vuart_priv *)d->priv;

    assert(vuart_data->buf_pos < VUART_BUFLEN);
    vuart_data->buffer[vuart_data->buf_pos++] = c;

    /* We flush after every character is sent instead of only at newlines. This is so typing in characters on the
     * console doesn't look weird. This can be slow when displaying a lot of information quickly.
     *
     * We could probably implement some SW timeout that flushes every so often if there is data available.
     */
    flush_pl011_vconsole_device(d);

    pl011_vuart_inject_irq(vuart_data);
}

static memory_fault_result_t handle_pl011_vuart_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                                void *cookie)
{
    uint32_t *reg;
    int offset;
    uint32_t mask;
    struct device *dev;
    dev = (struct device *)cookie;
    uint32_t v;
    UNUSED int data;
    common_uart_regs_t *uart_regs;

    uart_regs = (common_uart_regs_t *)pl011_vuart_priv_get_regs(dev);

    /* Gather fault information */
    offset = fault_addr - dev->pstart;
    reg = (uint32_t *)(pl011_vuart_priv_get_regs(dev) + offset - (offset % 4));
    mask = get_vcpu_fault_data_mask(vcpu);

    /* Handle the fault */
    if (offset < 0 || VUART_SIZE <= offset) {
        /* Out of range, treat as SBZ */
        set_vcpu_fault_data(vcpu, 0);
        ZF_LOGE("Unhandled offset for common uart device %p\n", offset);
        return FAULT_IGNORE;
    }
    else if (is_vcpu_read_fault(vcpu)) {
        switch (offset) {
            case FLAG_REG:
                data = 0;
                if (rb_has_data(recv_ring)) {
                    data |= RX_FE;
                }
                data |= TX_FE;
                set_vcpu_fault_data(vcpu, data);
                break;
            case DATA_REG:
                if (!rb_has_data(recv_ring)) {
                    data = rb_receive_byte(recv_ring);
                    set_vcpu_fault_data(vcpu, data);
                }
                if (rb_has_data(recv_ring)) {
                    uart_regs->ris &= ~RX_IRS;
                }
                break;

            case IRQ_CLR_REG:
                return FAULT_IGNORE;
            default:
                /* Blindly read out data */
                set_vcpu_fault_data(vcpu, *reg);
        }
        advance_vcpu_fault(vcpu);

    } else {
        uint32_t fault_data = get_vcpu_fault_data(vcpu);
        fault_data &= mask;
        switch (offset) {
            case DATA_REG: // we are writing data to be sent out
                pl011_vuart_putchar(dev, fault_data & 0xff);
                break;
            case STAT_ERR_CLR_REG: // any write clears error bits data value does not matter
                /* Only clear set bits */
                uart_regs->sr_ecr = 0;
                break;
            case IRQ_CLR_REG:
                fault_data &= 0x7ff;
                uart_regs ->ris &= ~fault_data;
                break;
            case IRQ_MASK_SET_REG:
                v = uart_regs->imsc & ~mask;
                v |= fault_data;
                uart_regs->imsc |= v;
                uart_regs->mis |= v;
                break;
            case CONTROL_REG:
            case LOW_PWR_CTR_REG:
            case INT_BAUD_REG:
            case FRACT_BAUD_REG:
            case LINE_CTRL_REG:
            case DMA_CTRL_REG:
            case IRQ_FIFO_SELECT_REG:
                /* Blindly write to the device */
                v = *reg & ~mask;
                v |= fault_data;
                *reg = v;
                break;
            /* read only regs */
            case FLAG_REG:
            case RAW_IRQ_STAT_REG:
            case MASK_IRQ_STAT_REG:
            case PERIPH_ID_START ... PERIPH_ID_END:
            case PERIPH_CELL_ID_START ... PERIPH_CELL_ID_END:
            default:
                return FAULT_IGNORE;
            }
            advance_vcpu_fault(vcpu);
    }
    return FAULT_HANDLED;
}

int vm_install_pl011_vconsole(vm_t *vm, print_func_t func)
{
    static int once = 0;

    ZF_LOGF_IF(once, "Only install vconsole once\n");

    struct device* d;
    int err;

    d = (struct device *)calloc(1, sizeof(struct device));

    ZF_LOGF_IF(NULL == d, "Failed to allocate pl011 device struct")

    *d = dev_pl011_vuart;

    /* Initialise the virtual device */
    vuart_data = (struct vuart_priv *) calloc(1, sizeof(struct vuart_priv));
    ZF_LOGF_IF(NULL == vuart_data, "Failed to malloc vconsole device\n");

    vuart_data->vm = vm;
    vuart_data->int_pending = 0;
    vuart_data->callback = func;

    vuart_data->regs = calloc(1, VUART_SIZE);

    ZF_LOGF_IF(NULL == vuart_data->regs, "Failed to allocate registers for vconsole device");

    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d->pstart, d->size,
                                                                &handle_pl011_vuart_fault, (void *)d);

    ZF_LOGF_IF(NULL == reservation, "Failed to reserve memory for pl011 vuart");

    d->priv = vuart_data;

    reset_common_vuart(d);

    /* Initialise virtual IRQ */
    vuart_data->virq = PL011_VCONSOLE_IRQ;
    err = vm_register_irq(vm->vcpus[BOOT_VCPU], PL011_VCONSOLE_IRQ, &pl011_vuart_ack, vuart_data);
    ZF_LOGF_IF(err, "Failed to initialize vconsole virq\n");

    /* Initialize input ring buffer */
    void *ring_buf_base = (char *)malloc(sizeof(char) * VUART_BUFLEN);
    ZF_LOGF_IF(NULL == ring_buf_base, "Failed to initialize input ring buffer\n");

    send_ring = rb_new(ring_buf_base, VUART_BUFLEN);
    recv_ring = rb_new(ring_buf_base, VUART_BUFLEN);

    once = 1;

    ZF_LOGE("installed the console");
    return 0;
}