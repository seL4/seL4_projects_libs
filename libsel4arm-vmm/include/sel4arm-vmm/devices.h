/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */
#pragma once

#include <stdint.h>

#include <autoconf.h>

#include <sel4arm-vmm/fault.h>
#include <platsupport/gpio.h>
#include <platsupport/plat/clock.h>

#define NO_STREAM_ID 0

typedef struct vm vm_t;

/**
 * Logical device identifier
 */
enum devid {
    DEV_RAM,
    DEV_VGIC_DIST,
    DEV_VGIC_CPU,
    DEV_VGIC_VCPU,
    DEV_IRQ_COMBINER,
    DEV_PWM_TIMER,
    DEV_WDT_TIMER,
    DEV_MCT_TIMER,
    DEV_UART0,
    DEV_UART1,
    DEV_UART2,
    DEV_UART3,
    /* Number of devices */
    DEV_NDEVICES,
    /* Special values */
    DEV_CUSTOM,
    /* Aliases */
    DEV_CONSOLE = DEV_UART3
};


/**
 * Device description
 */
struct device {
/// Logical identifier for internal use
    enum devid devid;
/// A string representation of the device. Useful for debugging
    const char *name;
/// The physical address of the device */
    seL4_Word pstart;
/// Device mapping size */
    seL4_Word size;
#ifdef CONFIG_ARM_SMMU_V2
/// Stream ID */
    uint16_t sid;
#endif
/// Fault handler */
    int (*handle_page_fault)(struct device *d, vm_t *vm, fault_t *fault);
/// device emulation private data */
    void *priv;
};

/**
 * GPIO Access control device
 */
struct gpio_device;

/**
 * Clock Access control device
 */
struct clock_device;

/**
 * Install a passthrough device into a VM
 * @param[in] vm     A handle to the VM that the device should be install to
 * @param[in] device A description of the device
 * @return           0 on success
 */
int vm_install_passthrough_device(vm_t *vm, const struct device *device);

/**
 * Install a device backed by ram into a VM
 * @param[in] vm     A handle to the VM that the device should be install to
 * @param[in] device A description of the device
 * @return           0 on success
 */
int vm_install_ram_only_device(vm_t *vm, const struct device *device);

/**
 * Install a passthrough device into a VM, but trap and print all access
 * @param[in] vm     A handle to the VM that the device should be install to
 * @param[in] device A description of the device
 * @return           0 on success
 */
int vm_install_listening_device(vm_t *vm, const struct device *device);

/**
 * Install a device into the VM and call its associated fault handler on access
 * @param[in] vm     A handle to the VM that the device should be install to
 * @param[in] device A description of the device
 * @return           0 on success
 */
int vm_install_emulated_device(vm_t *vm, const struct device *device);


/**
 * Add a generic device to the VM without performind any initialisation of the device
 * When the VM receives a fault with an address that is in the range of this device,
 * it will call the assigned fault handler.
 * @param[in] vm     A handle to the VM that the device should be install to
 * @param[in] device A description of the device
 * @return           0 on success
 */
int vm_add_device(vm_t *vm, const struct device *d);

/**
 * Map a given frame cap into a VM's IPA.
 * @param[in] vm         A handle to the VM that the frame should be mapped into
 * @param[in] cap        Cap of the frame to be mapped
 * @param[in] ipa        The IPA that the frame is to be mapped at
 * @param[in] size_bits  The frame size in bits
 * @param[in] cached     Is it mapped with caching enabled?
 * @param[in] vm_rights  Mapping rights
 * @return               0 on success
 */
int vm_map_frame(vm_t *vm, seL4_CPtr cap, uintptr_t ipa, size_t size_bits, int cached, seL4_CapRights_t vm_rights);

/**** GPIO ****/
enum vacdev_default {
    VACDEV_DEFAULT_ALLOW,
    VACDEV_DEFAULT_DENY
};

enum vacdev_action {
    VACDEV_REPORT_ONLY,
    VACDEV_MASK_ONLY,
    VACDEV_REPORT_AND_MASK
};


/**
 * Installs a generic access controlled device
 * @param[in] vm     The VM to install the device into
 * @param[in] d      A description of the device to install
 * @param[in] mask   An access mask. The mask provides a map of device bits that
 *                   are modifiable by the guest.
 *                   '1' represents bits that the guest can read and write
 *                   '0' represents bits that can only be read by the guest
 *                   Underlying memory for the mask should remain accessible for
 *                   the life of this device. The mask may be updated at run time
 *                   on demand.
 * @param[in] size   The size of the mask. This is useful for conserving memory in
 *                   cases where the underlying device does not occupy a full
 *                   page. If an access lies outside of the range of the mask,
 *                   guest access.
 * @param[in] action Action to take when access is violated.
 * @return           0 on success
 */
int vm_install_generic_ac_device(vm_t *vm, const struct device *d, void *mask,
                                 size_t size, enum vacdev_action action);
/**
 * Install a GPIO access control
 * @param[in] vm         The VM to install the device into
 * @param[in] default_ac The default access control state to apply
 * @param[in] action     Action to take when access is violated
 * @return               A handle to the GPIO Access control device, NULL on
 *                       failure
 */
struct gpio_device *vm_install_ac_gpio(vm_t *vm, enum vacdev_default default_ac,
                                       enum vacdev_action action);

/**
 * Provide GPIO pin access to the VM
 * @param[in] gpio_device  A handle to the GPIO Access Control device
 * @param[in] gpio_id      The ID of the GPIO to provide access to
 * @return                 0 on success
 */
int vm_gpio_provide(struct gpio_device *gpio_device, gpio_id_t gpio_id);

/**
 * Restrict GPIO pin access to the VM
 * @param[in] gpio_device  A handle to the GPIO Access Control device
 * @param[in] gpio_id      The ID of the GPIO to deny access to
 * @return                 0 on success
 */
int vm_gpio_restrict(struct gpio_device *gpio_device, gpio_id_t gpio_id);


/**** CLOCK ****/

/**
 * Install a CLOCK access control
 * @param[in] vm         The VM to install the device into
 * @param[in] default_ac The default access control state to apply
 * @param[in] action     Action to take when access is violated
 * @return               A handle to the GPIO Access control device, NULL on
 *                       failure
 */
struct clock_device *vm_install_ac_clock(vm_t *vm, enum vacdev_default default_ac,
                                         enum vacdev_action action);

/**
 * Provide clock access to the VM
 * @param[in] clock_device  A handle to the clock Access Control device
 * @param[in] clk_id        The ID of the clock to provide access to
 * @return                  0 on success
 */
int vm_clock_provide(struct clock_device *clock_device, enum clk_id clk_id);

/**
 * Deny clock access to the VM
 * @param[in] clock_device  A handle to the clock Access Control device
 * @param[in] clk_id        The ID of the clock to deny access to
 * @return                  0 on success
 */
int vm_clock_restrict(struct clock_device *clock_device, enum clk_id clk_id);
