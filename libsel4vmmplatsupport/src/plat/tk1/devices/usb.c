/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdlib.h>
#include <string.h>
#include <sel4/types.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>

#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/plat/usb.h>
#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/arch/guest_reboot.h>
#include <utils/io.h>

#define USB2_CONTROLLER_USB2D_USBCMD_0_OFFSET 0x130
#define USB2D_USB_COMMAND_REGISTER_RESET_BIT BIT(1)

const struct device dev_usb = {
    .name = "usb",
    .pstart = 0x7d004000,
    .size = PAGE_SIZE_4K,
    .priv = NULL
};

static int usb_vm_reboot_hook(vm_t *vm, void *token)
{
    void *usb_cmd_register = token + USB2_CONTROLLER_USB2D_USBCMD_0_OFFSET;
    uint32_t reg = RAW_READ32(usb_cmd_register);
    RAW_WRITE32(reg | USB2D_USB_COMMAND_REGISTER_RESET_BIT, usb_cmd_register);
    return 0;
}


static memory_fault_result_t handle_usb_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                              void *cookie)
{
    ZF_LOGE("Fault occured on passthrough usb device");
    return FAULT_ERROR;
}

int vm_install_tk1_usb_passthrough_device(vm_t *vm, reboot_hooks_list_t *reboot_hooks)
{
    /* Add the device */
    void *vmm_addr = create_device_reservation_frame(vm, dev_usb.pstart, seL4_AllRights, handle_usb_fault, NULL);
    if (vmm_addr == NULL) {
        ZF_LOGE("Failed to create passthrough usb device");
        return -1;
    }

    int err = vmm_register_reboot_callback(reboot_hooks, usb_vm_reboot_hook, vmm_addr);
    if (err) {
        ZF_LOGE("vm_register_reboot_callback returned error: %d", err);
        return -1;
    }

    return 0;
}
