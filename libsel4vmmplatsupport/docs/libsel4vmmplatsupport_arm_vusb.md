<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `vusb.h`

The libsel4vmmplatsupport vusb interface presents a Virtual USB driver for ARM-based VM's.

### Brief content:

**Functions**:

> [`vm_install_vusb(vm, hcd, pbase, virq, vmm_ncap, vm_ncap, badge)`](#function-vm_install_vusbvm-hcd-pbase-virq-vmm_ncap-vm_ncap-badge)

> [`vm_vusb_notify(vusb)`](#function-vm_vusb_notifyvusb)


## Functions

The interface `vusb.h` defines the following functions.

### Function `vm_install_vusb(vm, hcd, pbase, virq, vmm_ncap, vm_ncap, badge)`

Install a virtual usb device
Calls made to this hcd may be redirected for filtering.

**Parameters:**

- `vm {vm_t *}`: The VM in which to install the device
- `hcd {usb_host_t *}`: The USB host controller that should be used for USB transactions.
- `pbase {uintptr_t}`: The guest physical address of the device (2 pages)
- `virq {int}`: The virtual IRQ number for this device
- `vmm_ncap {seL4_CPtr}`: The capability to the endpoint at which the VMM waits for notifications.
- `vm_ncap {seL4_CPtr}`: The index at which to install a notification capability into the VM
- `badge {int}`: The seL4 badge which should be applied to the notification capability.

**Returns:**

- A handle to the virtual usb device, or NULL on failure

Back to [interface description](#interface-vusbh).

### Function `vm_vusb_notify(vusb)`

This function should be called when a notification is received from the
VM. The notification is identifyable by a message on the fault endpoint
of the VM which has a badge that matches that which was passed into the
vm_install_vusb function.

**Parameters:**

- `vusb {vusb_device_t *}`: A handle to a virtual usb device

**Returns:**

No return

Back to [interface description](#interface-vusbh).


Back to [top](#).

