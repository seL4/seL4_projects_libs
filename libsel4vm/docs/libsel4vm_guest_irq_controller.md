<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_irq_controller.h`

The libsel4vm IRQ controller interface provides a base abstraction around initialising a guest VM irq controller
and methods for injecting IRQs into a running VM instance.

### Brief content:

**Functions**:

> [`vm_inject_irq(vcpu, irq)`](#function-vm_inject_irqvcpu-irq)

> [`vm_set_irq_level(vcpu, irq, irq_level)`](#function-vm_set_irq_levelvcpu-irq-irq_level)

> [`vm_register_irq(vcpu, irq, ack_fn, cookie)`](#function-vm_register_irqvcpu-irq-ack_fn-cookie)

> [`vm_create_default_irq_controller(vm)`](#function-vm_create_default_irq_controllervm)


## Functions

The interface `guest_irq_controller.h` defines the following functions.

### Function `vm_inject_irq(vcpu, irq)`

Inject an IRQ into a VM's interrupt controller

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the VCPU
- `irq {int}`: IRQ number to inject

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_irq_controllerh).

### Function `vm_set_irq_level(vcpu, irq, irq_level)`

Set level of IRQ number into a VM's interrupt controller

**Parameters:**

- `vcpu {vm_vcpu_t}`: Handle to the VCPU
- `irq {int}`: IRQ number to set level on
- `irq_level {int}`: Value of IRQ level

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_irq_controllerh).

### Function `vm_register_irq(vcpu, irq, ack_fn, cookie)`

Register irq with an acknowledgment function

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the VCPU
- `irq {int}`: IRQ number to register acknowledgement function on
- `ack_fn {irq_ack_fn_t}`: IRQ acknowledgement function
- `cookie {void *}`: Cookie to pass back with IRQ acknowledgement function

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_irq_controllerh).

### Function `vm_create_default_irq_controller(vm)`

Install the default interrupt controller into the VM

**Parameters:**

- `vm {vm_t *}`: Handle to the VM

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_irq_controllerh).


Back to [top](#).

