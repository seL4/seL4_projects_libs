<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vm_arch.h`

The guest x86 vm interface is central to using libsel4vm on an x86 platform, providing definitions of the x86 guest vm
datastructures and primitives to configure the VM instance.

### Brief content:



**Structs**:

> [`vm_vcpu`](#struct-vm_vcpu)

> [`vm_vcpu_arch`](#struct-vm_vcpu_arch)


## Structs

The interface `guest_vm_arch.h` defines the following structs.

### Struct `vm_vcpu`

Structure representing x86 specific vm properties

**Elements:**

- `vmexit_handler {vmexit_handler_ptr}`: Set of exit handler hooks
- `vmcall_handlers {vmcall_handler_t *}`: Set of registered vmcall handlers
- `vmcall_num_handler {unsigned int}`: Total number of registered vmcall handlers
- `guest_pd {uintptr_t}`: Guest physical address of where we built the vm's page directory
- `unhandled_ioport_callback {unhandled_ioport_callback_fn}`: A callback for processing unhandled ioport faults
- `unhandled_ioport_callback_cookie {void *}`: A cookie to supply to the ioport callback
- `ioport_list {vm_io_port_list_t}`: List of registered ioport handlers
- `i8259_gs {i8259_t *}`: PIC machine state

Back to [interface description](#interface-guest_vm_archh).

### Struct `vm_vcpu_arch`

Structure representing x86 specific vcpu properties

**Elements:**

- `guest_state {guest_state_t *}`: Current VCPU State
- `lapic {vm_lapic_t *}`: VM local apic

Back to [interface description](#interface-guest_vm_archh).


Back to [top](#).

