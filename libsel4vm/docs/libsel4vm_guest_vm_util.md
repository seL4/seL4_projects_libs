<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vm_util.h`

The libsel4vm VM util interface provides a set of useful methods to query a guest vm instance.

### Brief content:

**Functions**:

> [`vm_get_vcpu_tcb(vcpu)`](#function-vm_get_vcpu_tcbvcpu)

> [`vm_get_vcpu(vm, vcpu_id)`](#function-vm_get_vcpuvm-vcpu_id)

> [`vm_vcpu_for_target_cpu(vm, target_cpu)`](#function-vm_vcpu_for_target_cpuvm-target_cpu)

> [`vm_find_free_unassigned_vcpu(vm)`](#function-vm_find_free_unassigned_vcpuvm)

> [`is_vcpu_online(vcpu)`](#function-is_vcpu_onlinevcpu)

> [`vm_get_vspace(vm)`](#function-vm_get_vspacevm)

> [`vm_get_vmm_vspace(vm)`](#function-vm_get_vmm_vspacevm)


## Functions

The interface `guest_vm_util.h` defines the following functions.

### Function `vm_get_vcpu_tcb(vcpu)`

Get the TCB CPtr a given VCPU is associated with

**Parameters:**

- `vcpu {vm_vcpu_t}`: A handle to the vcpu

**Returns:**

- seL4_CPtr of TCB object

Back to [interface description](#interface-guest_vm_utilh).

### Function `vm_get_vcpu(vm, vcpu_id)`

Get the VCPU CPtr associatated with a given logical ID

**Parameters:**

- `vm {vm_t *}`: A handle to the vm owning the vcpu
- `vcpu_id {int}`: Logical ID of the vcpu

**Returns:**

- seL4_CapNull if no vcpu exists, otherwise the seL4_CPtr of the VCPU object

Back to [interface description](#interface-guest_vm_utilh).

### Function `vm_vcpu_for_target_cpu(vm, target_cpu)`

Get the VCPU object that is assigned to a given target core ID

**Parameters:**

- `vm {vm_t *}`: A handle to the vm owning the vcpu
- `target_cpu {int}`: Target core ID

**Returns:**

- NULL if no vcpu is assigned to the target core, otherwise the vm_vcpu_t object

Back to [interface description](#interface-guest_vm_utilh).

### Function `vm_find_free_unassigned_vcpu(vm)`

Find a VCPU object that hasn't been assigned to a target core

**Parameters:**

- `vm {vm_t *}`: A handle to the vm owning the vcpu

**Returns:**

- NULL if no vcpu can be found, otherwise the vm_vcpu_t object

Back to [interface description](#interface-guest_vm_utilh).

### Function `is_vcpu_online(vcpu)`

Find if a given VCPU is online

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the vcpu

**Returns:**

- True if the vcpu is online, otherwise False

Back to [interface description](#interface-guest_vm_utilh).

### Function `vm_get_vspace(vm)`

Get the vspace of a given VM instance

**Parameters:**

- `vm {vm_t *}`: A handle to the VM

**Returns:**

- A vspace_t object

Back to [interface description](#interface-guest_vm_utilh).

### Function `vm_get_vmm_vspace(vm)`

Get the vspace of the host VMM associated with a given VM instance

**Parameters:**

- `vm {vm_t *}`: A handle to the VM

**Returns:**

- A vspace_t object

Back to [interface description](#interface-guest_vm_utilh).


Back to [top](#).

