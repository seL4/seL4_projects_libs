<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `boot.h`

The libsel4vm boot interface provides us with base abstractions to create, initialise and configure VM and VCPU instances.

### Brief content:

**Functions**:

> [`vm_init(vm, vka, host_simple, host_vspace, io_ops, host_endpoint, name)`](#function-vm_initvm-vka-host_simple-host_vspace-io_ops-host_endpoint-name)

> [`vm_create_vcpu(vm, priority)`](#function-vm_create_vcpuvm-priority)

> [`vm_assign_vcpu_target(vcpu, target_cpu)`](#function-vm_assign_vcpu_targetvcpu-target_cpu)


## Functions

The interface `boot.h` defines the following functions.

### Function `vm_init(vm, vka, host_simple, host_vspace, io_ops, host_endpoint, name)`

Initialise/Create VM

**Parameters:**

- `vm {vm_t *}`: Handle to the VM being initialised
- `vka {vka_t *}`: Initialised handle to virtual kernel allocator for seL4 kernel object allocation
- `host_simple {simple_t *}`: Initialised handle to hosts simple environment
- `host_vspace {vspace_t}`: Initialised handle to hosts vspace
- `ps_io_ops {ps_io_ops_t *}`: Initialised handle to platforms io ops
- `host_enpoint {seL4_CPtr}`: Host's endpoint. The library will wait and manage the endpoint when running a VM instance
- `name {const char *}`: String used to describe VM. Useful for debugging

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-booth).

### Function `vm_create_vcpu(vm, priority)`

Create a VCPU for a given VM

**Parameters:**

- `vm {vm_t *}`: A handle to VM being configured with a new vcpu
- `priority {int}`: The scheduling priority assigned to the VCPU thread

**Returns:**

- NULL for error, otherwise pointer to created vm_vcpu_t object

Back to [interface description](#interface-booth).

### Function `vm_assign_vcpu_target(vcpu, target_cpu)`

Assign a vcpu with logical target cpu to run on

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the VCPU
- `target {int}`: Logical target CPU ID

**Returns:**

- -1 for error, otherwise 0 for success

Back to [interface description](#interface-booth).


Back to [top](#).

