<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vm.h`

The guest vm interface is central to libsel4vm, providing definitions of the guest vm datastructure and
primitives to run the VM instance and start its vcpus.

### Brief content:

**Functions**:

> [`vm_run(vm_t *vm)`](#function-vm_runvm_t-vm)

> [`vcpu_start(vcpu)`](#function-vcpu_startvcpu)

> [`vm_register_unhandled_mem_fault_callback(vm, fault_handler, cookie)`](#function-vm_register_unhandled_mem_fault_callbackvm-fault_handler-cookie)

> [`vm_register_notification_callback(vm, notification_callback, cookie)`](#function-vm_register_notification_callbackvm-notification_callback-cookie)



**Structs**:

> [`vm_ram_region`](#struct-vm_ram_region)

> [`vm_mem`](#struct-vm_mem)

> [`vm_tcb`](#struct-vm_tcb)

> [`vm_vcpu`](#struct-vm_vcpu)

> [`vm_run`](#struct-vm_run)

> [`vm_cspace`](#struct-vm_cspace)

> [`vm`](#struct-vm)


## Functions

The interface `guest_vm.h` defines the following functions.

### Function `vm_run(vm_t *vm)`

Enter the VM event runtime loop.
This funtion is a blocking call, returning on the event of an unhandled VM exit or error

**Parameters:**

- `vm {vm_t *}`: A handle to the VM to run

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_vmh).

### Function `vcpu_start(vcpu)`

Start an initialised vcpu thread

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to vcpu to start

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_vmh).

### Function `vm_register_unhandled_mem_fault_callback(vm, fault_handler, cookie)`

Register a callback for processing unhandled memory faults (memory regions not previously registered or reserved)

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `fault_handler {unhandled_mem_fault_callback_fn}`: A user supplied callback to process unhandled memory faults
- `cookie {void *}`: A cookie to supply to the memory fault handler

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_vmh).

### Function `vm_register_notification_callback(vm, notification_callback, cookie)`

Register a callback for processing unhandled notifications (events unknown to libsel4vm)

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `notification_callback {notification_callback_fn}`: A user supplied callback to process unhandled notifications
- `cookie {void *}`: A cookie to supply to the callback

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_vmh).


## Structs

The interface `guest_vm.h` defines the following structs.

### Struct `vm_ram_region`

Structure representing individual RAM region. A VM can have multiple regions to represent its total RAM

**Elements:**

- `start {uintptr_t}`: Guest physical start address of region
- `size {size_t}`: Size of region in bytes
- `allocated {int}`: Whether or not this region has been 'allocated'

Back to [interface description](#interface-guest_vmh).

### Struct `vm_mem`

Structure representing VM memory managment

**Elements:**

- `vm_vspace {vspace_t}`: Guest VM's vspace
- `vm_vspace_root {vka_object_t}`: VKA allocated guest VM root vspace
- `vmm_vspace {vspace_t}`: Hosts/VMMs vspace
- `num_ram_regions {int}`: Total number of registered `vm_ram_regions`
- `Set {struct vm_ram_region *}`: of registered `vm_ram_regions`
- `Initialised {vm_memory_reservation_cookie_t *}`: instance of vm memory interface
- `unhandled_mem_fault_handler {unhandled_mem_fault_callback_fn}`: Registered callback for unhandled memory faults
- `unhandled_mem_fault_cookie {void *}`: User data passed onto unhandled mem fault callback

Back to [interface description](#interface-guest_vmh).

### Struct `vm_tcb`

Structure used for TCB management within a VCPU

**Elements:**

- `tcb {vka_object_t}`: VKA allocated TCB object
- `sc {vka_object_t}`: VKA allocated scheduling context
- `priority {int}`: VCPU scheduling priority

Back to [interface description](#interface-guest_vmh).

### Struct `vm_vcpu`

Structure used to represent a VCPU

**Elements:**

- `vm {struct vm *}`: Parent VM
- `vcpu {vka_object_t}`: VKA allocated vcpu object
- `tcb {struct vm_tcb}`: VCPUs TCB management structure
- `vcpu_id {unsigned int}`: VCPU Identifier
- `target_cpu {int}`: The target core the vcpu is assigned to
- `vcpu_online {bool}`: Flag representing if the vcpu has been started
- `vcpu_arch {struct vm_vcpu_arch}`: Architecture specific vcpu properties

Back to [interface description](#interface-guest_vmh).

### Struct `vm_run`

VM Runtime management structure

**Elements:**

- `exit_reason {int}`: Records last vm exit reason
- `notification_callback {notification_callback_fn}`: Callback for processing unhandled notifications
- `notification_callback_cookie {void *}`: A cookie to supply to the notification callback

Back to [interface description](#interface-guest_vmh).

### Struct `vm_cspace`

VM cspace management structure

**Elements:**

- `cspace_obj {vka_object_t}`: VKA allocated cspace object
- `cspace_root_data {seL4_Word}`: cspace root data capability

Back to [interface description](#interface-guest_vmh).

### Struct `vm`

Structure representing a VM instance

**Elements:**

- `arch {struct vm_arch}`: Architecture specfic vm structure
- `num_vcpus {unsigned int}`: Number of vcpus created for the VM
- `vcpus {struct vm_vcpu*}`: vcpu's belonging to the VM
- `mem {struct vm_mem}`: Memory management structure
- `run {struct vm_run}`: VM Runtime management structure
- `cspace {struct vm_cspace}`: VM CSpace management structure
- `host_endpoint {seL4_CPtr}`: Host/VMM endpoint. `vm_run` waits on this enpoint
- `vka {vka_t *}`: Handle to virtual kernel allocator for seL4 kernel object allocation
- `io_ops {ps_io_ops_t *}`: Handle to platforms io ops
- `simple {simple_t *}`: Handle to hosts simple environment
- `vm_name {char *}`: String used to describe VM. Useful for debugging
- `vm_id {unsigned int}`: Identifier for VM. Useful for debugging
- `vm_initialised {bool}`: Boolean flagging whether VM is intialised or not

Back to [interface description](#interface-guest_vmh).


Back to [top](#).

