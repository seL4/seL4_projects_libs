<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_x86_context.h`

The libsel4vm x86 context interface provides a set of useful getters and setters on x86 vcpu thread contexts.
This interface is commonly leveraged by VMM's to initialise a vcpu state, process a vcpu fault and
accordingly update its state.

### Brief content:

**Functions**:

> [`vm_set_thread_context(vcpu, context)`](#function-vm_set_thread_contextvcpu-context)

> [`vm_set_thread_context_reg(vcpu, reg, value)`](#function-vm_set_thread_context_regvcpu-reg-value)

> [`vm_get_thread_context(vcpu, context)`](#function-vm_get_thread_contextvcpu-context)

> [`vm_get_thread_context_reg(vcpu, reg, value)`](#function-vm_get_thread_context_regvcpu-reg-value)

> [`vm_set_vmcs_field(vcpu, field, value)`](#function-vm_set_vmcs_fieldvcpu-field-value)

> [`vm_get_vmcs_field(vcpu, field, value)`](#function-vm_get_vmcs_fieldvcpu-field-value)


## Functions

The interface `guest_x86_context.h` defines the following functions.

### Function `vm_set_thread_context(vcpu, context)`

Set a VCPU's thread registers given a seL4_VCPUContext

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `context {seL4_VCPUContext}`: seL4_VCPUContext applied to VCPU Registers

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).

### Function `vm_set_thread_context_reg(vcpu, reg, value)`

Set a single VCPU's thread register in a seL4_VCPUContext

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {vcpu_context_reg_t}`: Register enumerated by vcpu_context_reg
- `value {uint32_t}`: Value to set register with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).

### Function `vm_get_thread_context(vcpu, context)`

Get a VCPU's thread context

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `context {seL4_VCPUContext *}`: Pointer to user supplied seL4_VCPUContext to populate with VCPU's current context

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).

### Function `vm_get_thread_context_reg(vcpu, reg, value)`

Get a single VCPU's thread register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {vcpu_context_reg_t}`: Register enumerated by vcpu_context_reg
- `value {uint32_t *}`: Pointer to user supplied variable to populate register value with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).

### Function `vm_set_vmcs_field(vcpu, field, value)`

Set a VMCS field

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {seL4_Word}`: VMCS field
- `value {uint32_t}`: Value to set VMCS field with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).

### Function `vm_get_vmcs_field(vcpu, field, value)`

Get a VMCS register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {seL4_Word}`: VMCS field
- `value {uint32_t *}`: Pointer to user supplied variable to populate VMCS field value with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_x86_contexth).


Back to [top](#).

