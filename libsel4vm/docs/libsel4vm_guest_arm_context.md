<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_arm_context.h`

The libsel4vm ARM context interface provides a set of useful getters and setters on ARM vcpu thread contexts.
This interface is commonly leveraged by VMM's to initialise a vcpu state, process a vcpu fault and
accordingly update its state.

### Brief content:

**Functions**:

> [`vm_set_thread_context(vcpu, context)`](#function-vm_set_thread_contextvcpu-context)

> [`vm_set_thread_context_reg(vcpu, reg, value)`](#function-vm_set_thread_context_regvcpu-reg-value)

> [`vm_get_thread_context(vcpu, context)`](#function-vm_get_thread_contextvcpu-context)

> [`vm_get_thread_context_reg(vcpu, reg, value)`](#function-vm_get_thread_context_regvcpu-reg-value)

> [`vm_set_arm_vcpu_reg(vcpu, reg, value)`](#function-vm_set_arm_vcpu_regvcpu-reg-value)

> [`vm_get_arm_vcpu_reg(vcpu, reg, value)`](#function-vm_get_arm_vcpu_regvcpu-reg-value)


## Functions

The interface `guest_arm_context.h` defines the following functions.

### Function `vm_set_thread_context(vcpu, context)`

Set a VCPU's thread registers given a TCB user context

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `context {seL4_UserContext}`: seL4_UserContext applied to VCPU's TCB

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).

### Function `vm_set_thread_context_reg(vcpu, reg, value)`

Set a single VCPU's TCB register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {unsigned int}`: Index offset of register in seL4_UserContext e.g pc (seL4_UserContext.pc) => 0
- `value {uintptr_t}`: Value to set TCB register with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).

### Function `vm_get_thread_context(vcpu, context)`

Get a VCPU's TCB user context

**Parameters:**

- `vcpu {vm_vcpu_t}`: Handle to the vcpu
- `context {seL4_UserContext}`: Pointer to user supplied seL4_UserContext to populate with VCPU's TCB user context

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).

### Function `vm_get_thread_context_reg(vcpu, reg, value)`

Get a single VCPU's TCB register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {unsigned int}`: Index offset of register in seL4_UserContext e.g pc (seL4_UserContext.pc) => 0
- `value {uintptr_t *}`: Pointer to user supplied variable to populate TCB register value with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).

### Function `vm_set_arm_vcpu_reg(vcpu, reg, value)`

Set an ARM VCPU register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {seL4_Word}`: VCPU Register field defined in seL4_VCPUReg
- `value {uintptr_t *}`: Value to set VCPU register with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).

### Function `vm_get_arm_vcpu_reg(vcpu, reg, value)`

Get an ARM VCPU register

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to the vcpu
- `reg {seL4_Word}`: VCPU Register field defined in seL4_VCPUReg
- `value {uintptr_t *}`: Pointer to user supplied variable to populate VCPU register value with

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-guest_arm_contexth).


Back to [top](#).

