<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vcpu_fault.h`

The libsel4vm VCPU fault interface provides a set of useful methods to query and configure vcpu objects that
have faulted during execution. This interface is commonly leveraged by VMM's to process a vcpu fault and handle
it accordingly.

### Brief content:

**Functions**:

> [`get_vcpu_fault_address(vcpu)`](#function-get_vcpu_fault_addressvcpu)

> [`get_vcpu_fault_ip(vcpu)`](#function-get_vcpu_fault_ipvcpu)

> [`get_vcpu_fault_data(vcpu)`](#function-get_vcpu_fault_datavcpu)

> [`get_vcpu_fault_data_mask(vcpu)`](#function-get_vcpu_fault_data_maskvcpu)

> [`get_vcpu_fault_size(vcpu)`](#function-get_vcpu_fault_sizevcpu)

> [`is_vcpu_read_fault(vcpu)`](#function-is_vcpu_read_faultvcpu)

> [`set_vcpu_fault_data(vcpu, data)`](#function-set_vcpu_fault_datavcpu-data)

> [`emulate_vcpu_fault(vcpu, data)`](#function-emulate_vcpu_faultvcpu-data)

> [`advance_vcpu_fault(vcpu)`](#function-advance_vcpu_faultvcpu)

> [`restart_vcpu_fault(vcpu)`](#function-restart_vcpu_faultvcpu)


## Functions

The interface `guest_vcpu_fault.h` defines the following functions.

### Function `get_vcpu_fault_address(vcpu)`

Get current fault address of vcpu

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- Current fault address of vcpu

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `get_vcpu_fault_ip(vcpu)`

Get instruction pointer of current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- Intruction pointer of vcpu fault

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `get_vcpu_fault_data(vcpu)`

Get the data of the current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- Data of vcpu fault

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `get_vcpu_fault_data_mask(vcpu)`

Get data mask of the current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- Data mask of vcpu fault

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `get_vcpu_fault_size(vcpu)`

Get access size of the current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- Access size of vcpu fault

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `is_vcpu_read_fault(vcpu)`

Is current vcpu fault a read fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

- True if read fault, False if write fault

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `set_vcpu_fault_data(vcpu, data)`

Set the data of the current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu
- `data {seL4_Word}`: Data to set for current vcpu fault

**Returns:**

- 0 for success, otherwise -1 for error

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `emulate_vcpu_fault(vcpu, data)`

Emulate a read or write fault on a given data value

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu
- `data {seL4_Word}`: Data to perform emulate fault on

**Returns:**

- Emulation result of vcpu fault over given data value

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `advance_vcpu_fault(vcpu)`

Advance the current vcpu fault to the next stage/instruction

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

No return

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `restart_vcpu_fault(vcpu)`

Restart the current vcpu fault

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Handle to vcpu

**Returns:**

No return

Back to [interface description](#interface-guest_vcpu_faulth).


Back to [top](#).

