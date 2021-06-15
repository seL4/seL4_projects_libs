/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/sel4.h>

#include <vka/vka.h>
#include <simple/simple.h>
#include <vspace/vspace.h>
#include <platsupport/io.h>

#include <sel4vm/arch/guest_vm_arch.h>
#include <sel4vm/guest_memory.h>

typedef struct vm vm_t;
typedef struct vm_vcpu vm_vcpu_t;
typedef struct vm_mem vm_mem_t;
typedef struct vm_ram_region vm_ram_region_t;
typedef struct vm_run vm_run_t;
typedef struct vm_arch vm_arch_t;

/***
 * @module guest_vm.h
 * The guest vm interface is central to libsel4vm, providing definitions of the guest vm datastructure and
 * primitives to run the VM instance and start its vcpus.
 */

/**
 * Type signature of unhandled memory fault function, invoked when a memory fault is unable to be handled
 * @param {vm_t *} vm           A handle to the VM
 * @param {vm_vcpu_t *} vcpu    A handle to the fault vcpu
 * @param {uintptr_t} paddr     Faulting guest physical address
 * @param {size_t} len          Length of faulted access
 * @param {void *} cookie       User cookie to pass onto callback
 * @return                      Fault handling status code: HANDLED, UNHANDLED, RESTART, ERROR
 */
typedef memory_fault_result_t (*unhandled_mem_fault_callback_fn)(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t paddr,
                                                                 size_t len, void *cookie);

/**
 * Type signature of unhandled notification callback, invoked when a notification is recieved and cannot be processed
 * by the vm runtime
 * @param {vm_t *} vm                   A handle to the VM
 * @param {seL4_Word} badge             Badge of inbound event
 * @param {seL4_MessageInfo_t} tag      seL4 Message Info tag of inbound event (if IPC related)
 * @param {void *} cookie               User cookie to pass onto callback
 * @return                              -1 on failure otherwise 0 for success
 */
typedef int (*notification_callback_fn)(vm_t *vm, seL4_Word badge, seL4_MessageInfo_t tag,
                                        void *cookie);

/***
 * @struct vm_ram_region
 * Structure representing individual RAM region. A VM can have multiple regions to represent its total RAM
 * @param {uintptr_t} start     Guest physical start address of region
 * @param {size_t} size         Size of region in bytes
 * @param {int} allocated       Whether or not this region has been 'allocated'
 */
struct vm_ram_region {
    uintptr_t start;
    size_t size;
    int allocated;
};

/***
 * @struct vm_mem
 * Structure representing VM memory managment
 * @param {vspace_t} vm_vspace                                              Guest VM's vspace
 * @param {vka_object_t} vm_vspace_root                                     VKA allocated guest VM root vspace
 * @param {vspace_t} vmm_vspace                                             Hosts/VMMs vspace
 * @param {int} num_ram_regions                                             Total number of registered `vm_ram_regions`
 * @param {struct vm_ram_region *}                                          Set of registered `vm_ram_regions`
 * @param {vm_memory_reservation_cookie_t *}                                Initialised instance of vm memory interface
 * @param {unhandled_mem_fault_callback_fn}  unhandled_mem_fault_handler    Registered callback for unhandled memory faults
 * @param {void *} unhandled_mem_fault_cookie                               User data passed onto unhandled mem fault callback
 * @param {int} clean_cache                                                 Flag to clean cache when loading images
 * @param {int} map_one_to_one                                              Flag to tell VMM to map memory 1:1
 */
struct vm_mem {
    /* Guest vm vspace management */
    vspace_t vm_vspace;
    /* Guest vm root vspace */
    vka_object_t vm_vspace_root;
    /* vmm vspace */
    vspace_t vmm_vspace;
    /* Guest vm ram regions */
    /* We maintain all pieces of ram as a sorted list of regions.
     * This is memory that we will specifically give the guest as actual RAM */
    int num_ram_regions;
    struct vm_ram_region *ram_regions;
    /* Memory reservations */
    vm_memory_reservation_cookie_t *reservation_cookie;
    unhandled_mem_fault_callback_fn unhandled_mem_fault_handler;
    void *unhandled_mem_fault_cookie;
    int clean_cache;
    int map_one_to_one;
};

/***
 * @struct vm_tcb
 * Structure used for TCB management within a VCPU
 * @param {vka_object_t} tcb            VKA allocated TCB object
 * @param {vka_object_t} sc             VKA allocated scheduling context
 * @param {int} priority                VCPU scheduling priority
 */
struct vm_tcb {
    /* Guest vm tcb management objects */
    vka_object_t tcb;
    vka_object_t sc;
    vka_object_t sched_ctrl;
    /* vcpu scheduling priority */
    int priority;
};

/***
 * @struct vm_vcpu
 * Structure used to represent a VCPU
 * @param {struct vm *} vm                  Parent VM
 * @param {vka_object_t} vcpu               VKA allocated vcpu object
 * @param {struct vm_tcb} tcb               VCPUs TCB management structure
 * @param {unsigned int} vcpu_id            VCPU Identifier
 * @param {int} target_cpu                  The target core the vcpu is assigned to
 * @param {bool} vcpu_online                Flag representing if the vcpu has been started
 * @param {struct vm_vcpu_arch} vcpu_arch   Architecture specific vcpu properties
 */
struct vm_vcpu {
    /* Parent vm */
    struct vm *vm;
    /* Kernel vcpu object */
    vka_object_t vcpu;
    /* vm tcb */
    struct vm_tcb tcb;
    /* Id of vcpu */
    unsigned int vcpu_id;
    /* The target core the vcpu is assigned to */
    int target_cpu;
    /* is the vcpu online */
    bool vcpu_online;
    /* Architecture specfic vcpu */
    struct vm_vcpu_arch vcpu_arch;
};

/***
 * @struct vm_run
 * VM Runtime management structure
 * @param {int} exit_reason                                     Records last vm exit reason
 * @param {notification_callback_fn} notification_callback      Callback for processing unhandled notifications
 * @param {void *} notification_callback_cookie                 A cookie to supply to the notification callback
 */
struct vm_run {
    int exit_reason;
    notification_callback_fn notification_callback;
    void *notification_callback_cookie;
};

/***
 * @struct vm_cspace
 * VM cspace management structure
 * @param {vka_object_t} cspace_obj     VKA allocated cspace object
 * @param {seL4_Word} cspace_root_data  cspace root data capability
 */
struct vm_cspace {
    vka_object_t cspace_obj;
    seL4_Word cspace_root_data;
};

/***
 * @struct vm
 * Structure representing a VM instance
 * @param {struct vm_arch} arch         Architecture specfic vm structure
 * @param {unsigned int} num_vcpus      Number of vcpus created for the VM
 * @param {struct vm_vcpu*} vcpus       vcpu's belonging to the VM
 * @param {struct vm_mem} mem           Memory management structure
 * @param {struct vm_run} run           VM Runtime management structure
 * @param {struct vm_cspace} cspace     VM CSpace management structure
 * @param {seL4_CPtr} host_endpoint     Host/VMM endpoint. `vm_run` waits on this enpoint
 * @param {vka_t *} vka                 Handle to virtual kernel allocator for seL4 kernel object allocation
 * @param {ps_io_ops_t *} io_ops        Handle to platforms io ops
 * @param {simple_t *} simple           Handle to hosts simple environment
 * @param {seL4_Word} entry             Entry address for the loaded kernel
 * @param {char *} vm_name              String used to describe VM. Useful for debugging
 * @param {unsigned int} vm_id          Identifier for VM. Useful for debugging
 * @param {bool} vm_initialised         Boolean flagging whether VM is intialised or not
 */
struct vm {
    /* Architecture specfic vm structure */
    struct vm_arch arch;
    /* vm vcpus */
    unsigned int num_vcpus;
    struct vm_vcpu *vcpus[CONFIG_MAX_NUM_NODES];
    /* vm memory management */
    struct vm_mem mem;
    /* vm runtime management */
    struct vm_run run;
    /* Guest vm cspace */
    struct vm_cspace cspace;
    /* Host endoint (i.e. vmm) to wait for VM faults and host events */
    seL4_CPtr host_endpoint;
    /* Support & Resource Managements */
    vka_t *vka;
    ps_io_ops_t *io_ops;
    simple_t *simple;

    /* vm entry address */
    seL4_Word entry;

    /* Debugging & Identification */
    char *vm_name;
    unsigned int vm_id;
    bool vm_initialised;
};

/***
 * @function vm_run(vm_t *vm)
 * Enter the VM event runtime loop.
 * This funtion is a blocking call, returning on the event of an unhandled VM exit or error
 * @param {vm_t *} vm   A handle to the VM to run
 * @return              0 on success, -1 on error
 */
int vm_run(vm_t *vm);

/***
 * @function vcpu_start(vcpu)
 * Start an initialised vcpu thread
 * @param {vm_vcpu_t *} vcpu    A handle to vcpu to start
 * @return                      0 on success, -1 on error
 */
int vcpu_start(vm_vcpu_t *vcpu);

/* Unhandled fault callback registration functions */

/***
 * @function vm_register_unhandled_mem_fault_callback(vm, fault_handler, cookie)
 * Register a callback for processing unhandled memory faults (memory regions not previously registered or reserved)
 * @param {vm_t *} vm                                           A handle to the VM
 * @param {unhandled_mem_fault_callback_fn} fault_handler       A user supplied callback to process unhandled memory faults
 * @param {void *} cookie                                       A cookie to supply to the memory fault handler
 * @return                                                      0 on success, -1 on error
 */
int vm_register_unhandled_mem_fault_callback(vm_t *vm, unhandled_mem_fault_callback_fn fault_handler,
                                             void *cookie);

/***
 * @function vm_register_notification_callback(vm, notification_callback, cookie)
 * Register a callback for processing unhandled notifications (events unknown to libsel4vm)
 * @param {vm_t *} vm                                           A handle to the VM
 * @param {notification_callback_fn} notification_callback      A user supplied callback to process unhandled notifications
 * @param {void *} cookie                                       A cookie to supply to the callback
 * @return                                                      0 on success, -1 on error
 */
int vm_register_notification_callback(vm_t *vm, notification_callback_fn notification_callback,
                                      void *cookie);
