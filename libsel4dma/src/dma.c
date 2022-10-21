/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
/* Author: alex.kroh@nicta.com.au */

#include <dma/dma.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <autoconf.h>

//#define DMA_DEBUG
#ifdef DMA_DEBUG
#define dprintf(...) do{     \
        printf("DMA| ");     \
        printf(__VA_ARGS__); \
    }while(0)
#else
#define dprintf(...) do{}while(0)
#endif

#if defined(CONFIG_PLAT_IMX6) || defined(IMX6)
#define DMA_MINALIGN_BYTES 32
#elif defined(CONFIG_PLAT_EXYNOS4)
#define DMA_MINALIGN_BYTES 32
#elif defined(CONFIG_PLAT_EXYNOS5)
#define DMA_MINALIGN_BYTES 32
#elif defined(CONFIG_PLAT_ZYNQMP)
#define DMA_MINALIGN_BYTES 32
#else
#warning Unknown platform. DMA alignment defaulting to 32 bytes.
#define DMA_MINALIGN_BYTES 32
#endif

#define _malloc malloc
#define _free   free

/* dma_mem_t flag bit that signals that the memory is in use */
#define DMFLAG_ALLOCATED 1

/* Linked list of descriptors */
struct dma_memd_node {
    /* Description of this memory chunk */
    struct dma_mem_descriptor desc;
    /* Number of frames in this region */
    int nframes;
    /* Caps to the underlying frames */
    void **alloc_cookies;
    /* Head of linked list of regions */
    dma_mem_t dma_mem_head;
    /* Chain */
    struct dma_memd_node *next;
};

/* Linked list of memory regions */
struct dma_mem {
    /* Offset within the described memory */
    uintptr_t offset;
    /* Flags */
    int flags;
    /* Parent node */
    struct dma_memd_node *node;
    /* Chain */
    dma_mem_t next;
};

struct dma_allocator {
    dma_morecore_fn morecore;
    struct dma_memd_node *head;
};


/*** Helpers ***/

static inline int _is_free(dma_mem_t m)
{
    return !(m->flags & DMFLAG_ALLOCATED);
}

static inline size_t _node_size(struct dma_memd_node *n)
{
    return n->nframes << n->desc.size_bits;
}

static inline size_t _mem_size(dma_mem_t m)
{
    struct dma_memd_node *n = m->node;
    size_t s;
    if (m->next == NULL) {
        s = _node_size(n);;
    } else {
        s = m->next->offset;
    }
    return s -= m->offset;
}

/* @pre the offset must be contained within the node provided node */
static inline dma_mem_t _find_mem(struct dma_memd_node *n, uintptr_t offset)
{
    dma_mem_t m;
    assert(n);
    assert(offset < _node_size(n));
    m = n->dma_mem_head;
    while (m->next != NULL && offset >= m->next->offset) {
        m = m->next;
    }
    return m;
}

static void _mem_compact(dma_mem_t m)
{
    while (m->next != NULL && _is_free(m->next)) {
        dma_mem_t compact = m->next;
        dprintf("Compacting:\n");
        m->next = compact->next;
        _free(compact);
    }
}

/*** Debug ***/

static void print_dma_mem(dma_mem_t m, const char *prefix)
{
    dprintf("%s{p0x%08x, v0x%08x, s0x%x %s}\n", prefix,
            (uint32_t)dma_paddr(m), (uint32_t)dma_vaddr(m), _mem_size(m),
            _is_free(m) ? "FREE" : "USED");
}

static void print_dma_node(struct dma_memd_node *n)
{
    dma_mem_t m;
    dprintf("NODE:\n");
    for (m = n->dma_mem_head; m != NULL; m = m->next) {
        print_dma_mem(m, ">");
    }
}

static void print_dma_allocator(struct dma_allocator *a)
{
    struct dma_memd_node *n;
    dprintf("ALLOC:\n");
    for (n = a->head; n != NULL; n = n->next) {
        print_dma_node(n);
    }
}

/*** Interface ***/


static struct dma_memd_node *do_dma_provide_mem(struct dma_allocator *allocator,
                                                struct dma_mem_descriptor *dma_desc)
{
    struct dma_memd_node *n;
    dma_mem_t m;

    /* The memory size must be sane */
    assert(dma_desc->size_bits > 0 && dma_desc->size_bits < 32);
    /* Allocate some objects */
    m = (dma_mem_t)_malloc(sizeof(*m));
    if (m == NULL) {
        return NULL;
    }
    /* We do not attempt to tack this region onto an existing node */
    n = (struct dma_memd_node *)_malloc(sizeof(*n));
    if (n == NULL) {
        _free(m);
        return NULL;
    }
    /* Initialise free memory */
    m->offset = 0;
    m->flags = 0;
    m->next = NULL;
    m->node = n;
    /* Initialise the pool node */
    n->desc = *dma_desc;
    n->dma_mem_head = m;
    n->next = allocator->head;
    n->nframes = 1;
    n->alloc_cookies = _malloc(sizeof(*n->alloc_cookies) * n->nframes);
    if (n->alloc_cookies == NULL) {
        _free(n);
        _free(m);
        return NULL;
    }
    n->alloc_cookies[0] = dma_desc->alloc_cookie;
    /* Add the node to the allocator */
    allocator->head = n;

    dprintf("DMA memory provided\n");
    print_dma_allocator(allocator);
    return n;
}



struct dma_allocator *
dma_allocator_init(dma_morecore_fn morecore)
{
    struct dma_allocator *alloc;
    alloc = (struct dma_allocator *)_malloc(sizeof(*alloc));
    if (alloc == NULL) {
        return NULL;
    }
    alloc->morecore = morecore;
    alloc->head = NULL;
    return alloc;
}


int dma_provide_mem(struct dma_allocator *allocator,
                    struct dma_mem_descriptor dma_desc)
{
    struct dma_memd_node *n;
    n = do_dma_provide_mem(allocator, &dma_desc);
    return n == NULL;
}

static dma_mem_t dma_memd_alloc(struct dma_memd_node *n, size_t size, int align)
{
    dma_mem_t m;
    dprintf("Allocating 0x%x aligned to 0x%x\n", size, align);
    for (m = n->dma_mem_head; m != NULL; m = m->next) {
        if (_is_free(m)) {
            size_t mem_size;
            int mem_align;
            paddr_t paddr;

            _mem_compact(m);

            /* Find the region size and alignment */
            paddr = dma_paddr(m);
            mem_align = align - (paddr % align);
            if (mem_align == align) {
                mem_align = 0;
            }
            mem_size = _mem_size(m) - mem_align;
            /* Check for overflow in subtraction and check our size */
            if (_mem_size(m) > mem_align && mem_size >= size) {
                m->offset += mem_align;
                /* Split off the free memory if possible */
                if (mem_size > size) {
                    dma_mem_t split;
                    split = (dma_mem_t)_malloc(sizeof(*split));
                    if (split != NULL) {
                        split->offset = m->offset + size;
                        split->flags = m->flags;
                        split->next = m->next;
                        split->node = m->node;

                        m->next = split;
                    } else {
                        /* Just use the over size region... */
                    }
                }
                m->flags |= DMFLAG_ALLOCATED;
                return m;
            }
        }
    }
    return NULL;
}

vaddr_t dma_alloc(struct dma_allocator *allocator, size_t size, int align,
                  enum dma_flags flags, dma_mem_t *ret_mem)
{
    int cached;
    struct dma_memd_node *n;
    assert(allocator);

    if (align < DMA_MINALIGN_BYTES) {
        align = DMA_MINALIGN_BYTES;
    }
    /* TODO access patterns and cachability */
    cached = 0;
    (void)flags;

    /* Cycle through nodes looking for free memory */
    for (n = allocator->head; n != NULL; n = n->next) {
        dma_mem_t m;
        /* TODO some tricky stuff first to see if an allocation from
         * this node is appropriate (access patterns/cachability) */
        m = dma_memd_alloc(n, size, align);
        if (m != NULL) {
            dprintf("DMA mem allocated\n");
            print_dma_mem(m, "-");
            print_dma_allocator(allocator);
            if (ret_mem) {
                *ret_mem = m;
            }
            return dma_vaddr(m);
        }
    }

    /* Out of memory! Allocate more if we have the ability */
    if (allocator->morecore) {
        struct dma_mem_descriptor dma_desc;
        struct dma_memd_node *n;
        dma_mem_t m;
        int err;
        dprintf("Morecore called for %d bytes\n", size);
        /* Grab more core */
        err = allocator->morecore(size, cached, &dma_desc);
        if (err) {
            return NULL;
        }
        /* Add the memory to the allocator */
        n = do_dma_provide_mem(allocator, &dma_desc);
        if (n == NULL) {
            return NULL;
        }
        /* Perform the allocation */
        m = dma_memd_alloc(n, size, align);
        assert(m);
        if (m == NULL) {
            return NULL;
        }
        /* Success */
        dprintf("DMA mem allocated\n");
        print_dma_mem(m, "-");
        print_dma_allocator(allocator);
        if (ret_mem) {
            *ret_mem = m;
        }
        return dma_vaddr(m);
    }
    dprintf("Failed to allocate DMA memory\n");
    return NULL;
}

int dma_reclaim_mem(struct dma_allocator *allocator,
                    struct dma_mem_descriptor *dma_desc)
{
    struct dma_memd_node *n;
    struct dma_memd_node **nptr = &allocator->head;
    for (n = allocator->head; n != NULL; n = n->next) {
        dma_mem_t m = n->dma_mem_head;
        _mem_compact(m);
        if (_is_free(m) && !m->next) {
            *dma_desc = n->desc;
            /* Currently there is not support for compacted nodes */
            assert(n->nframes == 1);
            dma_desc->alloc_cookie = n->alloc_cookies[0];
            /* Remove the node and free memory */
            *nptr = n->next;
            _free(m);
            _free(n->alloc_cookies);
            _free(n);
            return 0;
        }
        nptr = &n->next;
    }
    return -1;
}

void dma_free(dma_mem_t m)
{
    if (m) {
        m->flags &= ~DMFLAG_ALLOCATED;
        _mem_compact(m);
    }
}

/*** Address translation ***/

vaddr_t dma_vaddr(dma_mem_t m)
{
    if (m) {
        assert(m->node);
        return (vaddr_t)((uintptr_t)m->node->desc.vaddr + m->offset);
    } else {
        return (vaddr_t)0;
    }
}

paddr_t dma_paddr(dma_mem_t m)
{
    if (m) {
        assert(m->node);
        return m->node->desc.paddr + m->offset;
    } else {
        return (paddr_t)0;
    }
}

dma_mem_t dma_plookup(struct dma_allocator *dma_allocator, paddr_t paddr)
{
    struct dma_memd_node *n;
    uintptr_t offs;
    /* Search the list for the associated node */
    for (n = dma_allocator->head; n != NULL; n = n->next) {
        if (n->desc.paddr <= paddr && n->desc.paddr < _node_size(n)) {
            break;
        }
    }
    /* Search the mem list for the assocated dma_mem_t */
    if (n == NULL) {
        return NULL;
    } else {
        offs = paddr - n->desc.paddr;
        return _find_mem(n, offs);
    }
}

dma_mem_t dma_vlookup(struct dma_allocator *dma_allocator, vaddr_t vaddr)
{
    struct dma_memd_node *n;
    uintptr_t offs;
    /* Search the list for the associated node */
    for (n = dma_allocator->head; n != NULL; n = n->next) {
        uintptr_t vn = (uintptr_t)n->desc.vaddr;
        uintptr_t vm = (uintptr_t)vaddr;
        if (vn <= vm && vm < vn + _node_size(n)) {
            break;
        }
    }
    /* Search the mem list for the assocated dma_mem_t */
    if (n == NULL) {
        return NULL;
    } else {
        offs = (uintptr_t)vaddr - (uintptr_t)n->desc.vaddr;
        return _find_mem(n, offs);
    }
}



/*** Cache ops ***/

void dma_clean(dma_mem_t m, vaddr_t vstart, vaddr_t vend)
{
    (void)m;
    (void)vstart;
    (void)vend;
}


void dma_invalidate(dma_mem_t m, vaddr_t vstart, vaddr_t vend)
{
    (void)m;
    (void)vstart;
    (void)vend;
}

void dma_cleaninvalidate(dma_mem_t m, vaddr_t vstart, vaddr_t vend)
{
    (void)m;
    (void)vstart;
    (void)vend;
}

/******** libplatsupport adapter ********/

static void *dma_dma_alloc(void *cookie, size_t size, int align, int cached, ps_mem_flags_t flags)
{
    struct dma_allocator *dalloc;
    vaddr_t vaddr;
    enum dma_flags dma_flags;
    assert(cookie);
    dalloc = (struct dma_allocator *)cookie;

    if (cached) {
        dma_flags = DMAF_COHERENT;
    } else {
        switch (flags) {
        case PS_MEM_NORMAL:
            dma_flags = DMAF_HRW;
            break;
        case PS_MEM_HR:
            dma_flags = DMAF_HR;
            break;
        case PS_MEM_HW:
            dma_flags = DMAF_HW;
            break;
        default:
            dma_flags = DMAF_COHERENT;
        }
    }

    vaddr = dma_alloc(dalloc, size, align, dma_flags, NULL);
    return vaddr;
}

static void dma_dma_free(void *cookie, void *addr, size_t size)
{
    struct dma_allocator *dalloc;
    assert(cookie);
    dalloc = (struct dma_allocator *)cookie;
    dma_free(dma_vlookup(dalloc, addr));
}


static uintptr_t dma_dma_pin(void *cookie, void *addr, size_t size)
{
    struct dma_allocator *dalloc;
    assert(cookie);
    /* DMA memory is pinned when allocated */
    dalloc = (struct dma_allocator *)cookie;
    return dma_paddr(dma_vlookup(dalloc, addr));
}

static void dma_dma_unpin(void *cookie UNUSED, void *addr UNUSED, size_t size UNUSED)
{
    /* DMA memory is unpinned when freed */
}


int dma_dmaman_init(dma_morecore_fn morecore, ps_dma_cache_op_fn_t cache_ops,
                    ps_dma_man_t *dma_man)
{
    struct dma_allocator *dalloc;
    assert(dma_man);

    dalloc = dma_allocator_init(morecore);
    if (dalloc != NULL) {
        dma_man->cookie = dalloc;
        dma_man->dma_cache_op_fn = cache_ops;
        dma_man->dma_alloc_fn = &dma_dma_alloc;
        dma_man->dma_free_fn = &dma_dma_free;
        dma_man->dma_pin_fn = &dma_dma_pin;
        dma_man->dma_unpin_fn = &dma_dma_unpin;
        return 0;
    } else {
        return -1;
    }
}


