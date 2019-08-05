/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

#define not_pending(...) !is_pending(__VA_ARGS__)
#define not_active(...)  !is_active(__VA_ARGS__)
#define not_enabled(...) !is_enabled(__VA_ARGS__)

enum gic_dist_action {
    ACTION_READONLY,
    ACTION_PASSTHROUGH,
    ACTION_ENABLE,
    ACTION_ENABLE_SET,
    ACTION_ENABLE_CLR,
    ACTION_PENDING_SET,
    ACTION_PENDING_CLR,
    ACTION_SGI,
    ACTION_UNKNOWN
};

static inline enum gic_dist_action gic_dist_get_action(int offset)
{
    /* Handle the fault
     * The only fields we care about are enable_set/clr
     * We have 2 options for other registers:
     *  a) ignore writes and hope the VM acts appropriately
     *  b) allow write access so the VM thinks there is no problem,
     *     but do not honour them
     */
    if (0x000 <= offset && offset < 0x004) {     /* enable          */
        return ACTION_ENABLE;
    } else if (0x080 <= offset && offset < 0x100) { /* Security        */
        return ACTION_PASSTHROUGH;
    } else if (0x100 <= offset && offset < 0x180) { /* enable_set      */
        return ACTION_ENABLE_SET;
    } else if (0x180 <= offset && offset < 0x200) { /* enable_clr      */
        return ACTION_ENABLE_CLR;
    } else if (0x200 <= offset && offset < 0x280) { /* pending_set     */
        return ACTION_PENDING_SET;
    } else if (0x280 <= offset && offset < 0x300) { /* pending_clr     */
        return ACTION_PENDING_CLR;
    } else if (0x300 <= offset && offset < 0x380) { /* active          */
        return ACTION_READONLY;
    } else if (0x400 <= offset && offset < 0x7FC) { /* priority        */
        return ACTION_READONLY;
    } else if (0x800 <= offset && offset < 0x8FC) { /* targets         */
        return ACTION_READONLY;
    } else if (0xC00 <= offset && offset < 0xD00) { /* config          */
        return ACTION_READONLY;
    } else if (0xD00 <= offset && offset < 0xD80) { /* spi config      */
        return ACTION_READONLY;
    } else if (0xDD4 <= offset && offset < 0xDD8) { /* legacy_int      */
        return ACTION_READONLY;
    } else if (0xDE0 <= offset && offset < 0xDE4) { /* match_d         */
        return ACTION_READONLY;
    } else if (0xDE4 <= offset && offset < 0xDE8) { /* enable_d        */
        return ACTION_READONLY;
    } else if (0xF00 <= offset && offset < 0xF04) { /* sgi_control     */
        return ACTION_PASSTHROUGH;
    } else if (0xF10 <= offset && offset < 0xF10) { /* sgi_pending_clr */
        return ACTION_SGI;
    } else {
        return ACTION_READONLY;
    }
    return ACTION_UNKNOWN;
}

extern const struct device dev_vgic_dist;
