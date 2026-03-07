// Commands for sending messages to a GU126x64D VFD display
//
// Copyright (C) 2025
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_from_us
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN

struct gu126x64d {
    struct gpio_out sck, sin, ss, hb;
    struct gpio_in mb;
    uint32_t delay_ticks;
    uint8_t flags;
};

#define GU126X64D_LSB_FIRST   0x01
#define GU126X64D_FALLING_EDGE 0x02


/****************************************************************
 * Transmit functions
 ****************************************************************/

static __always_inline uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static void
ndelay(uint32_t ticks)
{
    if (CONFIG_MACH_AVR)
        // Slower MCUs don't require a delay
        return;
    uint32_t end = timer_read_time() + ticks;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Wait for MB to reach the requested level, with fallback timeout.
// Current debug hypothesis: MB=0 busy, MB=1 ready.
static void
gu126x64d_wait_mb(struct gu126x64d *g, uint8_t level)
{
    uint32_t end = timer_read_time() + g->delay_ticks;
    while (gpio_in_read(g->mb) != level) {
        if (!timer_is_before(timer_read_time(), end))
            break; // Timeout — continue for robustness while debugging
        irq_poll();
    }
}

static __always_inline void
gu126x64d_wait_ready(struct gu126x64d *g)
{
    gu126x64d_wait_mb(g, 1);
}

static __always_inline void
gu126x64d_wait_busy(struct gu126x64d *g)
{
    gu126x64d_wait_mb(g, 0);
}

static __always_inline void
gu126x64d_ss_assert(struct gu126x64d *g, uint32_t delay)
{
    gpio_out_write(g->sck, (g->flags & GU126X64D_FALLING_EDGE) ? 1 : 0);
    gpio_out_write(g->ss, 0);
    ndelay(delay);
}

static __always_inline void
gu126x64d_ss_deassert(struct gu126x64d *g, uint32_t delay)
{
    ndelay(delay);
    gpio_out_write(g->ss, 1);
}

// Clock out one raw byte with /SS already asserted. The bit order and active
// sampling edge are configurable to allow quick testing against module setup.
static void
gu126x64d_xmit_raw_byte(struct gu126x64d *g, uint8_t data)
{
    struct gpio_out sck = g->sck, sin = g->sin;
    uint32_t delay = nsecs_to_ticks(1000);
    uint8_t falling = !!(g->flags & GU126X64D_FALLING_EDGE);
    uint8_t lsb_first = !!(g->flags & GU126X64D_LSB_FIRST);

    gu126x64d_wait_ready(g);
    uint8_t i;
    for (i = 0; i < 8; i++) {
        gpio_out_write(sin, lsb_first ? (data & 1) : ((data >> 7) & 1));
        if (lsb_first)
            data >>= 1;
        else
            data <<= 1;
        ndelay(delay);
        if (falling) {
            gpio_out_write(sck, 0); // Falling edge — data latched by display
            ndelay(delay);
            gpio_out_write(sck, 1); // Return to idle high
        } else {
            gpio_out_write(sck, 1); // Rising edge — data latched by display
            ndelay(delay);
            gpio_out_write(sck, 0); // Return to idle low
        }
    }
    // The reference implementation waits for the module to acknowledge each
    // byte by asserting busy after the transfer.
    gu126x64d_wait_busy(g);
}

// Transmit one full command packet while keeping /SS low across the entire
// packet. Bytes are sent raw; any higher-level hex encoding is handled by the
// host code.
static void
gu126x64d_xmit(struct gu126x64d *g, uint8_t len, uint8_t *data)
{
    if (!len)
        return;
    uint32_t delay = nsecs_to_ticks(1000);
    gu126x64d_ss_assert(g, delay);
    while (len--) {
        uint8_t d = *data++;
        gu126x64d_xmit_raw_byte(g, d);
    }
    gu126x64d_ss_deassert(g, delay);
}


/****************************************************************
 * Interface
 ****************************************************************/

void
command_config_gu126x64d(uint32_t *args)
{
    struct gu126x64d *g = oid_alloc(args[0], command_config_gu126x64d,
                                    sizeof(*g));
    g->flags = args[7];
    g->sck = gpio_out_setup(args[1],
                            (g->flags & GU126X64D_FALLING_EDGE) ? 1 : 0);
    g->ss = gpio_out_setup(args[2], 1);  // /SS idle high
    g->sin = gpio_out_setup(args[3], 0);
    g->mb = gpio_in_setup(args[4], 0);
    g->hb = gpio_out_setup(args[5], 0);  // HB LOW = host ready
    g->delay_ticks = args[6];
}
DECL_COMMAND(command_config_gu126x64d,
             "config_gu126x64d oid=%c sck_pin=%u ss_pin=%u sin_pin=%u"
             " mb_pin=%u hb_pin=%u delay_ticks=%u flags=%u");

void
command_gu126x64d_send_cmds(uint32_t *args)
{
    struct gu126x64d *g = oid_lookup(args[0], command_config_gu126x64d);
    uint8_t len = args[1], *cmds = command_decode_ptr(args[2]);
    gu126x64d_xmit(g, len, cmds);
}
DECL_COMMAND(command_gu126x64d_send_cmds,
             "gu126x64d_send_cmds oid=%c cmds=%*s");

void
command_gu126x64d_send_data(uint32_t *args)
{
    struct gu126x64d *g = oid_lookup(args[0], command_config_gu126x64d);
    uint8_t len = args[1], *data = command_decode_ptr(args[2]);
    gu126x64d_xmit(g, len, data);
}
DECL_COMMAND(command_gu126x64d_send_data,
             "gu126x64d_send_data oid=%c data=%*s");

void
gu126x64d_shutdown(void)
{
    uint8_t i;
    struct gu126x64d *g;
    foreach_oid(i, g, command_config_gu126x64d) {
        gpio_out_write(g->sck,
                       (g->flags & GU126X64D_FALLING_EDGE) ? 1 : 0);
        gpio_out_write(g->sin, 0);
        gpio_out_write(g->ss, 0);
        gpio_out_write(g->hb, 0);
    }
}
DECL_SHUTDOWN(gu126x64d_shutdown);
