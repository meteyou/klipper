// Commands for sending messages to a GU128x64D VFD display
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

struct gu128x64d {
    struct gpio_out sck, sin, ss, hb;
    struct gpio_in mb;
    uint32_t delay_ticks;
    uint8_t flags;
};

#define GU128X64D_LSB_FIRST    0x01
#define GU128X64D_FALLING_EDGE 0x02


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

// Wait for MB (Module Busy) to reach the requested level, with timeout.
// Per GU128x64D datasheet (section 5.3.3 SPI timing):
//   MB HIGH (1) = module ready / idle
//   MB LOW  (0) = module busy processing
static void
gu128x64d_wait_mb(struct gu128x64d *g, uint8_t level)
{
    uint32_t end = timer_read_time() + g->delay_ticks;
    while (gpio_in_read(g->mb) != level) {
        if (!timer_is_before(timer_read_time(), end))
            break; // Timeout — proceed to avoid lockup
        irq_poll();
    }
}

static __always_inline void
gu128x64d_wait_ready(struct gu128x64d *g)
{
    gu128x64d_wait_mb(g, 1);
}

static __always_inline void
gu128x64d_wait_busy(struct gu128x64d *g)
{
    gu128x64d_wait_mb(g, 0);
}

static __always_inline void
gu128x64d_ss_assert(struct gu128x64d *g, uint32_t delay)
{
    gpio_out_write(g->sck, (g->flags & GU128X64D_FALLING_EDGE) ? 1 : 0);
    gpio_out_write(g->ss, 0);
    ndelay(delay);
}

static __always_inline void
gu128x64d_ss_deassert(struct gu128x64d *g, uint32_t delay)
{
    ndelay(delay);
    gpio_out_write(g->ss, 1);
}

// Clock out one raw byte with /SS already asserted.  The bit order and
// active sampling edge are configurable via the flags field.
static void
gu128x64d_xmit_raw_byte(struct gu128x64d *g, uint8_t data)
{
    struct gpio_out sck = g->sck, sin = g->sin;
    uint32_t delay = nsecs_to_ticks(1000);
    uint8_t falling = !!(g->flags & GU128X64D_FALLING_EDGE);
    uint8_t lsb_first = !!(g->flags & GU128X64D_LSB_FIRST);

    gu128x64d_wait_ready(g);
    uint8_t i;
    for (i = 0; i < 8; i++) {
        gpio_out_write(sin, lsb_first ? (data & 1) : ((data >> 7) & 1));
        if (lsb_first)
            data >>= 1;
        else
            data <<= 1;
        ndelay(delay);
        if (falling) {
            gpio_out_write(sck, 0); // Falling edge — data latched
            ndelay(delay);
            gpio_out_write(sck, 1); // Return to idle high
        } else {
            gpio_out_write(sck, 1); // Rising edge — data latched
            ndelay(delay);
            gpio_out_write(sck, 0); // Return to idle low
        }
    }
    // Wait for busy acknowledgement after each byte (per datasheet timing)
    gu128x64d_wait_busy(g);
}

// Transmit a complete command/data packet with /SS held low for the
// entire duration.
static void
gu128x64d_xmit(struct gu128x64d *g, uint8_t len, uint8_t *data)
{
    if (!len)
        return;
    uint32_t delay = nsecs_to_ticks(1000);
    gu128x64d_ss_assert(g, delay);
    while (len--) {
        uint8_t d = *data++;
        gu128x64d_xmit_raw_byte(g, d);
    }
    gu128x64d_ss_deassert(g, delay);
}


/****************************************************************
 * Interface
 ****************************************************************/

void
command_config_gu128x64d(uint32_t *args)
{
    struct gu128x64d *g = oid_alloc(args[0], command_config_gu128x64d,
                                    sizeof(*g));
    g->flags = args[7];
    g->sck = gpio_out_setup(args[1],
                            (g->flags & GU128X64D_FALLING_EDGE) ? 1 : 0);
    g->ss = gpio_out_setup(args[2], 1);  // /SS idle high
    g->sin = gpio_out_setup(args[3], 0);
    g->mb = gpio_in_setup(args[4], 0);
    g->hb = gpio_out_setup(args[5], 0);  // HB LOW = host ready
    g->delay_ticks = args[6];
}
DECL_COMMAND(command_config_gu128x64d,
             "config_gu128x64d oid=%c sck_pin=%u ss_pin=%u sin_pin=%u"
             " mb_pin=%u hb_pin=%u delay_ticks=%u flags=%u");

void
command_gu128x64d_send_cmds(uint32_t *args)
{
    struct gu128x64d *g = oid_lookup(args[0], command_config_gu128x64d);
    uint8_t len = args[1], *cmds = command_decode_ptr(args[2]);
    gu128x64d_xmit(g, len, cmds);
}
DECL_COMMAND(command_gu128x64d_send_cmds,
             "gu128x64d_send_cmds oid=%c cmds=%*s");

void
command_gu128x64d_send_data(uint32_t *args)
{
    struct gu128x64d *g = oid_lookup(args[0], command_config_gu128x64d);
    uint8_t len = args[1], *data = command_decode_ptr(args[2]);
    gu128x64d_xmit(g, len, data);
}
DECL_COMMAND(command_gu128x64d_send_data,
             "gu128x64d_send_data oid=%c data=%*s");

void
gu128x64d_shutdown(void)
{
    uint8_t i;
    struct gu128x64d *g;
    foreach_oid(i, g, command_config_gu128x64d) {
        gpio_out_write(g->sck,
                       (g->flags & GU128X64D_FALLING_EDGE) ? 1 : 0);
        gpio_out_write(g->sin, 0);
        gpio_out_write(g->ss, 1);  // /SS idle high — release SPI bus
        gpio_out_write(g->hb, 0);
    }
}
DECL_SHUTDOWN(gu128x64d_shutdown);
