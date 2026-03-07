# Support for GU126x64D VFD (Vacuum Fluorescent Display)
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from .. import bus
from . import font8x14

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

# Busy delays used by the MCU C driver (for delay_ticks calculation)
GU126X64D_CMD_DELAY = .000050   # 50us per command byte
GU126X64D_DATA_DELAY = .000250  # 250us per data byte

TextGlyphs = { 'right_arrow': b'\x1a', 'degrees': b'\xf8' }


class GU126X64D:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        # Resolve all pins and ensure they are on the same MCU
        pin_names = ['sck_pin', 'ss_pin', 'sin_pin', 'mb_pin', 'hb_pin']
        pins = [ppins.lookup_pin(config.get(name)) for name in pin_names]
        mcu = None
        for pin_params in pins:
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("gu126x64d all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = [pin_params['pin'] for pin_params in pins]
        self.mcu = mcu
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.send_cmds_cmd = self.send_data_cmd = None
        # Reset pin (optional, independent of the SPI driver)
        self.rst_pin = config.get("rst_pin", None)
        # Display settings
        self.brightness = config.getint('brightness', 7, minval=1, maxval=8)
        # Framebuffer: 8 pages × 128 columns (only 126 sent to display)
        self.columns = 128
        self.vram = [bytearray(self.columns) for i in range(8)]
        self.all_framebuffers = [(self.vram[i], bytearray(b'~' * self.columns),
                                  i) for i in range(8)]
        # Cache fonts and icons in display byte order
        self.font = [self._swizzle_bits(bytearray(c))
                     for c in font8x14.VGA_FONT]
        self.icons = {}
    def build_config(self):
        self.mcu.add_config_cmd(
            "config_gu126x64d oid=%d sck_pin=%s ss_pin=%s sin_pin=%s"
            " mb_pin=%s hb_pin=%s delay_ticks=%d" % (
                self.oid, self.pins[0], self.pins[1], self.pins[2],
                self.pins[3], self.pins[4],
                self.mcu.seconds_to_clock(GU126X64D_DATA_DELAY)))
        cmd_queue = self.mcu.alloc_command_queue()
        self.send_cmds_cmd = self.mcu.lookup_command(
            "gu126x64d_send_cmds oid=%c cmds=%*s", cq=cmd_queue)
        self.send_data_cmd = self.mcu.lookup_command(
            "gu126x64d_send_data oid=%c data=%*s", cq=cmd_queue)
        # Setup reset pin helper using the command queue
        self._setup_reset(cmd_queue)
        # Register G-Code command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_DISPLAY_BRIGHTNESS',
                               self.cmd_SET_DISPLAY_BRIGHTNESS,
                               desc=self.cmd_SET_DISPLAY_BRIGHTNESS_help)
    def _setup_reset(self, cmd_queue):
        self.mcu_reset = None
        if self.rst_pin is None:
            return
        self.mcu_reset = bus.MCU_bus_digital_out(
            self.mcu, self.rst_pin, cmd_queue)
    def init(self):
        # Hardware reset via /RES pin
        if self.mcu_reset is not None:
            curtime = self.printer.get_reactor().monotonic()
            print_time = self.mcu.estimated_print_time(curtime)
            # Toggle reset: low for 100ms, then high, wait 300ms
            minclock = self.mcu.print_time_to_clock(print_time + .100)
            self.mcu_reset.update_digital_out(0, minclock=minclock)
            minclock = self.mcu.print_time_to_clock(print_time + .200)
            self.mcu_reset.update_digital_out(1, minclock=minclock)
            minclock = self.mcu.print_time_to_clock(print_time + .300)
            self.mcu_reset.update_digital_out(1, minclock=minclock)
        # Software Reset (0x19)
        self.send_cmds_cmd.send([self.oid, [0x19]],
                                reqclock=BACKGROUND_PRIORITY_CLOCK)
        # Write Mode: 0x1A, 0x80 (vertical orientation, horizontal cursor)
        self.send_cmds_cmd.send([self.oid, [0x1A, 0x80]],
                                reqclock=BACKGROUND_PRIORITY_CLOCK)
        # Brightness: 0x1B, 0xF8 + brightness (1-8 → 0xF9-0xFF)
        self.send_cmds_cmd.send([self.oid, [0x1B, 0xF8 + self.brightness]],
                                reqclock=BACKGROUND_PRIORITY_CLOCK)
        # Clear display area
        self.send_cmds_cmd.send([self.oid, [0x12, 0, 0, 125, 63]],
                                reqclock=BACKGROUND_PRIORITY_CLOCK)
        self.flush()
    def flush(self):
        # Differential update — only send changed regions per page
        for new_data, old_data, page in self.all_framebuffers:
            if new_data == old_data:
                continue
            # Only compare columns 0-125 (126 pixel width)
            diffs = [[i, 1] for i in range(126)
                     if new_data[i] != old_data[i]]
            if not diffs:
                old_data[:] = new_data
                continue
            # Batch together changes that are close to each other
            for i in range(len(diffs) - 2, -1, -1):
                pos, count = diffs[i]
                nextpos, nextcount = diffs[i + 1]
                if pos + 5 >= nextpos and nextcount < 16:
                    diffs[i][1] = nextcount + (nextpos - pos)
                    del diffs[i + 1]
            # Transmit changes
            for col_pos, count in diffs:
                y = page * 8
                packet = [0x10, col_pos, y, 0x18, count]
                packet.extend(new_data[col_pos:col_pos + count])
                self.send_cmds_cmd.send(
                    [self.oid, packet],
                    reqclock=BACKGROUND_PRIORITY_CLOCK)
            old_data[:] = new_data
    # Framebuffer methods (same as uc1701.DisplayBase)
    def _swizzle_bits(self, data):
        # Convert from "rows of pixels" to "columns of pixels"
        # GU126x64D uses Bit7=top, Bit0=bottom in vertical write mode.
        top = bot = 0
        for row in range(8):
            spaced = (data[row] * 0x8040201008040201) & 0x8080808080808080
            top |= spaced >> row
            spaced = (data[row + 8] * 0x8040201008040201) & 0x8080808080808080
            bot |= spaced >> row
        bits_top = [(top >> s) & 0xff for s in range(0, 64, 8)]
        bits_bot = [(bot >> s) & 0xff for s in range(0, 64, 8)]
        return (bytearray(bits_top), bytearray(bits_bot))
    def set_glyphs(self, glyphs):
        for glyph_name, glyph_data in glyphs.items():
            icon = glyph_data.get('icon16x16')
            if icon is not None:
                top1, bot1 = self._swizzle_bits(icon[0])
                top2, bot2 = self._swizzle_bits(icon[1])
                self.icons[glyph_name] = (top1 + top2, bot1 + bot2)
    def write_text(self, x, y, data):
        if x + len(data) > 16:
            data = data[:16 - min(x, 16)]
        pix_x = x * 8
        page_top = self.vram[y * 2]
        page_bot = self.vram[y * 2 + 1]
        for c in bytearray(data):
            bits_top, bits_bot = self.font[c]
            page_top[pix_x:pix_x + 8] = bits_top
            page_bot[pix_x:pix_x + 8] = bits_bot
            pix_x += 8
    def write_graphics(self, x, y, data):
        if x >= 16 or y >= 4 or len(data) != 16:
            return
        bits_top, bits_bot = self._swizzle_bits(data)
        pix_x = x * 8
        page_top = self.vram[y * 2]
        page_bot = self.vram[y * 2 + 1]
        for i in range(8):
            page_top[pix_x + i] ^= bits_top[i]
            page_bot[pix_x + i] ^= bits_bot[i]
    def write_glyph(self, x, y, glyph_name):
        icon = self.icons.get(glyph_name)
        if icon is not None and x < 15:
            pix_x = x * 8
            page_idx = y * 2
            self.vram[page_idx][pix_x:pix_x + 16] = icon[0]
            self.vram[page_idx + 1][pix_x:pix_x + 16] = icon[1]
            return 2
        char = TextGlyphs.get(glyph_name)
        if char is not None:
            self.write_text(x, y, char)
            return 1
        return 0
    def clear(self):
        zeros = bytearray(self.columns)
        for page in self.vram:
            page[:] = zeros
    def get_dimensions(self):
        return (16, 4)
    # Brightness G-Code
    def _set_brightness(self, brightness):
        self.brightness = brightness
        self.send_cmds_cmd.send(
            [self.oid, [0x1B, 0xF8 + brightness]],
            reqclock=BACKGROUND_PRIORITY_CLOCK)
    cmd_SET_DISPLAY_BRIGHTNESS_help = "Set VFD display brightness (1-8)"
    def cmd_SET_DISPLAY_BRIGHTNESS(self, gcmd):
        brightness = gcmd.get_int('BRIGHTNESS', minval=1, maxval=8)
        self._set_brightness(brightness)
