# Support for GU128x64D VFD (Vacuum Fluorescent Display)
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from .. import bus
from . import font8x14

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

# MB timeout for the MCU C driver (delay_ticks calculation).
# Most per-byte busy times are <50us; 1ms provides ample margin.
GU128X64D_MB_TIMEOUT = .001000

TextGlyphs = { 'right_arrow': b'\x1a', 'degrees': b'\xf8' }

TEST_PATTERNS = {
    'solid', 'vertical_stripes', 'horizontal_stripes', 'checker',
    'topbit', 'botbit', 'pagebytes', 'font'
}


class GU128X64D:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        # Resolve all pins and ensure they are on the same MCU
        pin_names = ['sck_pin', 'ss_pin', 'sin_pin', 'mb_pin', 'hb_pin']
        pins = [ppins.lookup_pin(config.get(name)) for name in pin_names]
        mcu = None
        for pin_params in pins:
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("gu128x64d all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = [pin_params['pin'] for pin_params in pins]
        self.mcu = mcu
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.send_cmds_cmd = self.send_data_cmd = None
        # Reset pin (optional, independent of the SPI driver)
        self.rst_pin = config.get("rst_pin", None)
        # Display settings
        self.brightness = config.getint('brightness', 7, minval=1, maxval=7)
        bit_order_map = {'msb_first': 0, 'lsb_first': 1}
        edge_map = {'rising': 0, 'falling': 2}
        self.spi_flags = (
            config.getchoice('spi_bit_order', bit_order_map,
                             default='msb_first')
            | config.getchoice('spi_clock_edge', edge_map,
                               default='rising'))
        # Framebuffer: 8 pages × 128 columns
        self.columns = 128
        self.vram = [bytearray(self.columns) for i in range(8)]
        self.all_framebuffers = [(self.vram[i], bytearray(b'~' * self.columns),
                                  i) for i in range(8)]
        # Cache fonts and icons in display byte order
        self.font = [self._swizzle_bits(bytearray(c))
                     for c in font8x14.VGA_FONT]
        self.icons = {}
        self.test_pattern = None
    def build_config(self):
        self.mcu.add_config_cmd(
            "config_gu128x64d oid=%d sck_pin=%s ss_pin=%s sin_pin=%s"
            " mb_pin=%s hb_pin=%s delay_ticks=%d flags=%d" % (
                self.oid, self.pins[0], self.pins[1], self.pins[2],
                self.pins[3], self.pins[4],
                self.mcu.seconds_to_clock(GU128X64D_MB_TIMEOUT),
                self.spi_flags))
        cmd_queue = self.mcu.alloc_command_queue()
        self.send_cmds_cmd = self.mcu.lookup_command(
            "gu128x64d_send_cmds oid=%c cmds=%*s", cq=cmd_queue)
        self.send_data_cmd = self.mcu.lookup_command(
            "gu128x64d_send_data oid=%c data=%*s", cq=cmd_queue)
        # Setup reset pin helper using the command queue
        self._setup_reset(cmd_queue)
        # Register G-Code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_DISPLAY_BRIGHTNESS',
                               self.cmd_SET_DISPLAY_BRIGHTNESS,
                               desc=self.cmd_SET_DISPLAY_BRIGHTNESS_help)
        gcode.register_command('SET_GU_DISPLAY_TEST_PATTERN',
                               self.cmd_SET_GU_DISPLAY_TEST_PATTERN,
                               desc=self.cmd_SET_GU_DISPLAY_TEST_PATTERN_help)
    def _setup_reset(self, cmd_queue):
        self.mcu_reset = None
        if self.rst_pin is None:
            return
        self.mcu_reset = bus.MCU_bus_digital_out(
            self.mcu, self.rst_pin, cmd_queue)
    def _request_redraw(self):
        display = self.printer.lookup_object('display', None)
        if display is not None:
            display.request_redraw()
    def _encode_hex_bytes(self, cmds):
        """Encode raw bytes into the GU128x64D hex-receive format.
        Each byte B is sent as: 0x60, ASCII_HI_NIBBLE, ASCII_LO_NIBBLE."""
        out = []
        for b in bytearray(cmds):
            out.extend((0x60, ord('%X' % (b >> 4)), ord('%X' % (b & 0x0f))))
        return out
    def _send_raw(self, cmds, minclock=0,
                  reqclock=BACKGROUND_PRIORITY_CLOCK):
        self.send_cmds_cmd.send([self.oid, cmds], minclock=minclock,
                                reqclock=reqclock)
    def _send_hex(self, cmds, minclock=0,
                  reqclock=BACKGROUND_PRIORITY_CLOCK):
        self.send_cmds_cmd.send(
            [self.oid, self._encode_hex_bytes(cmds)],
            minclock=minclock, reqclock=reqclock)
    def _set_test_pattern(self, pattern):
        self.test_pattern = pattern
        self._request_redraw()
    def _write_text_to_vram(self, x, y, data):
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
    def _write_graphics_to_vram(self, x, y, data):
        if x >= 16 or y >= 4 or len(data) != 16:
            return
        bits_top, bits_bot = self._swizzle_bits(data)
        pix_x = x * 8
        page_top = self.vram[y * 2]
        page_bot = self.vram[y * 2 + 1]
        for i in range(8):
            page_top[pix_x + i] ^= bits_top[i]
            page_bot[pix_x + i] ^= bits_bot[i]
    def _fill_test_pattern(self):
        for page_idx, page in enumerate(self.vram):
            for col in range(self.columns):
                if self.test_pattern == 'solid':
                    val = 0xff
                elif self.test_pattern == 'vertical_stripes':
                    val = 0xff if (col & 1) else 0x00
                elif self.test_pattern == 'horizontal_stripes':
                    val = 0xaa
                elif self.test_pattern == 'checker':
                    val = 0xaa if (col & 1) else 0x55
                elif self.test_pattern == 'topbit':
                    val = 0x80
                elif self.test_pattern == 'botbit':
                    val = 0x01
                elif self.test_pattern == 'pagebytes':
                    val = (0x80, 0x01, 0xaa, 0x55)[page_idx & 3]
                else:
                    val = 0x00
                page[col] = val
        if self.test_pattern == 'font':
            self._write_text_to_vram(0, 0, b'0123456789ABCDEF')
            self._write_text_to_vram(0, 1, b'FEDCBA9876543210')
            self._write_text_to_vram(0, 2, b'AaMmWw#@[]{}()<>')
            self._write_text_to_vram(0, 3, b'bit7 top? bit0?')
    def init(self):
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        print_time = self.mcu.estimated_print_time(curtime)
        init_time = print_time
        if self.mcu_reset is not None:
            # Hardware reset: /RES low for 100ms, then high, settle 120ms.
            # Datasheet section 5.4: /RES low >1.5us, then wait >30ms.
            minclock = self.mcu.print_time_to_clock(print_time + .100)
            self.mcu_reset.update_digital_out(0, minclock=minclock)
            minclock = self.mcu.print_time_to_clock(print_time + .200)
            self.mcu_reset.update_digital_out(1, minclock=minclock)
            minclock = self.mcu.print_time_to_clock(print_time + .300)
            self.mcu_reset.update_digital_out(1, minclock=minclock)
            init_time = print_time + .320
        else:
            # No reset pin — send software reset (cmd 0x19, 500ms busy)
            # in hex mode (enabled by default at power-up).
            self._send_hex([0x19],
                           minclock=self.mcu.print_time_to_clock(init_time))
            init_time += .600
        # After reset, hex receive mode is active (datasheet section 6.28).
        # Disable hex mode (1BH+42H) so we can send raw binary commands.
        self._send_hex([0x1B, 0x42],
                       minclock=self.mcu.print_time_to_clock(init_time))
        # Set write mode: vertical graphic data, horizontal cursor advance
        # (datasheet section 6.21, bit 7 = vertical orientation)
        self._send_raw([0x1A, 0x80],
                       minclock=self.mcu.print_time_to_clock(
                           init_time + .010))
        # Set brightness (datasheet section 6.23: F8H=off .. FFH=max)
        self._send_raw([0x1B, 0xF8 + self.brightness],
                       minclock=self.mcu.print_time_to_clock(
                           init_time + .020))
        self.flush()
    def flush(self):
        # Differential update — only send changed regions per page
        for new_data, old_data, page in self.all_framebuffers:
            if new_data == old_data:
                continue
            # Compare all 128 columns
            diffs = [[i, 1] for i in range(self.columns)
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
                # Y coordinate is absolute pixel position (8 pixels per page)
                y = page * 8
                # Set cursor position (cmd 10H + X + Y)
                self._send_raw([0x10, col_pos, y])
                # Graphic write (cmd 18H + length + data)
                packet = [0x18, count]
                packet.extend(new_data[col_pos:col_pos + count])
                self._send_raw(packet)
            old_data[:] = new_data
    # Framebuffer methods
    def _swizzle_bits(self, data):
        # Convert from "rows of pixels" to "columns of pixels".
        # GU128x64D vertical mode: Bit7 = top pixel, Bit0 = bottom pixel
        # (datasheet section 7.4: "MSB is positioned to the top").
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
        if self.test_pattern is not None:
            return
        self._write_text_to_vram(x, y, data)
    def write_graphics(self, x, y, data):
        if self.test_pattern is not None:
            return
        self._write_graphics_to_vram(x, y, data)
    def write_glyph(self, x, y, glyph_name):
        if self.test_pattern is not None:
            return 0
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
        if self.test_pattern is not None:
            self._fill_test_pattern()
    def get_dimensions(self):
        return (16, 4)
    # Brightness G-Code
    def _set_brightness(self, brightness):
        self.brightness = brightness
        self._send_raw([0x1B, 0xF8 + brightness],
                       minclock=self.mcu.print_time_to_clock(
                           self.mcu.estimated_print_time(
                               self.printer.get_reactor().monotonic())
                           + .010))
    cmd_SET_DISPLAY_BRIGHTNESS_help = "Set VFD display brightness (1-7)"
    def cmd_SET_DISPLAY_BRIGHTNESS(self, gcmd):
        brightness = gcmd.get_int('BRIGHTNESS', minval=1, maxval=7)
        self._set_brightness(brightness)
    cmd_SET_GU_DISPLAY_TEST_PATTERN_help = (
        "Enable a GU128x64D test pattern or PATTERN=off to disable")
    def cmd_SET_GU_DISPLAY_TEST_PATTERN(self, gcmd):
        pattern = gcmd.get('PATTERN').strip().lower()
        if pattern == 'off':
            self._set_test_pattern(None)
            gcmd.respond_info('GU128x64D test pattern disabled')
            return
        if pattern not in TEST_PATTERNS:
            raise gcmd.error(
                "Unknown GU128x64D test pattern '%s'" % (pattern,))
        self._set_test_pattern(pattern)
        gcmd.respond_info("GU128x64D test pattern '%s' enabled" % (pattern,))
