# Support fans that are enabled when a heater is on
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100

class PrinterHeaterFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.load_object(config, 'heaters')
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.heaters = []
        self.fan = fan.Fan(config, default_shutdown_speed=1.)
        self.fan_speed = config.getfloat("fan_speed", 1., minval=0., maxval=1.)
        self.min_speed = config.getfloat("min_speed", 0., minval=0., maxval=1.)
        self.probe_speed = config.getfloat("probe_speed", self.fan_speed, minval=0., maxval=1.)
        self.temp_speed_table = config.getlists('temp_speed_table', None, seps=(',', '\n'), count=5, parser=float)
        self.original_fan_speed = None
        self.last_speed = 0.
        self.fan_timer = None

        # Register SET_HEATER_FAN command
        gcode = self.printer.lookup_object('gcode')
        self.fan_name = config.get_name().split()[1]
        gcode.register_mux_command("SET_HEATER_FAN", "FAN", self.fan_name,
                                 self.cmd_SET_HEATER_FAN,
                                 desc=self.cmd_SET_HEATER_FAN_help)
        gcode.register_mux_command("SET_PROBE_FAN", "FAN", self.fan_name,
                                 self.cmd_SET_PROBE_FAN,
                                 desc=self.cmd_SET_PROBE_FAN_help)
        gcode.register_mux_command("RESTORE_FAN", "FAN", self.fan_name,
                                 self.cmd_RESTORE_FAN,
                                 desc=self.cmd_RESTORE_FAN_help)
        self.printer.register_event_handler("inductance_coil:probe_start", self._handle_probe_start)
        self.printer.register_event_handler("inductance_coil:probe_end", self._handle_probe_end)
    def handle_ready(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        reactor = self.printer.get_reactor()
        self.fan_timer = reactor.register_timer(self.callback, reactor.monotonic()+PIN_MIN_TIME)
    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)
    def set_probe_speed(self):
        if self.original_fan_speed is None:
            self.original_fan_speed = self.fan_speed
        self.fan_speed = self.probe_speed
        self.last_speed = -1  # Force update in next callback
        if self.fan_timer is not None:
            self.reactor.update_timer(self.fan_timer, self.reactor.NOW)
    def restore_fan_speed(self):
        if self.original_fan_speed is not None:
            self.fan_speed = self.original_fan_speed
            self.original_fan_speed = None
            self.last_speed = -1  # Force update in next callback
            if self.fan_timer is not None:
                self.reactor.update_timer(self.fan_timer, self.reactor.NOW)
    def callback(self, eventtime):
        speed = 0.
        if self.temp_speed_table is not None:
            for rule in self.temp_speed_table:
                temp_threshold, target_temp_threshold, satisfied_heater_threshold, heater_count_threshold, rule_speed = rule
                satisfied_heaters = 0
                heater_count = 0
                for heater in self.heaters:
                    current_temp, target_temp = heater.get_temp(eventtime)
                    if target_temp > 0:
                        heater_count += 1
                    if current_temp > temp_threshold or target_temp > target_temp_threshold:
                        satisfied_heaters += 1

                if satisfied_heaters >= satisfied_heater_threshold and heater_count >= heater_count_threshold:
                    speed = max(self.min_speed, min(rule_speed, 1.0))
                    break
        else:
            for heater in self.heaters:
                current_temp, target_temp = heater.get_temp(eventtime)
                if target_temp > self.heater_temp or current_temp > self.heater_temp:
                    speed = max(self.min_speed, min(self.fan_speed, 1.0))
        if speed != self.last_speed:
            self.last_speed = speed
            curtime = self.printer.get_reactor().monotonic()
            print_time = self.fan.get_mcu().estimated_print_time(curtime)
            self.fan.set_speed(print_time + PIN_MIN_TIME, speed)
        return eventtime + 1.
    def _handle_probe_start(self):
        # Get current extruder name
        cur_extruder_name = self.printer.lookup_object('toolhead').get_extruder().get_name()
        if cur_extruder_name in self.heater_names:
            self.set_probe_speed()
    def _handle_probe_end(self):
        cur_extruder_name = self.printer.lookup_object('toolhead').get_extruder().get_name()
        if cur_extruder_name in self.heater_names:
            self.restore_fan_speed()
    cmd_SET_HEATER_FAN_help = "Set the speed of a heater fan (0.0 to 1.0)"
    def cmd_SET_HEATER_FAN(self, gcmd):
        speed = gcmd.get_float('SPEED', minval=0., maxval=1.)
        if speed > 0 and speed < self.min_speed:
            gcmd.respond_info("Error: Speed cannot be below minimum speed of %.0f%%"
                           % (self.min_speed * 100,))
            return
        self.fan_speed = speed
        self.last_speed = -1  # Force update in next callback
        if self.fan_timer is not None:
            self.reactor.update_timer(self.fan_timer, self.reactor.NOW)
        gcmd.respond_info("%s speed set to %.0f%%" % (self.fan_name, speed * 100,))
    cmd_SET_PROBE_FAN_help = "Set fan speed for probing"
    def cmd_SET_PROBE_FAN(self, gcmd):
        """Set fan speed for probing"""
        self.set_probe_speed()
        gcmd.respond_info("%s probe speed set to %.0f%%" % (
            self.fan_name, self.probe_speed * 100))
    cmd_RESTORE_FAN_help = "Restore original fan speed"
    def cmd_RESTORE_FAN(self, gcmd):
        """Restore original fan speed"""
        self.restore_fan_speed()
        gcmd.respond_info("%s speed restored to %.0f%%" % (
            self.fan_name, self.fan_speed * 100))

def load_config_prefix(config):
    return PrinterHeaterFan(config)
