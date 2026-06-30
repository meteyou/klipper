# SGP40 I2C-based VOC sensor support
#
# Copyright (C) 2024-2026 Stefan Dej <meteyou@gmail.com>
# Copyright (C) 2022 Adrien Le Masle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Based on the Nevermore sgp40.py code:
# https://github.com/nevermore3d/Nevermore_Max/blob/master/Software/Klipper/

import logging
import math
from . import bus

REPORT_TIME = 1.
SGP40_I2C_ADDR = 0x59
SGP40_GENERAL_CALL_ADDR = 0x00
SGP40_WORD_LEN = 3

SGP40_CMD = {
    "GET_SERIAL": [0x36, 0x82],
    "HEATER_OFF": [0x36, 0x15],
    "SOFT_RESET": [0x06],
    "SELF_TEST": [0x28, 0x0E],
    "MEASURE_RAW": [0x26, 0x0F]
}

_MAGNUS_A = 17.62
_MAGNUS_B = 243.12
_REF_VAPOR_EXP = (_MAGNUS_A * 25.) / (_MAGNUS_B + 25.)

# Constants for the Sensirion Gas Index Algorithm v3.2.0 (VOC)
_VOC_INITIAL_BLACKOUT = 45.
_VOC_INDEX_GAIN = 230.
_VOC_SRAW_STD_INITIAL = 50.
_VOC_SRAW_STD_BONUS = 220.
_VOC_TAU_MEAN_HOURS = 12.
_VOC_TAU_VARIANCE_HOURS = 12.
_VOC_TAU_INITIAL_MEAN = 20.
_VOC_INIT_DURATION_MEAN = 3600. * 0.75
_VOC_INIT_TRANSITION_MEAN = 0.01
_VOC_TAU_INITIAL_VARIANCE = 2500.
_VOC_INIT_DURATION_VARIANCE = 3600. * 1.45
_VOC_INIT_TRANSITION_VARIANCE = 0.01
_VOC_GATING_THRESHOLD = 340.
_VOC_GATING_THRESHOLD_INITIAL = 510.
_VOC_GATING_THRESHOLD_TRANSITION = 0.09
_VOC_GATING_MAX_DURATION_MINUTES = 60. * 3.
_VOC_GATING_MAX_RATIO = 0.3
_VOC_SIGMOID_L = 500.
_VOC_SIGMOID_K = -0.0065
_VOC_SIGMOID_X0 = 213.
_VOC_INDEX_OFFSET_DEFAULT = 100.
_VOC_LP_TAU_FAST = 20.
_VOC_LP_TAU_SLOW = 500.
_VOC_LP_ALPHA = -0.2
_VOC_SRAW_MINIMUM = 20000
_VOC_GAMMA_SCALING = 64.
_VOC_ADDITIONAL_GAMMA_MEAN_SCALING = 8.
_VOC_UPTIME_LIMIT_MAX = 32767.

class SGP40:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=SGP40_I2C_ADDR, default_speed=100000)
        self._error = self.printer.command_error
        self._callback = None
        self.temp_sensor_name = config.get('sgp40_ref_temp_sensor', None)
        self.humidity_sensor_name = config.get(
            'sgp40_ref_humidity_sensor', None)
        self.soft_reset_on_startup = config.getboolean(
            'sgp40_soft_reset_on_startup', False)
        self.i2c_general_call = None
        if self.soft_reset_on_startup:
            sw_pins = None
            if config.get('i2c_software_scl_pin', None) is not None:
                ppins = self.printer.lookup_object("pins")
                sw_pin_names = ['i2c_software_%s_pin' % (name,)
                                for name in ['scl', 'sda']]
                sw_pins = tuple([
                    ppins.lookup_pin(config.get(name), share_type=name)['pin']
                    for name in sw_pin_names])
            self.i2c_general_call = bus.MCU_I2C(
                self.i2c.get_mcu(), self.i2c.bus, SGP40_GENERAL_CALL_ADDR,
                self.i2c.speed, sw_pins)

        self.temp_sensor = None
        self.humidity_sensor = None
        self.temp = 25.
        self.humidity = 50.
        self.min_temp = self.max_temp = 0.
        self.sample_timer = self.reactor.register_timer(self._sample_sgp40)

        self.voc_algorithm = VOCAlgorithm(REPORT_TIME)
        self.raw = self.voc = 0
        self._measuring = False
        self.printer.add_object("sgp40 " + self.name, self)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self.handle_ready)

    def _lookup_ref_sensor(self, sensor_name, field):
        sensor = self.printer.lookup_object(sensor_name)
        if not hasattr(sensor, 'get_status'):
            raise self.printer.config_error(
                "'%s' does not have a status." % (sensor_name,))
        status = sensor.get_status(self.reactor.monotonic())
        if field not in status:
            raise self.printer.config_error(
                "'%s' does not report '%s'." % (sensor_name, field))
        return sensor

    def handle_connect(self):
        if self.temp_sensor_name is not None:
            self.temp_sensor = self._lookup_ref_sensor(
                self.temp_sensor_name, "temperature")
        if self.humidity_sensor_name is not None:
            self.humidity_sensor = self._lookup_ref_sensor(
                self.humidity_sensor_name, "humidity")
        self._init_sgp40()

    def handle_ready(self):
        # Delay start slightly so reference sensors can report initial values
        self.reactor.update_timer(
            self.sample_timer, self.reactor.monotonic() + REPORT_TIME)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _init_sgp40(self):
        logging.info("SGP40 %s: Initializing sensor", self.name)

        if self.soft_reset_on_startup:
            logging.info("SGP40 %s: Performing soft reset", self.name)
            # Data sheet specifies a maximum of 0.6ms for the soft reset
            self._write_command(SGP40_CMD["SOFT_RESET"], delay=.001,
                                i2c=self.i2c_general_call)
        else:
            self._write_command(SGP40_CMD["HEATER_OFF"], delay=.001)

        # Get the serial number to verify communication
        serial = self._write_command(SGP40_CMD["GET_SERIAL"], read_len=3,
                                     delay=.001)
        if serial is None:
            logging.error(
                "SGP40 %s: Failed to read serial number, communication error",
                self.name)
        else:
            serial_str = ''.join(["%04x" % x for x in serial])
            logging.info("SGP40 %s: Serial number: %s", self.name, serial_str)

        # Perform a self-test with increased delay
        logging.info("SGP40 %s: Performing self-test", self.name)
        # Data sheet specifies a maximum of 320ms for the self-test
        self_test_result = self._write_command(SGP40_CMD["SELF_TEST"],
                                               read_len=1, delay=.32)
        if (self_test_result is None
                or (self_test_result[0] & 0xFF00) != 0xD400):
            logging.error("SGP40 %s: Self-test failed", self.name)
        else:
            logging.info("SGP40 %s: Self-test passed successfully", self.name)

    def _sample_sgp40(self, eventtime):
        try:
            # Read the measurement started last cycle. Some sensors need far
            # longer than the datasheet's 30ms, so we don't pause for it and
            # use a full REPORT_TIME interval to wait for it.
            if self._measuring:
                self._measuring = False
                value = self._read(1)
                if self.printer.is_shutdown():
                    return self.reactor.NEVER
                if value is not None:
                    self.raw = value[0]
                    self.voc = self.voc_algorithm.process(self.raw)

            if self.temp_sensor is not None:
                temp = self.temp_sensor.get_status(eventtime).get(
                    "temperature")
                if temp is not None:
                    self.temp = temp

            if self.humidity_sensor is not None:
                humidity = self.humidity_sensor.get_status(eventtime).get(
                    "humidity")
                if humidity is not None:
                    self.humidity = humidity
            elif self.temp_sensor is not None:
                # calculate humidity from temperature
                self.humidity = self._calculate_humidity(self.temp)
            else:
                self.humidity = 50.

            # Start the next measurement to read it at the start of next cycle
            cmd = (SGP40_CMD["MEASURE_RAW"]
                   + self._humidity_to_ticks(self.humidity)
                   + self._temperature_to_ticks(self.temp))
            self.i2c.i2c_write(cmd)
            if self.printer.is_shutdown():
                return self.reactor.NEVER
            self._measuring = True
        except Exception:
            logging.exception("SGP40 %s: Error reading data", self.name)
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "SGP40 temperature %0.1f outside range of %0.1f:%0.1f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        if self._callback is not None:
            self._callback(print_time, self.temp)
        return max(measured_time + .1, eventtime + REPORT_TIME)

    def _write_command(self, command, delay=0., read_len=0, i2c=None):
        if i2c is None:
            i2c = self.i2c
        try:
            i2c.i2c_write(command)
        except self._error as e:
            logging.error("SGP40 %s: I2C write failed: %s", self.name, e)
            return None

        # Delay for the command to be processed
        if delay > 0:
            self.reactor.pause(self.reactor.monotonic() + delay)

        if read_len == 0:
            return None

        return self._read(read_len)

    def _read(self, read_len):
        reply_len = read_len * SGP40_WORD_LEN
        params = None
        for _ in range(3):
            try:
                params = self.i2c.i2c_read([], reply_len, retry=False)
                break
            except self._error:
                self.reactor.pause(self.reactor.monotonic() + .05)

        if params is None:
            logging.error("SGP40 %s: I2C read failed after retries", self.name)
            return None

        if 'response' not in params or len(params['response']) != reply_len:
            logging.error("SGP40 %s: Invalid response length from sensor",
                          self.name)
            return None

        response = bytearray(params['response'])
        words = []
        for i in range(0, reply_len, SGP40_WORD_LEN):
            data = response[i:i + 2]
            crc = response[i + 2]
            if self._generate_crc(data) != crc:
                logging.error("SGP40 %s: CRC mismatch in sensor response",
                              self.name)
                return None
            words.append((response[i] << 8) | response[i + 1])
        return words

    def _calculate_humidity(self, temp):
        # Magnus formula for estimating the saturation vapor pressure curve
        # relative to 50% RH at 25C
        if temp <= -_MAGNUS_B:
            return 100.
        exp_diff = _REF_VAPOR_EXP - ((_MAGNUS_A * temp) / (_MAGNUS_B + temp))
        if exp_diff > 10.:
            return 100.
        relative_humidity = math.exp(exp_diff) * 50.
        return max(0., min(100., relative_humidity))

    def _temperature_to_ticks(self, temperature):
        temperature = max(-45., min(130., temperature))
        ticks = int(round(((temperature + 45.) * 65535.) / 175.))
        ticks = max(0, min(65535, ticks))
        data = [(ticks >> 8) & 0xFF, ticks & 0xFF]
        crc = self._generate_crc(data)

        return data + [crc]

    def _humidity_to_ticks(self, humidity):
        humidity = max(0., min(100., humidity))
        ticks = int(round((humidity * 65535.) / 100.))
        ticks = max(0, min(65535, ticks))
        data = [(ticks >> 8) & 0xFF, ticks & 0xFF]
        crc = self._generate_crc(data)

        return data + [crc]

    def _generate_crc(self, data):
        # From SGP40 data sheet
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
            'humidity': round(self.humidity, 1),
            'gas_raw': self.raw,
            'gas': self.voc,
        }

# Sensirion Gas Index Algorithm v3.2.0 (VOC) to convert SGP40 raw readings
# to VOC index. Ported from Sensirion's floating-point C reference:
# https://github.com/Sensirion/gas-index-algorithm
# Copyright (c) 2022, Sensirion AG
# SPDX-License-Identifier: BSD-3-Clause
class VOCAlgorithm:
    def __init__(self, sampling_interval=REPORT_TIME):
        self._sampling_interval = float(sampling_interval)
        self._index_offset = _VOC_INDEX_OFFSET_DEFAULT
        self._index_gain = _VOC_INDEX_GAIN
        self._sraw_std_initial = _VOC_SRAW_STD_INITIAL
        self._tau_mean_hours = _VOC_TAU_MEAN_HOURS
        self._tau_variance_hours = _VOC_TAU_VARIANCE_HOURS
        self._gating_max_duration_minutes = _VOC_GATING_MAX_DURATION_MINUTES
        self.reset()

    def reset(self):
        self._uptime = 0.
        self._sraw = 0.
        self._gas_index = 0.
        # Mean/variance estimator state
        dt = self._sampling_interval
        dt_hours = dt / 3600.
        gamma_mean_scale = (_VOC_ADDITIONAL_GAMMA_MEAN_SCALING
                            * _VOC_GAMMA_SCALING)
        self._mve_initialized = False
        self._mve_mean = 0.
        self._mve_sraw_offset = 0.
        self._mve_std = self._sraw_std_initial
        self._mve_gamma_mean_base = (
            (gamma_mean_scale * dt_hours)
            / (self._tau_mean_hours + dt_hours))
        self._mve_gamma_variance_base = (
            (_VOC_GAMMA_SCALING * dt_hours)
            / (self._tau_variance_hours + dt_hours))
        self._mve_gamma_initial_mean = (
            (gamma_mean_scale * dt) / (_VOC_TAU_INITIAL_MEAN + dt))
        self._mve_gamma_initial_variance = (
            (_VOC_GAMMA_SCALING * dt) / (_VOC_TAU_INITIAL_VARIANCE + dt))
        self._mve_gamma_mean = 0.
        self._mve_gamma_variance = 0.
        self._mve_uptime_gamma = 0.
        self._mve_uptime_gating = 0.
        self._mve_gating_duration_minutes = 0.
        # MOX model parameters
        self._mox_sraw_std = self._mve_std
        self._mox_sraw_mean = self._mve_mean + self._mve_sraw_offset
        # Adaptive lowpass filter state
        self._lp_a1 = dt / (_VOC_LP_TAU_FAST + dt)
        self._lp_a2 = dt / (_VOC_LP_TAU_SLOW + dt)
        self._lp_initialized = False
        self._lp_x1 = 0.
        self._lp_x2 = 0.
        self._lp_x3 = 0.

    def _sigmoid(self, sample, x0, k):
        x = k * (sample - x0)
        if x < -50.:
            return 1.
        elif x > 50.:
            return 0.
        return 1. / (1. + math.exp(x))

    def _calculate_gamma(self):
        dt = self._sampling_interval
        uptime_limit = _VOC_UPTIME_LIMIT_MAX - dt
        if self._mve_uptime_gamma < uptime_limit:
            self._mve_uptime_gamma += dt
        if self._mve_uptime_gating < uptime_limit:
            self._mve_uptime_gating += dt

        sigmoid_gamma_mean = self._sigmoid(
            self._mve_uptime_gamma, _VOC_INIT_DURATION_MEAN,
            _VOC_INIT_TRANSITION_MEAN)
        gamma_mean = self._mve_gamma_mean_base + (
            (self._mve_gamma_initial_mean - self._mve_gamma_mean_base)
            * sigmoid_gamma_mean)
        gating_threshold_mean = _VOC_GATING_THRESHOLD + (
            (_VOC_GATING_THRESHOLD_INITIAL - _VOC_GATING_THRESHOLD)
            * self._sigmoid(
                self._mve_uptime_gating, _VOC_INIT_DURATION_MEAN,
                _VOC_INIT_TRANSITION_MEAN))
        sigmoid_gating_mean = self._sigmoid(
            self._gas_index, gating_threshold_mean,
            _VOC_GATING_THRESHOLD_TRANSITION)
        self._mve_gamma_mean = sigmoid_gating_mean * gamma_mean

        sigmoid_gamma_variance = self._sigmoid(
            self._mve_uptime_gamma, _VOC_INIT_DURATION_VARIANCE,
            _VOC_INIT_TRANSITION_VARIANCE)
        gamma_variance = self._mve_gamma_variance_base + (
            (self._mve_gamma_initial_variance - self._mve_gamma_variance_base)
            * (sigmoid_gamma_variance - sigmoid_gamma_mean))
        gating_threshold_variance = _VOC_GATING_THRESHOLD + (
            (_VOC_GATING_THRESHOLD_INITIAL - _VOC_GATING_THRESHOLD)
            * self._sigmoid(
                self._mve_uptime_gating, _VOC_INIT_DURATION_VARIANCE,
                _VOC_INIT_TRANSITION_VARIANCE))
        sigmoid_gating_variance = self._sigmoid(
            self._gas_index, gating_threshold_variance,
            _VOC_GATING_THRESHOLD_TRANSITION)
        self._mve_gamma_variance = sigmoid_gating_variance * gamma_variance

        self._mve_gating_duration_minutes += (
            (dt / 60.)
            * (((1. - sigmoid_gating_mean) * (1. + _VOC_GATING_MAX_RATIO))
               - _VOC_GATING_MAX_RATIO))
        if self._mve_gating_duration_minutes < 0.:
            self._mve_gating_duration_minutes = 0.
        if (self._mve_gating_duration_minutes
                > self._gating_max_duration_minutes):
            self._mve_uptime_gating = 0.

    def _mean_variance_process(self, sraw):
        if not self._mve_initialized:
            self._mve_initialized = True
            self._mve_sraw_offset = sraw
            self._mve_mean = 0.
            return
        if self._mve_mean >= 100. or self._mve_mean <= -100.:
            self._mve_sraw_offset += self._mve_mean
            self._mve_mean = 0.
        sraw -= self._mve_sraw_offset
        self._calculate_gamma()
        delta_sgp = (sraw - self._mve_mean) / _VOC_GAMMA_SCALING
        if delta_sgp < 0.:
            c = self._mve_std - delta_sgp
        else:
            c = self._mve_std + delta_sgp
        additional_scaling = 1.
        if c > 1440.:
            additional_scaling = (c / 1440.) * (c / 1440.)
        self._mve_std = (
            math.sqrt(additional_scaling
                      * (_VOC_GAMMA_SCALING - self._mve_gamma_variance))
            * math.sqrt(
                self._mve_std * (self._mve_std
                                 / (_VOC_GAMMA_SCALING * additional_scaling))
                + (((self._mve_gamma_variance * delta_sgp)
                    / additional_scaling) * delta_sgp)))
        self._mve_mean += (
            (self._mve_gamma_mean * delta_sgp)
            / _VOC_ADDITIONAL_GAMMA_MEAN_SCALING)

    def _mox_model_process(self, sraw):
        return (((sraw - self._mox_sraw_mean)
                 / -(self._mox_sraw_std + _VOC_SRAW_STD_BONUS))
                * self._index_gain)

    def _sigmoid_scaled_process(self, sample):
        x = _VOC_SIGMOID_K * (sample - _VOC_SIGMOID_X0)
        if x < -50.:
            return _VOC_SIGMOID_L
        elif x > 50.:
            return 0.
        if sample >= 0.:
            shift = (_VOC_SIGMOID_L - (5. * self._index_offset)) / 4.
            return ((_VOC_SIGMOID_L + shift) / (1. + math.exp(x))) - shift
        return ((self._index_offset / _VOC_INDEX_OFFSET_DEFAULT)
                * (_VOC_SIGMOID_L / (1. + math.exp(x))))

    def _adaptive_lowpass_process(self, sample):
        if not self._lp_initialized:
            self._lp_x1 = self._lp_x2 = self._lp_x3 = sample
            self._lp_initialized = True
        self._lp_x1 = (1. - self._lp_a1) * self._lp_x1 + self._lp_a1 * sample
        self._lp_x2 = (1. - self._lp_a2) * self._lp_x2 + self._lp_a2 * sample
        abs_delta = abs(self._lp_x1 - self._lp_x2)
        f1 = math.exp(_VOC_LP_ALPHA * abs_delta)
        tau_a = ((_VOC_LP_TAU_SLOW - _VOC_LP_TAU_FAST) * f1
                 + _VOC_LP_TAU_FAST)
        a3 = self._sampling_interval / (self._sampling_interval + tau_a)
        self._lp_x3 = (1. - a3) * self._lp_x3 + a3 * sample
        return self._lp_x3

    def process(self, sraw):
        if self._uptime <= _VOC_INITIAL_BLACKOUT:
            self._uptime += self._sampling_interval
        else:
            if 0 < sraw < 65000:
                sraw = max(_VOC_SRAW_MINIMUM + 1,
                           min(_VOC_SRAW_MINIMUM + 32767, sraw))
                self._sraw = float(sraw - _VOC_SRAW_MINIMUM)
            self._gas_index = self._mox_model_process(self._sraw)
            self._gas_index = self._sigmoid_scaled_process(self._gas_index)
            self._gas_index = self._adaptive_lowpass_process(self._gas_index)
            if self._gas_index < 0.5:
                self._gas_index = 0.5
            if self._sraw > 0.:
                self._mean_variance_process(self._sraw)
                self._mox_sraw_std = self._mve_std
                self._mox_sraw_mean = self._mve_mean + self._mve_sraw_offset
        return int(self._gas_index + 0.5)

def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("SGP40", SGP40)
