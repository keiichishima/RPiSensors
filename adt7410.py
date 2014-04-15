#!/usr/bin/env python

# Copyright 2014 IIJ Innovation Institute Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY IIJ INNOVATION INSTITUTE INC. ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL IIJ INNOVATION INSTITUTE INC. OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''
A Python class to access ADT7410 based temperature sensor provided by
AKIZUKI DENSHI TSUSHO CO., LTD. as a part no. M-06675.  The smbus module
is required.

Example:

import smbus
import adt7410

bus = smbus.SMBus(1)
sensor = adt7410.Adt7410(bus)
print sensor.temperature
'''

# Default I2C address
DEFAULT_ADDRESS = 0x48

# Configuration parameters
OP_MODE_CONTINUOUS = 0b00000000
OP_MODE_ONESHOT    = 0b00100000
OP_MODE_1SPS       = 0b01000000
OP_MODE_SHUTDOWN   = 0b01100000
RESOLUTION_13BITS  = 0b00000000
RESOLUTION_16BITS  = 0b10000000

# Registers
REG_TEMPERATURE   = 0x00
REG_CONFIGURATION = 0x03

class Adt7410:
    def __init__(self, bus, addr = DEFAULT_ADDRESS,
                 op_mode = OP_MODE_CONTINUOUS,
                 resolution = RESOLUTION_13BITS):
        '''
        Initializes the sensor with some default values.

        bus: The SMBus descriptor on which this sensor is attached.
        addr: The I2C bus address
            (default is 0x48).
        op_mode: The initial operation mode
            (default is OP_MODE_CONTINUOUS).
        resolution: The resolution of the temperature value
            (default is RESOLUTION_13BITS).
        '''
        assert(bus is not None)
        assert(addr > 0b0000111
               and addr < 0b1111000)
        assert(op_mode == OP_MODE_CONTINUOUS
               or op_mode == OP_MODE_ONESHOT
               or op_mode == OP_MODE_1SPS
               or op_mode == OP_MODE_SHUTDOWN)
        assert(resolution == RESOLUTION_13BITS
               or resolution == RESOLUTION_16BITS)

        self._bus = bus
        self._addr = addr
        self._op_mode = op_mode
        self._resolution = resolution
        self._reconfigure()

    @property
    def temperature(self):
        '''
        Returns a temperature value.
        '''
        vals = self._bus.read_i2c_block_data(self._addr,
                                             REG_TEMPERATURE, 2)
        raw = vals[0] << 8 | vals[1]
        temp = 0
        if (self._resolution == RESOLUTION_13BITS):
            raw = raw >> 3
            if (raw > 4095):
                temp = (raw - 8192) / 16.0
            else:
                temp = raw / 16.0
        else:
            if (raw > 32767):
                temp = (raw - 65536) / 128.0
            else:
                temp = raw / 128.0
        return (temp)

    @property
    def op_mode(self):
        '''
        Gets/Sets current operation mode.

        OP_MODE_CONTINUOUS: Continuous conversion.
        OP_MODE_ONESHOT: One shot.
        OP_MODE_1SPS: 1 SPS mode.
        OP_MODE_SHUTDOWN: Shutdown mode.
        '''
        return (self._op_mode)
    @op_mode.setter
    def op_mode(self, op_mode):
        assert(op_mode == OP_MODE_CONTINUOUS
               or op_mode == OP_MODE_ONESHOT
               or op_mode == OP_MODE_1SPS
               or op_mode == OP_MODE_SHUTDOWN)
        self._op_mode = op_mode
        self._reconfigure()

    @property
    def resolution(self):
        '''
        Gets/Sets the resolution of temperature value.

        RESOLUTION_13BITS: 13 bits mode.
        RESOLUTION_16BITS: 16 bits mode.
        '''
        return (self._resolution)
    @resolution.setter
    def resolution(self, resolution):
        assert(resolution == RESOLUTION_13BITS
               or resolution == RESOLUTION_16BITS)
        self._resolution = resolution
        self._reconfigure()

    def _reconfigure(self):
        self._bus.write_byte_data(self._addr, REG_CONFIGURATION,
                                  (self._op_mode | self._resolution))
