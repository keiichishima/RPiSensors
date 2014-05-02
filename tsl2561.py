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

# Copyright 2014 Keiichi Shima. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''A Python class to access TSL2561 based light-to-digital convertor
provided by Switch-Science as a part no. SFE-SEN-12055.  The smbus
module is required.

Example:

import smbus
import tsl2561

bus = smbus.SMBus(1)
sensor = tsl2561.Tsl2561(bus)
print sensor.lux
'''

import sensorbase
import time

# Default I2C address
_DEFAULT_ADDRESS = 0x39

# Commands
_CMD       = 0x80
_CMD_CLEAR = 0x40
_CMD_WORD  = 0x20
_CMD_BLOCK = 0x10

# Registers
_REG_CONTROL   = 0x00
_REG_TIMING    = 0x01
_REG_ID        = 0x0A
_REG_BLOCKREAD = 0x0B
_REG_DATA0     = 0x0C
_REG_DATA1     = 0x0E


# Control parameters
_POWER_UP   = 0x03
_POWER_DOWN = 0x00

# Timing parameters
_GAIN_LOW          = 0b00000000
_GAIN_HIGH         = 0b00010000
_INTEGRATION_START = 0b00001000
_INTEGRATION_STOP  = 0b00000000
_INTEGRATE_13      = 0b00000000
_INTEGRATE_101     = 0b00000001
_INTEGRATE_402     = 0b00000010
_INTEGRATE_DEFAULT = _INTEGRATE_402
_INTEGRATE_NA      = 0b00000011

class Tsl2561(sensorbase.SensorBase):
    def __init__(self, bus, addr = _DEFAULT_ADDRESS):
        '''
        Initializes the sensor with some default values.

        bus: The SMBus descriptor on which this sensor is attached.
        addr: The I2C bus address
            (default is 0x39).
        '''
        assert(bus is not None)
        assert(addr > 0b000111
               and addr < 0b1111000)

        super(Tsl2561, self).__init__(
            update_callback = self._update_sensor_data)

        self._bus = bus
        self._addr = addr
        self._gain = _GAIN_LOW
        self._manual = _INTEGRATION_STOP
        self._integ = _INTEGRATE_DEFAULT

        self._channel0 = None
        self._channel1 = None
        self._lux = None

        self._control(_POWER_UP)
        self._reconfigure()

    @property
    def lux(self):
        '''
        Returns a lux value.  Returns None if no valid value
        is set yet.
        '''
        self._update()
        return (self._lux)

    def _update_sensor_data(self, **kwargs):
        cmd = _CMD | _CMD_WORD | _REG_DATA0
        vals = self._bus.read_i2c_block_data(self._addr, cmd, 2)
        self._channel0 = vals[1] << 8 | vals[0]

        cmd = _CMD | _CMD_WORD | _REG_DATA1
        vals = self._bus.read_i2c_block_data(self._addr, cmd, 2)
        self._channel1 = vals[1] << 8 | vals[0]

        # If either sensor is satulated, no acculate lux value
        # can be achieved.
        if (self._channel0 == 0xffff
            or self._channel1 == 0xffff):
            lux = None
            return

        # The following lux value calculation code is taken from
        # the SparkFun's example code.
        #
        # https://github.com/sparkfun/
        #         TSL2561_Luminosity_Sensor_BOB/blob/master/
        #         Libraries/SFE_TSL2561/SFE_TSL2561.cpp
        d0 = float(self._channel0)
        d1 = float(self._channel1)
        if (d0 == 0):
            # Sometimes, the channel0 returns 0 when dark...
            lux = 0.0
            return
        ratio = d1 / d0

        integ_scale = 1
        if (self._integ == _INTEGRATE_13):
            integ_scale = 402.0 / 13.7
        elif (self._integ == _INTEGRATE_101):
            integ_scale = 402.0 / 101.0
        elif (self._integ == _INTEGRATE_NA):
            integ_scale = 402.0 / kwargs['integration_time']
        d0 = d0 * integ_scale
        d1 = d1 * integ_scale

        if (self._gain == _GAIN_HIGH):
            d0 = d0 / 16
            d1 = d1 / 16

        if (ratio < 0.5):
            self._lux = 0.0304 * d0 - 0.062 * d0 * (ratio ** 1.4)
        elif (ratio < 0.61):
            self._lux = 0.0224 * d0 - 0.031 * d1
        elif (ratio < 0.80):
            self._lux = 0.0128 * d0 - 0.0153 * d1
        elif (ratio < 1.30):
            self._lux = 0.00146 * d0 - 0.00112 * d1
        else:
            self._lux = 0.0

        return

    def _control(self, params):
        cmd = _CMD | _REG_CONTROL
        self._bus.write_byte_data(self._addr, cmd, params)

        # Wait for 400ms to be power up.
        time.sleep(0.4)

    def _reconfigure(self):
        cmd = _CMD | _REG_TIMING
        timing = (self._gain | self._manual | self._integ)
        self._bus.write_byte_data(self._addr, cmd, timing)

        # Wait for 400ms to complete initial A/D conversion.
        time.sleep(0.4)

if __name__ == '__main__':
    import smbus

    bus = smbus.SMBus(1)
    sensor = Tsl2561(bus)
    for cache in [0, 5]:
        print '**********'
        print 'Cache lifetime is %d' % cache
        sensor.cache_lifetime = cache
        for c in range(10):
            print sensor.lux
