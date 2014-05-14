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

# mpl115a2 driver for Raspberry-pi
#
# Copyright (c) 2013, Yojiro UO <yuo@nui.org>, All right reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


'''A Python class to access MPL115A2 based air pressure sensor
provided by AKIZUKI DENSHI TSUSHO CO., LTD. as a part no. I-04596.
The smbus module is required.  This class is derived from Yojiro Uo's
original code published at https://gist.github.com/yojiro/6995427.

Example:

import smbus
import mpl115a2

bus = smbus.SMBus(1)
sensor = mpl115a2.Mpl115a2(bus)
print sensor.pressure_and_temperature

'''

import sensorbase
import struct
import time

# Default I2C address
_DEFAULT_ADDRESS = 0x60

# Registers
_REG_PRESSURE           = 0x00
_REG_TEMPERATURE        = 0x02
_REG_COEFFICIENT_OFFSET = 0x04
_REG_START_CONVERSION   = 0x12

# Commands
_CMD_START_CONVERSION   = 0x12

class Mpl115a2(sensorbase.SensorBase):
    def __init__(self, bus = None, addr = _DEFAULT_ADDRESS):
        '''Initializes the sensor with some default values.

        bus: The SMBus descriptor on which this sensor is attached.
        addr: The I2C bus address
            (default is 0x60).

        '''
        assert (bus is not None)
        assert(addr > 0b0000111
               and addr < 0b1111000)

        super(Mpl115a2, self).__init__(self._update_sensor_data)

        self._bus = bus
        self._addr = addr

        self._a0 = None
        self._b1 = None
        self._b2 = None
        self._c12 = None
        self._pressure = None
        self._temperature = None

        self._read_coefficient_offset()

    @property
    def pressure(self):
        '''Returns a pressure value.  Returns None if no valid value is set
        yet.

        '''
        self._update()
        return (self._pressure)

    @property
    def temperature(self):
        '''Returns a temperature value.  Returns None if no valid value is
        set yet.

        '''
        self._update()
        return (self._temperature)

    @property
    def pressure_and_temperature(self):
        '''Returns pressure and temperature values as a tuple.  This call can
        save 1 transaction than getting a pressure and temperature
        values separetely.  Returns (None, None) if no valid values
        are set yet.

        '''
        self._update()
        return (self._pressure, self._temperature)

    def _read_coefficient_offset(self):
        coeff = self._bus.read_i2c_block_data(self._addr,
                                              _REG_COEFFICIENT_OFFSET, 8)
        (a0, b1, b2, c12) = struct.unpack('>hhhh',
                                          ''.join([chr(x) for x in coeff]))
        self._a0 = float(a0) / (1 << 3)
        self._b1 = float(b1) / (1 << 13)
        self._b2 = float(b2) / (1 << 14)
        self._c12 = float(c12 >> 2) / (1 << 22)

    def _update_sensor_data(self):
        self._bus.write_byte_data(self._addr,
                                  _REG_START_CONVERSION,
                                  _CMD_START_CONVERSION)
        # Wait at least 3ms to complete the conversion process.
        time.sleep(0.004)

        vals = self._bus.read_i2c_block_data(self._addr,
                                             _REG_PRESSURE, 2)
        padc = (vals[0] << 8 | vals[1]) >> 6
        vals = self._bus.read_i2c_block_data(self._addr,
                                             _REG_TEMPERATURE, 2)
        tadc = (vals[0] << 8 | vals[1]) >> 6

        '''
        c12x2 = self._c12 * tadc
        a1 = self._b1 + c12x2
        a1x1 = a1 * padc
        y1 = self._a0 + a1x1
        a2x2 = self._b2 * tadc
        pcomp = y1 + a2x2
        '''
        pcomp = (self._a0 + (self._b1 + self._c12 * tadc) * padc
                 + self._b2 * tadc)
        self._pressure = ((650.0 / 1023.0) * pcomp) + 500.0
        self._temperature = ((tadc - 498.0) / -5.35) + 25.0

if __name__ == '__main__':
    import smbus

    bus = smbus.SMBus(1)
    sensor = Mpl115a2(bus)

    for cache in (0, 5):
        print '**********'
        print 'Cache lifetime is %d' % cache
        sensor.cache_lifetime = cache
        for c in range(10):
            print sensor.pressure
            print sensor.temperature
            print sensor.pressure_and_temperature
