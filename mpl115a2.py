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


'''
A Python class to access MPL115A2 based air pressure sensor provided by
AKIZUKI DENSHI TSUSHO CO., LTD. as a part no. I-04596.  The smbus module
is required.  This class is derived from Yojiro Uo's original code
published at https://gist.github.com/yojiro/6995427.

'''

import struct
import time

# Default I2C address
DEFAULT_ADDRESS = 0x60

# Registers
REG_PRESSURE           = 0x00
REG_TEMPERATURE        = 0x02
REG_COEFFICIENT_OFFSET = 0x04
REG_START_CONVERSION   = 0x12

# Commands
CMD_START_CONVERSION   = 0x12

class Mpl115a2:
    def __init__(self, bus, addr = DEFAULT_ADDRESS):

        assert (bus is not None)
        assert(addr > 0b0000111
               and addr < 0b1111000)

        self._bus = bus
        self._addr = addr
        self._a0 = None
        self._b1 = None
        self._b2 = None
        self._c12 = None
        self._read_coefficient_offset()

    def _read_coefficient_offset(self):
        reg = []
        for i in range(8):
            v = self._bus.read_byte_data(self._addr,
                                         REG_COEFFICIENT_OFFSET + i)
            reg.append(v)
        (a0, b1, b2, c12) = struct.unpack('>hhhh',
                                          ''.join([chr(x) for x in reg]))
        self._a0 = float(a0) / (1 << 3)
        self._b1 = float(b1) / (1 << 13)
        self._b2 = float(b2) / (1 << 14)
        self._c12 = float(c12 >> 2) / (1 << 22)

    def update(self):
        self._bus.write_i2c_block_data(self._addr,
                                       REG_START_CONVERSION,
                                       [CMD_START_CONVERSION])
        time.sleep(0.005)
        msb = self._bus.read_byte_data(self._addr,
                                       REG_PRESSURE + 0)
        lsb = self._bus.read_byte_data(self._addr,
                                       REG_PRESSURE + 1)
        self._pressure = ((msb << 8 | lsb) & 0xffc0) >> 6

        msb = self._bus.read_byte_data(self._addr,
                                       REG_TEMPERATURE + 0)
        lsb = self._bus.read_byte_data(self._addr,
                                       REG_TEMPERATURE + 1)
        self._temperature = ((msb << 8 | lsb) & 0xffc0) >> 6

    @property
    def pressure(self):
        pcomp = self._a0
        pcomp += (self._b1 + self._c12 * self._temperature) * self._pressure
        pcomp += self._b2 * self._temperature
        return (((650.0 / 1023.0) * pcomp) + 500.0)

    @property
    def temperature(self):
        return ((self._temperature - 498.0) / -5.35) + 25.0
