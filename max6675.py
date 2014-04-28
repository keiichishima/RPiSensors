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

'''
A Python class to access MAX6675 based temperature sensor.  The spidev
module (https://github.com/doceme/py-spidev) is required.

Example:

import spidev
import max6675

sensor = max6675.Max6675(0, 0)
print sensor.temperature
'''

import spidev
import time

class Max6675(object):
    def __init__(self, bus, client):
        '''
        Initializes the sensor.

        bus: The SPI bus.
        client: The identifier of the client.
        '''
        assert(bus is not None)
        assert(client is not None)

        self._bus = bus
        self._client = client
        self._temperature = None
        self._cache_lifetime = 0
        self._last_updated = None
        self._handle = spidev.SpiDev(self._bus, self._client)

    def __del__(self):
        if hasattr(self, '_handle'):
            self._handle.close()

    @property
    def temperature(self):
        '''
        Returns a temperature value.  Returns None if no valid
        value is set yet.
        '''
        self._update()
        return (self._temperature)

    @property
    def cache_lifetime(self):
        '''
        Gets/Sets the cache time (in seconds).
        '''
        return (self._cache_lifetime)
    @cache_lifetime.setter
    def cache_lifetime(self, cache_lifetime):
        assert(cache_lifetime >= 0)

        self._cache_lifetime = cache_lifetime

    def _update(self):
        if self._cache_lifetime > 0:
            now = time.time()
            if (self._last_updated is not None
                and self._last_updated + self._cache_lifetime > now):
                return
            self._last_updated = now

        vals = self._handle.readbytes(2)
        self._temperature = ((vals[0] << 8 | vals[1]) >> 3) * 0.25

if __name__ == '__main__':
    import spidev

    sensor = Max6675(0, 0)
    for cache in [0, 5]:
        print '**********'
        print 'Cache lifetime is %d' % cache
        sensor.cache_lifetime = cache
        for c in range(10):
            print sensor.temperature
