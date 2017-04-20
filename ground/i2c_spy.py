#!/usr/bin/env python
# -*- coding: latin-1 -*-
"""
I2C spy using the bus pirate

IO connection:
    brown : gnd
    purple : scl
    grey : sda
"""

import pyBusPirateLite.I2C as bp


import sys


class SpyError(Exception):
    """spy error"""
    pass


class Spy(object):
    """spy object"""
    def __init__(self, port='/dev/bus_pirate', baudrate=bp.I2CSpeed._100KHZ):
        # connecting
        self.i2c = bp.I2C(port, 115200)

        # reset device
        self.i2c.resetBP()

        # enter bit bang mode
        if not self.i2c.BBmode():
            raise SpyError('buspirate failed to enter in bitbang mode')

        # enter I2C
        if not self.i2c.enter_I2C():
            raise SpyError('buspirate failed to enter in I2C mode')

        # configure I2C link
        self.i2c.set_speed(baudrate)

        # enter spy mode
        self.i2c.bus_sniffer_start()

    def spy(self):
        """return spying event one by one"""
        return self.i2c.bus_sniffer_read()

    def stop(self):
        """exit the spy mode"""
        self.i2c.bus_sniffer_stop()


def usage():
    """
    ./spy [decode]
    by default, display I2C frames in raw mode
    if 'decode' is given, format frames using égère encoding
    """

    print(usage.__doc__)


def _spy(spy, formatter):
    out = ''

    try:
        while True:
            val = spy.spy()
            out += val

            if val == ']':
                formatter(out)
                out = ''

    except KeyboardInterrupt:
        print('stopping')
        spy.stop()


def spy_raw(spy):
    """display raw I2C frames, one by line"""
    def print_raw(frame):
        print('> ' + frame)

    print('égère I2C spy in raw mode\n')
    _spy(spy, print_raw)


def spy_with_decode(spy):
    import frame

    """display decoded I2C frames"""
    fr = frame.Frame()
    dc = fr.get_derived_class()

    def print_dec(frame):
        print('> ' + frame)

        dest, orig, t_id, cmde, stat = frame[:5]
        data = frame[5:]

        st = 'dest = 0x%02x, ' % dest
        st += 'orig = 0x%02x, ' % orig
        st += 't id = 0x%02x, ' % t_id
        st += 'cmde = 0x%02x ' % cmde
        st += '(%s), ' % dc[cmde].__name__
        st += 'stat = 0b%08b = ' % stat
        st += '{.err = %b, .rsp = %b, .len = %d} ' % (
                stat & 0b1000000,
                stat & 0b0100000,
                stat & 0b0000111,
        )
        st += ' data = ['
        st += ''.join(['0x%02x, ' % d for d in data])
        st += ']'
        print(st)

    print('égère I2C spy with égère frame decoding\n')
    _spy(spy, print_dec)


if __name__ == '__main__':
    """instanciate the spy and select the decoding mode"""
    spy = Spy()

    if len(sys.argv) > 1:
        if sys.argv[1] == 'decode':
            spy_with_decode(spy)
        else:
            usage()
    else:
        spy_raw(spy)
