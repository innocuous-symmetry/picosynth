"""
MicroPython Library for MCP3008 8-channel ADC with SPI

Datasheet for the MCP3008: https://www.microchip.com/datasheet/MCP3008

This code makes much use of Adafruit's CircuitPython code at
https://github.com/adafruit/Adafruit_CircuitPython_MCP3xxx
adapted for MicroPython.

Tested on the Raspberry Pi Pico.

Thanks, @Raspberry_Pi and @Adafruit, for all you've given us!
"""

import machine


class MCP3008:

    def __init__(self, spi, cs, ref_voltage=3.3):
        """
        Create MCP3008 instance

        Args:
            spi: configured SPI bus
            cs:  pin to use for chip select
            ref_voltage: r
        """
        self.cs = cs
        self.cs.value(1) # ncs on
        self._spi = spi
        self._out_buf = bytearray(3)
        self._out_buf[0] = 0x01
        self._in_buf = bytearray(3)
        self._ref_voltage = ref_voltage

    def reference_voltage(self) -> float:
        """Returns the MCP3xxx's reference voltage as a float."""
        return self._ref_voltage

    def read(self, pin, is_differential=False):
        """
        read a voltage or voltage difference using the MCP3008.

        Args:
            pin: the pin to use
            is_differential: if true, return the potential difference between two pins,


        Returns:
            voltage in range [0, 1023] where 1023 = VREF (3V3)

        """

        self.cs.value(0) # select
        self._out_buf[1] = ((not is_differential) << 7) | (pin << 4)
        self._spi.write_readinto(self._out_buf, self._in_buf)
        self.cs.value(1) # turn off
        return ((self._in_buf[1] & 0x03) << 8) | self._in_buf[2]

# And here's the code for the loop-back test.
from machine import Pin, SPI
from time import sleep

spi = SPI(0, sck=Pin(2),mosi=Pin(3),miso=Pin(4), baudrate=100000)
cs = Pin(22, Pin.OUT)
cs.value(1) # disable chip at start

chip = MCP3008(spi, cs)

def loop():
    cv_one = chip.read(0)
    cv_two = chip.read(1)
    cv_three = chip.read(2)
    
    print(f"input one: {cv_one}, input two: {cv_two}, input three: {cv_three}")
    
    sleep(0.01)
    
    cv_one = chip.read(0)
    cv_two = chip.read(1)
    cv_three = chip.read(2)
    
    print(f"input one: {cv_one}, input two: {cv_two}, input three: {cv_three}")
    
    sleep(0.01)


class Module(MCP3008):
    def __init__(self, module_function, spi, cs, ref_voltage=3.3):
        super().__init__(spi, cs, ref_voltage)
        self.module_function = module_function
    
    def read_all(self):
        return {
            'pot_one': self.read(0),
            'pot_two': self.read(1),
            'pot_three': self.read(2),
            'pot_four': self.read(3),
            'cv_in_one': self.read(4),
            'cv_in_two': self.read(5),
            'cv_in_three': self.read(6),
            'cv_in_four': self.read(7)
        }
    
