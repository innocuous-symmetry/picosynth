import machine

# adapted from MCCP3008 class by @romilly, which was adapted from Adafruit CircuitPython driver
# source: https://github.com/romilly/pico-code/blob/master/src/pico_code/pico/mcp3008/mcp3008.py
class MCP3008:
    def __init__(self, spi: machine.SPI, chip_select: machine.Pin, ref_voltage=3.3):
        self.chip_select = chip_select
        self.chip_select.value(1)

        self._spi = spi
        self._out_buffer = bytearray(3)
        self._out_buffer[0] = 0x01
        self._in_buffer = bytearray(3)
        self._ref_voltage = ref_voltage

    def get_voltage(self):
        return self._ref_voltage
    
    def read(self, pin):
        self.chip_select.value(0)
        self._out_buffer[1] = pin << 4
        self._spi
