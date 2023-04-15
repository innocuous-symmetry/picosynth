from machine import Pin, ADC, PWM, SPI
from utime import sleep
from random import randint
from math import floor, ceil

__SYSTEM_PWM_FREQUENCY__ = 1000

HIGH = 65535
MID = HIGH / 2
LOW = 0

WAVEFORMS = ['sine', 'square', 'saw', 'triangle']

# thank you rsta2 : https://github.com/rsta2/minisynth/blob/master/src/oscillator.cpp
SINE_WAVE = [
    0.00000000, 0.01745241, 0.03489950, 0.05233596, 0.06975647, 0.08715574, 0.10452846, 0.12186934,
    0.13917310, 0.15643447, 0.17364818, 0.19080900, 0.20791169, 0.22495105, 0.24192190, 0.25881905,
    0.27563736, 0.29237170, 0.30901699, 0.32556815, 0.34202014, 0.35836795, 0.37460659, 0.39073113,
    0.40673664, 0.42261826, 0.43837115, 0.45399050, 0.46947156, 0.48480962, 0.50000000, 0.51503807,
    0.52991926, 0.54463904, 0.55919290, 0.57357644, 0.58778525, 0.60181502, 0.61566148, 0.62932039,
    0.64278761, 0.65605903, 0.66913061, 0.68199836, 0.69465837, 0.70710678, 0.71933980, 0.73135370,
    0.74314483, 0.75470958, 0.76604444, 0.77714596, 0.78801075, 0.79863551, 0.80901699, 0.81915204,
    0.82903757, 0.83867057, 0.84804810, 0.85716730, 0.86602540, 0.87461971, 0.88294759, 0.89100652,
    0.89879405, 0.90630779, 0.91354546, 0.92050485, 0.92718385, 0.93358043, 0.93969262, 0.94551858,
    0.95105652, 0.95630476, 0.96126170, 0.96592583, 0.97029573, 0.97437006, 0.97814760, 0.98162718,
    0.98480775, 0.98768834, 0.99026807, 0.99254615, 0.99452190, 0.99619470, 0.99756405, 0.99862953,
    0.99939083, 0.99984770, 1.00000000, 0.99984770, 0.99939083, 0.99862953, 0.99756405, 0.99619470,
    0.99452190, 0.99254615, 0.99026807, 0.98768834, 0.98480775, 0.98162718, 0.97814760, 0.97437006,
    0.97029573, 0.96592583, 0.96126170, 0.95630476, 0.95105652, 0.94551858, 0.93969262, 0.93358043,
    0.92718385, 0.92050485, 0.91354546, 0.90630779, 0.89879405, 0.89100652, 0.88294759, 0.87461971,
    0.86602540, 0.85716730, 0.84804810, 0.83867057, 0.82903757, 0.81915204, 0.80901699, 0.79863551,
    0.78801075, 0.77714596, 0.76604444, 0.75470958, 0.74314483, 0.73135370, 0.71933980, 0.70710678,
    0.69465837, 0.68199836, 0.66913061, 0.65605903, 0.64278761, 0.62932039, 0.61566148, 0.60181502,
    0.58778525, 0.57357644, 0.55919290, 0.54463904, 0.52991926, 0.51503807, 0.50000000, 0.48480962,
    0.46947156, 0.45399050, 0.43837115, 0.42261826, 0.40673664, 0.39073113, 0.37460659, 0.35836795,
    0.34202014, 0.32556815, 0.30901699, 0.29237170, 0.27563736, 0.25881905, 0.24192190, 0.22495105,
    0.20791169, 0.19080900, 0.17364818, 0.15643447, 0.13917310, 0.12186934, 0.10452846, 0.08715574,
    0.06975647, 0.05233596, 0.03489950, 0.01745241, 0.00000000, -0.01745241, -0.03489950, -0.05233596,
    -0.06975647, -0.08715574, -0.10452846, -0.12186934, -0.13917310, -0.15643447, -0.17364818, -0.19080900,
    -0.20791169, -0.22495105, -0.24192190, -0.25881905, -0.27563736, -0.29237170, -0.30901699, -0.32556815,
    -0.34202014, -0.35836795, -0.37460659, -0.39073113, -0.40673664, -0.42261826, -0.43837115, -0.45399050,
    -0.46947156, -0.48480962, -0.50000000, -0.51503807, -0.52991926, -0.54463904, -0.55919290, -0.57357644,
    -0.58778525, -0.60181502, -0.61566148, -0.62932039, -0.64278761, -0.65605903, -0.66913061, -0.68199836,
    -0.69465837, -0.70710678, -0.71933980, -0.73135370, -0.74314483, -0.75470958, -0.76604444, -0.77714596,
    -0.78801075, -0.79863551, -0.80901699, -0.81915204, -0.82903757, -0.83867057, -0.84804810, -0.85716730,
    -0.86602540, -0.87461971, -0.88294759, -0.89100652, -0.89879405, -0.90630779, -0.91354546, -0.92050485,
    -0.92718385, -0.93358043, -0.93969262, -0.94551858, -0.95105652, -0.95630476, -0.96126170, -0.96592583,
    -0.97029573, -0.97437006, -0.97814760, -0.98162718, -0.98480775, -0.98768834, -0.99026807, -0.99254615,
    -0.99452190, -0.99619470, -0.99756405, -0.99862953, -0.99939083, -0.99984770, -1.00000000, -0.99984770,
    -0.99939083, -0.99862953, -0.99756405, -0.99619470, -0.99452190, -0.99254615, -0.99026807, -0.98768834,
    -0.98480775, -0.98162718, -0.97814760, -0.97437006, -0.97029573, -0.96592583, -0.96126170, -0.95630476,
    -0.95105652, -0.94551858, -0.93969262, -0.93358043, -0.92718385, -0.92050485, -0.91354546, -0.90630779,
    -0.89879405, -0.89100652, -0.88294759, -0.87461971, -0.86602540, -0.85716730, -0.84804810, -0.83867057,
    -0.82903757, -0.81915204, -0.80901699, -0.79863551, -0.78801075, -0.77714596, -0.76604444, -0.75470958,
    -0.74314483, -0.73135370, -0.71933980, -0.70710678, -0.69465837, -0.68199836, -0.66913061, -0.65605903,
    -0.64278761, -0.62932039, -0.61566148, -0.60181502, -0.58778525, -0.57357644, -0.55919290, -0.54463904,
    -0.52991926, -0.51503807, -0.50000000, -0.48480962, -0.46947156, -0.45399050, -0.43837115, -0.42261826,
    -0.40673664, -0.39073113, -0.37460659, -0.35836795, -0.34202014, -0.32556815, -0.30901699, -0.29237170,
    -0.27563736, -0.25881905, -0.24192190, -0.22495105, -0.20791169, -0.19080900, -0.17364818, -0.15643447,
    -0.13917310, -0.12186934, -0.10452846, -0.08715574, -0.06975647, -0.05233596, -0.03489950, -0.01745241
]

VOCT_PITCH_VALUES = [0.1, 0.2, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7]
# VOCT_PITCH_VALUES[x] * 65535 = our random pitch

# the format for specifying synthesizer configuration
SYNTH_CONFIG = {
    # valid member types: analog in, digital in
    "inputs": [],
    # valid member types: digital out, PWM out
    "outputs": [],

    # specifies the ways each module on the synthesizer should behave
    "module_config": {
        # modules A and B will have a different hardware configuration
        "A": {},
        "B": {},
        "C": {},
        "D": {}
    }
}

def blink(controller: type[ADC], target=Pin(25, Pin.OUT), sleep_duration = 1000):
    if not controller: pass

    # POT.read_u16() / 65535 approaches 0 as potentiometer approaches max,
    # approaches 1 as potentiometer approaches min
    
    modulated_sleep = max(floor(sleep_duration * (controller.read_u16()) / 65535), 5) / 1000
    
    target.value(1)
    sleep(modulated_sleep)
    
    modulated_sleep = max(floor(sleep_duration * (controller.read_u16()) / 65535), 5)
    
    target.value(0)
    sleep(modulated_sleep)

# convert input from -1 to 1 range, to u16
def polar_to_u16(input: float) -> int: 
    return floor((input + 1) * 32767.5)

# convert input from u16 to -1 to 1 range
def u16_to_polar(input: int) -> float: 
    return (input / 32767.5) - 1


""" " " " " " " " " " " " " " " " " " " " " " "
" BEGIN CLASS DEFINITIONS FOR BASIC HARDWARE BEHAVIORS
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ """

# for representing and interacting with waveform data
class Hertz:
    ONE_HERTZ = len(SINE_WAVE) / 1000

    def __init__(self, hz: int):
        self.hz = hz
        self.tick_freq = self.ONE_HERTZ * hz
        pass

class Oscillator:
    def __init__(self, waveform = 'sine', downsampling: int = 0, current_tick: int = 0) -> None:
        self.waveform = waveform if waveform in WAVEFORMS else None
        self.downsampling = downsampling
        self.current_tick = current_tick
        self.value = 0
        
    def out(self):
        if (self.waveform is 'square'):
            self.value = not self.value
        else:
            current_step = SINE_WAVE[floor(self.current_tick)]
            interval = self.downsampling if self.downsampling > 0 else 1
            self.current_tick = self.current_tick + interval if self.current_tick + interval < len(SINE_WAVE) else 0
            return current_step
        
    def out_u16(self):
        return polar_to_u16(self.out())

# basic set of analog input behaviors
class AnalogInput:
    def __init__(self, ADC_PIN):
        self.PIN = ADC(ADC_PIN)
        self.value = 0

    def read(self):
        self.value = self.PIN.read_u16()
        return self.value
    
    def read_polar(self):
        self.value = self.PIN.read_u16() / 65535
        return self.value
    
class Potentiometer(AnalogInput):
    def __init__(self, ADC_PIN: int):
        super().__init__(ADC_PIN)

# output behaviors and utilities
class PWMOutput:
    def __init__(self, PIN: int, duty: int = 512):
        new_pwm = PWM(Pin(PIN))
        new_pwm.freq(__SYSTEM_PWM_FREQUENCY__)
        self.PWM = new_pwm
        self.duty = duty

    def set_duty(self, duty: int):
        self.PWM.duty_u16(duty)
        return self.PWM.duty_u16()

    def get_duty(self):
        return self.PWM.duty_u16()
    
    def get_freq(self):
        return self.PWM.freq()
    
    def set_freq(self, freq: int):
        self.PWM.freq(freq)

class DigitalOut:
    def __init__(self, PIN: int):
        self.PIN = Pin(PIN, Pin.OUT)

    def read(self):
        return self.PIN.value()
    
"""
DEFINITIONS FOR HARDWARE PERIPHERALS
"""

# from MCCP3008 class by @romilly, which was adapted from Adafruit CircuitPython driver
# source: https://github.com/romilly/pico-code/blob/master/src/pico_code/pico/mcp3008/mcp3008.py
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
    

class Module:
    def __init__(self, mcp3008: MCP3008, out_one: int, out_two: int):
        self.chip = mcp3008
        self.cv_out_one = PWMOutput(out_one)
        self.cv_out_two = PWMOutput(out_two)

    def cleanup(self):
        self.cv_out_one.set_duty(0)
        self.cv_out_two.set_duty(0)

    def read_one(self, pin: int):
        return self.chip.read(pin)

    def read_all(self):
        return {
            'pot_one': chip.read(0),
            'pot_two': chip.read(1),
            'pot_three': chip.read(2),
            'pot_four': chip.read(3),
            'cv_in_one': chip.read(4),
            'cv_in_two': chip.read(5),
            'cv_in_three': chip.read(6),
            'cv_in_four': chip.read(7)
        }
    
    def loop(self, sleep_interval=0.01, function=None):
        if function:
            function()
        else:
            print()
            # convert to u16
            # cv_in_value = self.chip.read(4) * 64
            # print(cv_in_value)
            # self.cv_out_one.set_duty(cv_in_value)


        sleep(sleep_interval)


class ADSR(Module):
    def __init__(self, mcp3008: MCP3008, out_one: int, out_two: int):
        super().__init__(mcp3008, out_one, out_two)

    def loop(self, time_interval=0.01):
        if time_interval < 0:
            raise Exception("Time interval may not be less than 0")
        elif time_interval > 1:
            raise Exception("Time interval may not be greater than 1")

        # only move on to envelope generation when a gate is detected
        if self.chip.read(4) is not 0:
            # get data and convert to polar
            attack = (self.chip.read(0) / 1024)
            decay = (self.chip.read(1) / 1024)
            sustain = self.chip.read(2) / 1024
            release = (self.chip.read(3) / 1024)

            counter = 0
            value = 0

            print(self.chip.read(4))

            attack_steps = floor(attack / time_interval)
            while counter < attack_steps:
                value = value + time_interval
                counter = counter + 1
                print(value)
                self.cv_out_one.set_duty(floor(value))

            decay_steps = floor(decay / time_interval)
            while counter < attack_steps + decay_steps:
                value = value - time_interval
                counter = counter + 1
                print(value)
                self.cv_out_one.set_duty(floor(value))

        sleep(time_interval)


"""
variable initialization and prep for loop
"""
spi = SPI(0, sck=Pin(2),mosi=Pin(3),miso=Pin(4), baudrate=100000)
cs = Pin(22, Pin.OUT)
cs.value(1) # disable chip at start

chip = MCP3008(spi, cs)
module = Module(chip, out_one=14, out_two=15)
adsr = ADSR(chip, out_one=14, out_two=15)

try:
    while True:
        adsr.loop()
except KeyboardInterrupt:
    module.cleanup()
    print("Exiting program...")

