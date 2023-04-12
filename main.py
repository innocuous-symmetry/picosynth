from machine import Pin, ADC, PWM, SPI
from random import randint
from math import floor, ceil
from time import sleep

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

# adapted from MCCP3008 class by @romilly, which was adapted from Adafruit CircuitPython driver
# source: https://github.com/romilly/pico-code/blob/master/src/pico_code/pico/mcp3008/mcp3008.py
class MCP3008:
    def __init__(self, spi: SPI, chip_select: Pin, ref_voltage=3.3):
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
        self._spi.write_readinto(self._out_buffer, self._in_buffer)
        self.chip_select.value(1) # turn off
        return ((self._in_buffer[1] & 0x03) << 8) | self._in_buffer[2]
    
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

class Synthesizer:
    # PIN CONFIGURATION

    p1 = None       # GP0
    p2 = None       # GP1
  # p3 = GND
    p4 = None       # GP2
    p5 = None       # GP3
    p6 = None       # GP4
    p7 = None       # GP5
  # p8 = GND
    p9 = None       # GP6
    p10 = None
    p11 = None
    p12 = None
  # p13 = GND
    p14 = None      # GP10
    p15 = None
    p16 = None
    p17 = None
  # p18 = GND
    p19 = None      # GP14
    p20 = None
    p21 = None
    p22 = None
  # p23 = GND
    p24 = None      # GP18
    p25 = None
    p26 = None
    p27 = None
  # p28 = GND
    p29 = None      # GP22
  # p30 = RUN
    p31 = None      # GP26, ADC0
    p32 = None      # GP27, ADC1
  # p33 = GND, AGND
    p34 = None      # GP28, ADC2
    p35 = None      # ADC_VREF
    # p36 = 3v3(out)
    # p37 = 3V3_EN
    # p38 = GND
    # p39 = VSYS
    # p40 = VBUS

    def __init__(self, config):
        self.config = config

gate_out = PWMOutput(15, 0)
voct_out = PWMOutput(16, 0)

# spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=100_000)
# chip_select = Pin(22, Pin.OUT)
# chip_select.value(1)

# square = Pin(21, Pin.OUT)
# mcp = MCP3008(spi, chip_select)

cv_one = PWMOutput(16)
led_one = PWMOutput(17)

pot = Potentiometer(0)

osc = Oscillator('sine', 32)

timer = 1
sleep_interval = 0.01

try:
    while True:
        current_step = polar_to_u16(osc.out())
        
        led_one.set_duty(current_step)
        cv_one.set_duty(current_step)
        
        sleep(0.001 + (pot.read_polar() / 32))
except KeyboardInterrupt:
    print("Exiting program...")
    led_one.set_duty(0)
    cv_one.set_duty(0)
