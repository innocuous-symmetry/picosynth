class Patch:
    def __init__(self, config):
        self.config = config

A_FORMAT_HARDWARE = {
    "CV_IN": 4,
    "CV_OUT": 4,
    "ROTARY_ENCODER": True,
    "POTENTIOMETER": 4,
    "OLED": 1
}

B_FORMAT_HARDWARE = {
    "CV_IN": 3,
    "CV_OUT": 3,
    "ROTARY_ENCODER": False,
    "POTENTIOMETER": 3,
    "OLED": 0.5
}

class AFormatPatch(Patch):
    # expects to have hardware:
    # 3x CV in
    # 3x CV out
    # 3x potentiometer
    # half of OLED screen

    def __init__(self):
        super().__init__(self)

class BFormatPatch(Patch):
    # expects to have hardware:
    # 3x CV in
    # 3x CV out
    # 3x potentiometer
    # half of OLED screen
    def __init__(self):
        super().__init__(self)