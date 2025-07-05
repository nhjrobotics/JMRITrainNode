## https://github.com/satyamkr80/Raspberry-Pi-Pico-Micropython-Examples/blob/main/ws2812-neopixel-with-Pico-RP2040.py

import array, time
from machine import Pin
import rp2

# Configure the number of WS2812 LEDs
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


class ws2812_array:
    def __init__(self, num_leds, pin_num):
        self.num_leds = num_leds
        self.pin_num = pin_num

        # StateMachine init
        self.sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(self.pin_num))
        # Start the StateMachine, it will wait for data on its FIFO.
        self.sm.active(1)

        # Setup an array to hold LED RGB values.
        self.ar = array.array("I", [0 for _ in range(self.num_leds)])


    def pixels_set(self, led, color):
        self.ar[led] = (color[1]<<16) + (color[0]<<8) + color[2]


    def pixels_show(self):
        dimmer_ar = array.array("I", [0 for _ in range(self.num_leds)])
        for i,c in enumerate(self.ar):
            r = int(((c >> 8) & 0xFF))
            g = int(((c >> 16) & 0xFF))
            b = int((c & 0xFF))
            dimmer_ar[i] = (g<<16) + (r<<8) + b
        self.sm.put(dimmer_ar, 8)
        time.sleep_ms(10)