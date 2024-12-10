# SPDX-FileCopyrightText: 2024 Jeff Epler, written for Adafruit Industries
# SPDX-FileCopyrightText: 2021 Scott Shawcroft, written for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time

import adafruit_pioasm
import adafruit_pixelbuf
import adafruit_rp1pio
import board
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.rainbowchase import RainbowChase
from adafruit_led_animation.animation.rainbowcomet import RainbowComet
from adafruit_led_animation.animation.rainbowsparkle import RainbowSparkle
from adafruit_led_animation.sequence import AnimationSequence

# chosen because it's on a connector on the braincraft hat
NEOPIXEL = board.D13

# NeoPixels are 800khz bit streams. We are choosing zeros as <312ns hi, 936 lo>
# and ones as <700 ns hi, 556 ns lo>.
# The first two instructions always run while only one of the two final
# instructions run per bit. We start with the low period because it can be
# longer while waiting for more data.
program = """
.program ws2812
.side_set 1
.wrap_target
bitloop:
   out x 1        side 0 [6]; Drive low. Side-set still takes place before instruction stalls.
   jmp !x do_zero side 1 [3]; Branch on the bit we shifted out previous delay. Drive high.
 do_one:
   jmp  bitloop   side 1 [4]; Continue driving high, for a one (long pulse)
 do_zero:
   nop            side 0 [4]; Or drive low, for a zero (short pulse)
.wrap
"""

assembled = adafruit_pioasm.assemble(program)


class Pi5Pixelbuf(adafruit_pixelbuf.PixelBuf):
    def __init__(self, pin, size, **kwargs):
        self._pin = pin
        self.sm = adafruit_rp1pio.StateMachine(
            assembled,
            frequency=12_800_000,  # to get appropriate sub-bit times in PIO program
            first_sideset_pin=pin,
            auto_pull=True,
            out_shift_right=False,
            pull_threshold=8,
        )
        #print("real frequency", self.sm.frequency)
        super().__init__(size=size, **kwargs)

    def _transmit(self, buf):
        self.sm.write(buf)

pixels = Pi5Pixelbuf(NEOPIXEL, 120, auto_write=True, byteorder="BGR")

pixels.fill(0x10102)
pixels.show()
time.sleep(2)

rainbow = Rainbow(pixels, speed=0.02, period=2)
rainbow_chase = RainbowChase(pixels, speed=0.02, size=5, spacing=3)
rainbow_comet = RainbowComet(pixels, speed=0.02, tail_length=7, bounce=True)
rainbow_sparkle = RainbowSparkle(pixels, speed=0.02, num_sparkles=15)


animations = AnimationSequence(
    rainbow,
    rainbow_chase,
    rainbow_comet,
    rainbow_sparkle,
    advance_interval=5,
    auto_clear=True,
)

try:
    while True:
        animations.animate()
finally:
    time.sleep(.01)
    pixels.fill(0)
    pixels.show()
