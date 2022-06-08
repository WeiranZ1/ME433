import board
import time
import digitalio
import analogio
import math
from ulab import numpy as np # to get access to ulab numpy functions

some_led = digitalio.DigitalInOut(board.GP16) # put an LED circuit on pin GP16
some_led.direction = digitalio.Direction.OUTPUT
some_button = digitalio.DigitalInOut(board.GP15) # put a button circuit on GP15
some_button.direction = digitalio.Direction.INPUT
built_in_led = digitalio.DigitalInOut(board.LED)
built_in_led.direction = digitalio.Direction.OUTPUT
adc_pin_a0 = analogio.AnalogIn(board.A0)

# sampling frequency
fs = 2000
tsetp = 1/fs #sample time interval
a = np.linspace(0,1023,1024)  # time steps
for x in range(1024):
    y1 = np.sin(2*np.pi/100*x)
    y2 = np.sin(2*np.pi/300*x)
    y3 = np.sin(2*np.pi/500*x)

    a[x] = y1 + y2 + y3
    print("("+str(a[x])+",)") # print with plotting format


# perform fft
X = np.fft.fft(a)
for x in range(1024):
    print("("+str(X[x])+",)") # print with plotting format


