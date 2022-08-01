import board
from ulab import numpy as np

from adafruit_ov7670 import (
        OV7670,
        OV7670_SIZE_DIV16,
        OV7670_COLOR_YUV,
        OV7670_TEST_PATTERN_COLOR_BAR,
)

import sys
import time
import digitalio
import busio

with digitalio.DigitalInOut(board.GP10) as reset:
reset.switch_to_output(False)
time.sleep(0.001)
bus = busio.I2C(scl=board.GP5, sda=board.GP4)

cam = OV7670(
        bus,
        data_pins=[
        board.GP12,
                board.GP13,
                board.GP14,
                board.GP15,
                board.GP16,
                board.GP17,
                board.GP18,
                board.GP19,
],
        clock=board.GP11,
        vsync=board.GP7,
        href=board.GP21,
        mclk=board.GP20,
        shutdown=board.GP22,
        reset=board.GP10,
)

pid = cam.product_id
ver = cam.product_version
cam.size = OV7670_SIZE_DIV16
cam.colorspace = OV7670_COLOR_YUV
cam.flip_y = True
cam.flip_x = True

buf = bytearray(2 * cam.width * cam.height) # where all the raw data is stored

# store the converted pixel data
        red = np.linspace(1,1,cam.width * cam.height, dtype=np.float)
green = np.linspace(0,0,cam.width * cam.height, dtype=np.float)
blue = np.linspace(0,0,cam.width * cam.height, dtype=np.float)
ind = 0

uart = busio.UART(tx=board.GP0, rx=board.GP1, baudrate=115200,  timeout= 1) # open the uart for bluetooth
        uartPic = busio.UART(tx=board.GP8, rx=board.GP9, baudrate=230400,  timeout= 1) # open the uart for the pic

while True:
#sys.stdin.readline() # wait for a newline before taking an image
cam.capture(buf) # get the image

# process the raw data into color pixels
        ind = 0
for d in range(0,2 * cam.width * cam.height,4):
u = buf[d+1] - 128
v = buf[d+3] - 128
red[ind] = buf[d] + 1.370705 * v
if red[ind] > 255:
red[ind] = 255
if red[ind] < 0:
red[ind] = 0
green[ind] = buf[d] - 0.337633 * u - 0.698001 * v
if green[ind] > 255:
green[ind] = 255
if green[ind] < 0:
green[ind] = 0
blue[ind] = buf[d] + 1.732446 * u
if blue[ind] > 255:
blue[ind] = 255
if blue[ind] < 0:
blue[ind] = 0

ind = ind+1
red[ind] = buf[d+2] + 1.370705 * v
if red[ind] > 255:
red[ind] = 255
if red[ind] < 0:
red[ind] = 0
green[ind] = buf[d+2] - 0.337633 * u - 0.698001 * v
if green[ind] > 255:
green[ind] = 255
if green[ind] < 0:
green[ind] = 0
blue[ind] = buf[d+2] + 1.732446 * u
if blue[ind] > 255:
blue[ind] = 255
if blue[ind] < 0:
blue[ind] = 0
ind=ind+1

# send the color data as index red green blue
string_write = ""
num = 0
sumg = 0
for c in range(60,1200,40):
string_write = ""
string_write = string_write + str(num)+" "+str(int(red[c]))+" "+str(int(green[c]))+" "+str(int(blue[c]))+"\r\n"
uart.write(bytearray(string_write)) # uart prints are of type byte
num = num + 1
sumg = sumg + green[c]
meang = sumg / num
com = 0
su = 0
num = 0
for c in range(60,1200,40):
if green[c] < meang:
su = su + 255
com = com+255*num
num=num+1
compos = com/su
print("meang="+str(meang)+" com="+str(su)+" compos="+str(compos))
#print("("+str(meang/10)+","+str(com/1000)+","+str(compos)+",)")
picPrint = str(compos) + "\n"
uartPic.write(bytearray(picPrint))