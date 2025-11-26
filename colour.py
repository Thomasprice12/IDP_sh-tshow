from machine import Pin, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

i2c_bus = I2C(id=1, sda=Pin(18), scl=Pin(19), freq = 10) 
tcs = tcs3472(i2c_bus)
colourpin = Pin(17, Pin.OUT)
colourpin.value(1)
def colour_detect():
    colour = "none"
    colourpin.value(0)
    x = 0
    red = 0
    green = 0
    blue = 0 
    for i in range(0,5):
        x += tcs.light()
        red += tcs.rgb()[0]
        green += tcs.rgb()[1]
        blue += tcs.rgb()[2]
    x = x/5
    red = red/5
    blue = blue/5
    green = green/5
    if max(red, green, blue) == red and 700 > x > 500:
        colour = "red"
    elif max(red, green, blue) == blue and 1000 > x > 700:
        colour = "blue"
    elif 600 > x > 0:
        colour = "green" 
    elif x > 600:
        colour = "yellow"
    else: 
        colour = "none"
    colourpin.value(1)
    return colour

