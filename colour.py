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
    for i in range(0,20):
        x += tcs.light()
        red += tcs.
    x = x/20
    if 3000 > x > 2000:
        colour = "red"
    elif 2000 > x > 1400:
        colour = "green"
    elif 5000 > x > 3000:
        colour = "blue" 
    elif x > 5000:
        colour = "yellow"
    else: 
        colour = "none"
    print(colour)
    return colour
        
        

