from machine import Pin, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

colourpin = Pin(17, Pin.Out)
colourpin.value(1)
def colour_detect():
    colour = "none"
    colourpin.value(0)
    i2c_bus = I2C(id=1, sda=Pin(18), scl=Pin(19)) 
    tcs = tcs3472(i2c_bus)
    x = 0
    for i in range(0,5):
        x += tcs.light()
    x = x/5
    if 3000 > x > 2000:
        colour = "red"
    elif 2000 > x > 1400:
        colour = "green"
    elif 5000 > x > 3000:
        colour = "blue" 
    elif x > 5000:
        colour = "yellow"
    else 
        colour = "none"
    print(colour)
    return colour
        
        

