from machine import Pin, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

colourpin = Pin(17, 


def colour_detect():
    
    i2c_bus = I2C(id=1, sda=Pin(18), scl=Pin(19)) 
    print(i2c_bus.scan())
    tcs = tcs3472(i2c_bus)
    while True:
        print("Light:", tcs.light())

