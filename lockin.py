from machine import Pin, PWM
from utime import sleep, ticks_ms, ticks_diff

# ---------------- Motor class ----------------
class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=100):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def off(self):  
        self.pwm.duty_u16(0)


# ---------------- Setup motors & sensors ----------------
left_motor  = Motor(4, 5)
right_motor = Motor(7, 6)

s1 = Pin(8,  Pin.IN)   # leftmost
s2 = Pin(9,  Pin.IN)
s3 = Pin(10, Pin.IN)
s4 = Pin(11, Pin.IN)   # rightmost
