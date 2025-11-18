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





def go_straight(speed=FORWARD_SPEED):
    left_motor.Forward(speed)
    right_motor.Forward(speed)

def stop():
    left_motor.off()
    right_motor.off()


def turn_left_90():
    left_motor.Reverse(TURN_SPEED)
    right_motor.Forward(TURN_SPEED)
    sleep(TURN_TIME_90)
    stop()
    sleep(0.05)

def turn_right_90():
    left_motor.Forward(TURN_SPEED)
    right_motor.Reverse(TURN_SPEED)
    sleep(TURN_TIME_90)
    stop()
    sleep(0.05)

def uturn():
    turn_left_90()
    sleep(0.05)
    turn_left_90()

def read():
    return (s1.value(), s2.value(), s3.value(), s4.value())


# ---------------- Line Follow ----------------
