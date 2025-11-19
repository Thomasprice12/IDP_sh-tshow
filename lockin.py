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
LL= s1.value()
L=   s2.value()
R = s3.value()
RR = s4.value()

def go_straight(speed=FORWARD_SPEED):
    left_motor.Forward(speed)
    right_motor.Forward(speed)
def_adjust(left_speed=x, right_speed=y):
    left_motor.forward(left_speed)
    right_motor.forward(right_speed)

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
    return (LL, L, R,RR)
'''Black = 0, white = 1'''

'''block of code below should deaL with the 1st 2 junctions '''
counter = 0 
go_straight(100)
while true:
    if read() = (1,1,1,1):
        counter +=1
        if counter ==2:
            turn_right_90()
'''this deals with anything onwards. Pranoy, you need to add a function to deal with dead end bays'''            
while true:
    read()
    if read() == (0,1,1,0):
        go_straight(100)
    elif read() == (0,0,1,0):
        right_motor.Forward(70) # basically slow down right motor so turns to right
    elif read() == (0,1,0,0):
        left_motor.Forward(70) # likewise for left side down so turns to the left
    elif read() == (1,1,1,0):
        turn_left_90()		   # this will turn down every left hand junction i.e. the bays 

