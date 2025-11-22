from machine import Pin, PWM
from utime import sleep

# --- Actuator class ---
class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

# --- Motor class ---
class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

# --- Setup motors, sensors & actuators ---
left_motor = Motor(4, 5)
right_motor = Motor(7, 6)

s1 = Pin(8, Pin.IN)
s2 = Pin(9, Pin.IN)
s3 = Pin(10, Pin.IN)
s4 = Pin(11, Pin.IN)

actuator1 = Actuator(dirPin=0, PWMPin=1)

# --- Ground floor lifting mechanism function ---
def lifting_ground_floor():

    # robot turns
    right_motor.Forward(80)
    left_motor.Reverse(80)
    sleep(0.5)
    
    # lifts forklift to 35 mm
    actuator1.set(dir = 0, speed=50) 
    sleep(8)
    actuator1.set(dir = 1, speed=0)
    sleep(10)
    
    # go forward until reads (1,1,1,1)
    
    counter = 0 # stores the amount of times (1,1,1,1) read from sensors
    
    while counter < 4:
        
        # read sensors
        v1 = s1.value()
        v2 = s2.value()
        v3 = s3.value()
        v4 = s4.value()
        pattern = (v1, v2, v3, v4)

        sleep(0.1)
        
        # line following
        if pattern in [(1,1,0,0),(0,1,0,0)]:
            right_motor.Forward(60)
            left_motor.Reverse(50)
        elif pattern in [(0,0,1,1),(0,0,1,0)]:
            left_motor.Forward(60)
            right_motor.Reverse(50)
        elif pattern == (0,0,0,0):
            left_motor.Forward(85)
            right_motor.Forward(85)
        else:
            left_motor.Forward(85)
            right_motor.Forward(85)

        # checking when the end of the line is reached
        if pattern == (1,1,1,1):
            counter = counter + 1 # increments counter if pattern (1,1,1,1)
            
        else:
            counter = 0 # resets counter if pattern not (1,1,1,1)
            
    
    # colour sensor
    
    # lift forklift to > 40 mm
    actuator1.set(dir = 0, speed=50) 
    sleep(3.5)
    actuator1.set(dir = 1, speed=0)
    sleep(10)
    
    # reverse
    left_motor.Forward(y)
    right_motor.Forward(y)
    
    # lower forklift to 20 mm
    actuator1.set(dir = 1, speed=50) 
    sleep(7)
    actuator1.set(dir = 1, speed=0)
    sleep(10)
    
    # reverse
    left_motor.Forward(y)
    right_motor.Forward(y)
    
    # go home

lifting_ground_floor()
