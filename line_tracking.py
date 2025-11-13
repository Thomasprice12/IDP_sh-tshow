from machine import Pin, PWM
from utime import sleep, ticks_ms, ticks_diff

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
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=100):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))


# --- Setup motors & sensors ---
left_motor = Motor(4, 5)
right_motor = Motor(7, 6)

left_sensor = Pin(8, Pin.IN)
right_sensor = Pin(9, Pin.IN)
front_sensor = Pin(10, Pin.IN)

junction = 0
last_junction_time = ticks_ms()
last_pattern = (0, 0, 0)

while True:
    left_val = left_sensor.value()
    right_val = right_sensor.value()
    front_val = front_sensor.value()
    current_pattern = (left_val, right_val, front_val)

    # Debug: print current readings
    print("Sensors:", current_pattern, "Junction:", junction)

    # Update motors for normal line following
    if left_val == 1 and right_val == 0 and front_val ==0:
        right_motor.Forward(100)
        left_motor.Reverse(50)
    elif right_val == 1 and left_val == 0 and front_val == 0:
        left_motor.Forward(100)
        right_motor.Reverse(50)
    elif left_val == 0 and right_val == 0:
        left_motor.Forward(50)
        right_motor.Forward(50)
    else:
        left_motor.Forward(50)
        right_motor.Forward(50)

    # Detect junction *only* when pattern CHANGES
    if current_pattern != last_pattern:
        current_time = ticks_ms()
        time_since_last = ticks_diff(current_time, last_junction_time)

        # Junction 1: all 1s
        if left_val == 1 and right_val == 1 and front_val == 1 and junction == 0:
            junction += 1
            last_junction_time = current_time
            print(">>> Junction 1 detected → go straight")
            left_motor.Forward(50)
            right_motor.Forward(50)
            sleep(0.2)

        # Junction 2: 1 1 0 pattern
        elif left_val == 1 and right_val == 1 and front_val == 0 and junction == 1:
            junction += 1
            last_junction_time = current_time
            print(">>> Junction 2 detected → turn RIGHT")
            left_motor.Forward(100)
            right_motor.Reverse(50)
            sleep(0.5)
            
        #junction 3: 1 1 0 patter
        elif left_val == 1 and right_val == 1 and front_val == 0 and junction == 2:
            junction += 1
            last_junction_time = current_time
            print(">>> Junction 3 detected → turn LEFT")
            right_motor.Forward(100)
            left_motor.Reverse(50)
            sleep(0.5)
  

        




        
     # Update last pattern
    last_pattern = current_pattern

    sleep(0.05)

