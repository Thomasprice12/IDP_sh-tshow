

         
from machine import Pin, PWM
from utime import sleep, ticks_ms, ticks_diff
from machine import Pin, I2C
from libs.VL53L0X.VL53L0X import VL53L0X
from utime import sleep


'''PRANOY: below is code for the distance sensor. The idea is that the sensor will be, default, off. However, when the robot reaches one of the spurs we turn it on 
using th sensor_on() function. Then, the robot will turn on the spot and measure the distance. If the distance measured < 400mm there is a box. Otherwise the robot will turn to the
right and carry onto the next spur. We will turn off the sensor using the function sensor_off()'''
# Global variables
sensor = None
sensor_running = False


def sensor_on():
    global sensor, sensor_running

    print("Sensor ON")
    sensor_running = True

    # Setup sensor if not created yet
    if sensor is None:
        i2c = I2C(0, sda=Pin(20), scl=Pin(21))
        sensor = VL53L0X(i2c)

    # Start measuring
    sensor.start()
    THRESHOLD = 400  # 40 cm

    # ------------- MAIN SENSOR LOOP -------------
    while sensor_running:
        distance = sensor.read()

        if distance > 0:
            print("Distance:", distance, "mm")

            if distance < THRESHOLD:
                print("** BOX DETECTED → initiating procedure **")
                handle_spur()     # Call your special routine
                sensor_off()      # Stop sensor automatically
                break

        sleep(0.2)

    # When loop stops, stop sensor
    sensor.stop()
    print("Sensor OFF (stopped from loop)")


def sensor_off():
    """Tell the loop to stop."""
    global sensor_running
    print("Turning sensor OFF request")
    sensor_running = False
# ============================
#   MOTOR CLASS
# ============================


class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)
      
    def turn_around():
        # rotate 180 degrees — replace this with your real implementation
        left_motor.Forward(60)
        right_motor.Reverse(60)
        sleep(0.8)

    def Forward(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
        
        

# ============================
#   SETUP
# ============================
left_motor  = Motor(4, 5)
right_motor = Motor(7, 6)

s1 = Pin(8, Pin.IN)
s2 = Pin(9, Pin.IN)
s3 = Pin(10, Pin.IN)
s4 = Pin(11, Pin.IN)
motor_status_pin = 
'''Current GPIO PINS: 11,12,13,14,18,19,20,21,'''

junction = 0
last_pattern = (0,0,0,0)
stable_count = 0
REQUIRED_STABLE = 2

last_junction_time = ticks_ms()

# Timing gaps (per-junction)
JUNCTION_GAPS = [
    0, 1500, 1500, 1500]



# ============================
#   LINE FOLLOWING (SMOOTHED)
# ============================
def follow_line(pattern):

    # Left drift → inner two sensors see black: 1100 or 0100
    if pattern in [(1,1,0,0), (0,1,0,0)]:
        left_motor.Forward(40)
        right_motor.Forward(70)

    # Right drift → inner two sensors see black: 0011 or 0010
    elif pattern in [(0,0,1,1), (0,0,1,0)]:
        left_motor.Forward(70)
        right_motor.Forward(40)

    # Perfectly on line (0000)
    elif pattern == (0,0,0,0):
        left_motor.Forward(65)
        right_motor.Forward(65)

    # Unknown pattern → go straight safely
    else:
        left_motor.Forward(50)
        right_motor.Forward(50)



# ============================
#   JUNCTION ROUTE
# ============================
def forward_start(pattern, now):
    global junction, last_junction_time, stable_count

    # Skip junction if timing gap not reached
    if ticks_diff(now, last_junction_time) < JUNCTION_GAPS[junction]:
        return

    # ——— JUNCTION 1 ———
    if pattern == (1,1,1,1) and junction == 0:
        junction += 1
        last_junction_time = now
        print(">>> Junction 1")
        left_motor.Forward(20)
        right_motor.Forward(20)
        sleep(0.5)
        stable_count = 0
        return

    # ——— JUNCTION 2 ———
    if pattern == (1,1,1,1) and junction == 1:
        junction += 1
        last_junction_time = now
        print(">>> Junction 2")
        left_motor.Forward(80)
        right_motor.Reverse(80)
        sleep(0.5)
        stable_count = 0
        return

    # ——— JUNCTION 3 ———
    if pattern == (0,0,1,1) and junction == 2:
        junction += 1
        last_junction_time = now
        print(">>> Junction 3")
        left_motor.Forward(80)
        right_motor.Forward(80)
        sleep(0.5)
        stable_count = 0
        return

    # ——— JUNCTION 4 ———
    if pattern == (1,1,1,1) and junction == 3:
        junction += 1
        last_junction_time = now
        print(">>> Junction 4")
        right_motor.Forward(80)
        left_motor.Reverse(80)
        sleep(0.5)
        stable_count = 0
        return
    # ——— JUNCTIONS 5 → 11 ———
    if pattern == (1,1,0,0) and junction >= 4 and junction < 11:
        junction += 1
        sensor_on() 
        wh
        last_junction_time = now
        print(f">>> Junction {junction}")
        stable_count = 0
        
        return

def handle_spur():
    '''Turns left into spur'''
    left_motor.Reverse(70)
    right_motor.Forward(70)
    sleep(0.5)
    left_motor.Forward(40)
    right_motor.Forward(40)














# ============================
#   MAIN LOOP
# ============================
while True:
    # Read sensors
    v1 = s1.value()
    v2 = s2.value()
    v3 = s3.value()
    v4 = s4.value()
    pattern = (v1, v2, v3, v4)

    now = ticks_ms()

    # Print for debugging
    print("Sensors:", pattern, "  Junction:", junction)

    # ---------- Stability filter ----------
    if pattern == last_pattern:
        stable_count += 1
    else:
        stable_count = 0
    last_pattern = pattern

    # ---------- Line following ----------
    follow_line(pattern)

    # ---------- Junction detection ----------
    if stable_count >= REQUIRED_STABLE:
        forward_start(pattern, now)

    sleep(0.05)














      
     def go_home1():
       ....
       ....
       ....
      junction = 0
      if pattern == (0,0,1,1) and junction == 0:
        junction += 1
        last_junction_time = now
        print(">>> Junction 1")
        left_motor.Forward(70)
        right_motor.Reverse(40)
        sleep(0.5)
        stable_count = 0
        return

    # ——— JUNCTION 2 ———
    if pattern == (1,1,0,0) and junction == 1:
        junction += 1
        last_junction_time = now
        print(">>> Junction 2")
        left_motor.Forward(70)
        right_motor.Forward(70)
        sleep(0.5)
        stable_count = 0
        return

    # ——— JUNCTION 3 ———
    if pattern == (1,1,0,0) and junction == 2:
        junction += 1
        last_junction_time = now
        print(">>> Junction 3")
        left_motor.Reverse(40)
        right_motor.Forward(80)
        sleep(0.5)
        stable_count = 0
        return
    
    if pattern == (1,1,1,1) and junction == 3:
        junction += 1
        last_junction_time = now
        print(">>> Junction 3")
        left_motor.off
        right_motor.off
        sleep(0.5)
        turn_around()
    
        stable_count = 0
        return


         
#when it's at a spur:

if pattern == (1,1,0,0):
         sesnsor_on()
         
        
       

        


       











