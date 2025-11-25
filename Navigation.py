from machine import Pin, PWM, I2C
from utime import sleep, ticks_ms, ticks_diff
from libs.VL53L0X.VL53L0X import VL53L0X


# ============================================================
#   ACTUATOR CLASS
# ============================================================

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def set(self, dir, speed):
        """dir: 0 = extend, 1 = retract, speed: 0–100"""
        self.mDir.value(dir)
        speed = max(0, min(speed, 100))
        self.pwm.duty_u16(int(65535 * speed / 100))


# ============================================================
#   GLOBAL STATE
# ============================================================

sensor = None
sensor_running = False
junction = 0
last_pattern = (0, 0, 0, 0)
stable_count = 0
REQUIRED_STABLE = 2
last_junction_time = ticks_ms()

JUNCTION_GAPS = [0, 1500, 1500, 1500]


# ============================================================
#   SENSOR MANAGER (VL53L0X)
# ============================================================

def sensor_on():
    global sensor, sensor_running

    print("Sensor ON")
    sensor_running = True

    if sensor is None:
        i2c = I2C(0, sda=Pin(20), scl=Pin(21))
        sensor = VL53L0X(i2c)

    sensor.start()
    THRESHOLD = 400  # mm

    while sensor_running:
        distance = sensor.read()

        if distance > 0:
            print("Distance:", distance, "mm")

            if distance < THRESHOLD:
                print("** BOX DETECTED — Performing spur routine **")
                handle_spur()
                sensor_off()
                break

        sleep(0.2)

    sensor.stop()
    print("Sensor OFF")


def sensor_off():
    global sensor_running
    print("Stopping sensor…")
    sensor_running = False


# ============================================================
#   MOTOR CLASS
# ============================================================

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

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


# Instantiate motors
left_motor  = Motor(4, 5)
right_motor = Motor(7, 6)

# Instantiate actuator
actuator1 = Actuator(12, 13)   # <-- update to correct pins


# ============================================================
#   ROBOT MOVEMENT HELPERS
# ============================================================

def turn_around():
    left_motor.Forward(60)
    right_motor.Reverse(60)
    sleep(0.8)


# ============================================================
#   LINE SENSORS
# ============================================================

s1 = Pin(8, Pin.IN)
s2 = Pin(9, Pin.IN)
s3 = Pin(10, Pin.IN)
s4 = Pin(11, Pin.IN)


# ============================================================
#   LINE FOLLOWING LOGIC
# ============================================================

def follow_line(pattern):

    # Left drift
    if pattern in [(1,1,0,0), (0,1,0,0)]:
        left_motor.Forward(40)
        right_motor.Forward(70)

    # Right drift
    elif pattern in [(0,0,1,1), (0,0,1,0)]:
        left_motor.Forward(70)
        right_motor.Forward(40)

    # Straight
    elif pattern == (0,0,0,0):
        left_motor.Forward(65)
        right_motor.Forward(65)

    # Unknown pattern → slow straight
    else:
        left_motor.Forward(50)
        right_motor.Forward(50)


# ============================================================
#   JUNCTION HANDLING
# ============================================================

def forward_start(pattern, now):
    global junction, last_junction_time

    if ticks_diff(now, last_junction_time) < JUNCTION_GAPS[junction]:
        return

    # Junction 1
    if pattern == (1,1,1,1) and junction == 0:
        junction += 1
        last_junction_time = now
        print(">>> Junction 1")
        left_motor.Forward(20)
        right_motor.Forward(20)
        sleep(0.5)
        return

    # Junction 2
    if pattern == (1,1,1,1) and junction == 1:
        junction += 1
        last_junction_time = now
        print(">>> Junction 2")
        left_motor.Forward(80)
        right_motor.Reverse(80)
        sleep(0.5)
        return

    # Junction 3
    if pattern == (0,0,1,1) and junction == 2:
        junction += 1
        last_junction_time = now
        print(">>> Junction 3")
        left_motor.Forward(80)
        right_motor.Forward(80)
        sleep(0.5)
        return

    # Junction 4
    if pattern == (1,1,1,1) and junction == 3:
        junction += 1
        last_junction_time = now
        print(">>> Junction 4")
        left_motor.Reverse(80)
        right_motor.Forward(80)
        sleep(0.5)
        return

    # Spur junctions 5–11
    if pattern == (1,1,0,0) and 4 <= junction < 11:
        junction += 1
        last_junction_time = now
        print(f">>> Spur Junction {junction}")
        sensor_on()
        return


# ============================================================
#   SPUR ROUTINES (Pick-up)
# ============================================================

def handle_spur():
    """Turn into spur and execute pickup procedure."""
    # Turn left inside spur
    left_motor.Reverse(70)
    right_motor.Forward(70)
    sleep(0.5)

    # Move straight inside spur
    left_motor.Forward(40)
    right_motor.Forward(40)
    sleep(0.5)

    lifting_ground_floor()


def lifting_ground_floor():
    """Pick up object from spur."""
    # Lift to 35 mm
    actuator1.set(dir=0, speed=50)
    sleep(8)
    actuator1.set(dir=1, speed=0)
    sleep(10)

    # Drive forward
    left_motor.Forward(40)
    right_motor.Forward(40)
    sleep(2)

    # Lift above 40 mm
    actuator1.set(dir=0, speed=50)
    sleep(3.5)
    actuator1.set(dir=1, speed=0)
    sleep(10)

    # Reverse out
    left_motor.Reverse(40)
    right_motor.Reverse(40)
    sleep(2)

    # Lower to ~20 mm
    actuator1.set(dir=1, speed=50)
    sleep(7)
    actuator1.set(dir=1, speed=0)
    sleep(10)

    turn_around()


# ============================================================
#   MAIN LOOP
# ============================================================

while True:
    v1 = s1.value()
    v2 = s2.value()
    v3 = s3.value()
    v4 = s4.value()
    pattern = (v1, v2, v3, v4)

    now = ticks_ms()
    print("Sensors:", pattern, "  Junction:", junction)

    # Stability filter
    global last_pattern, stable_count
    if pattern == last_pattern:
        stable_count += 1
    else:
        stable_count = 0

    last_pattern = pattern

    # Follow the line
    follow_line(pattern)

    # Detect junctions
    if stable_count >= REQUIRED_STABLE:
        forward_start(pattern, now)

    sleep(0.05)





         
        
       

        


       











