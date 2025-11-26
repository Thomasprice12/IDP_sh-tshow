# robot_full_final_with_colour.py
# Single-file robot program for MicroPython
# - Motors, Actuator
# - VL53L0X distance sensor (activated at spurs)
# - TCS3472 colour sensor (reads colour before lifting)
# - Line following with 4 sensors
# - Junction handling
# - Spur handling and lift sequence
# - GO_HOME sequence integrated in the state machine

from machine import Pin, PWM, I2C
from utime import sleep, ticks_ms, ticks_diff

# VL53L0X library (same as earlier)
from libs.VL53L0X.VL53L0X import VL53L0X
# TCS3472 colour sensor library (user-provided path)
from libs.tcs3472_micropython.tcs3472 import tcs3472

# ---------------------------
# CONFIG / PINS (ADJUST HERE)
# ---------------------------
# Motors
LEFT_DIR_PIN  = 4
LEFT_PWM_PIN  = 5
RIGHT_DIR_PIN = 7
RIGHT_PWM_PIN = 6

# Line sensors
S1_PIN = 8   # left-most
S2_PIN = 9
S3_PIN = 10
S4_PIN = 11  # right-most
WalkingPin = Pin(16, Pin.OUT)
WalkingPin.value(0)   # start OFF

# Actuator (forklift)
ACT_DIR_PIN = 12
ACT_PWM_PIN = 13

# VL53L0X I2C pins (main I2C bus id=0)
I2C_VL_SDA = 20
I2C_VL_SCL = 21

# TCS3472 I2C pins (can be another bus id=1)
I2C_TCS_SDA = 18
I2C_TCS_SCL = 19
TCS_LED_PIN = 17  # LED/enable pin for colour sensor illumination

# Distance threshold (mm)
DIST_THRESHOLD = 400  # 400 mm = 40 cm

# Timing / stability
REQUIRED_STABLE = 2  # number of consecutive identical patterns to consider stable
JUNCTION_GAPS = [0, 1500, 1500, 1500]  # ms gap per junction index (use last if index exceeds length)

# ---------------------------
# UTIL / HELPERS
# ---------------------------
def safe_gap_for_junction(j):
    if j < len(JUNCTION_GAPS):
        return JUNCTION_GAPS[j]
    return JUNCTION_GAPS[-1]

# ---------------------------
# ACTUATOR CLASS
# ---------------------------
class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def set(self, dir, speed):
        """dir: 0 = extend/lift, 1 = retract/lower, speed: 0-100"""
        self.mDir.value(0 if dir == 0 else 1)
        speed = max(0, min(speed, 100))
        self.pwm.duty_u16(int(65535 * speed / 100))

    def stop(self):
        self.pwm.duty_u16(0)

# ---------------------------
# MOTOR CLASS
# ---------------------------
class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()
        self.is_on = False

    def off(self):
        self.pwm.duty_u16(0)
        self.is_on = False

    def Forward(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))
        self.is_on = (duty > 0)

    def Reverse(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
        self.is_on = (duty > 0)
# ---------------------------
# HARDWARE INSTANTIATION
# ---------------------------
left_motor  = Motor(LEFT_DIR_PIN, LEFT_PWM_PIN)
right_motor = Motor(RIGHT_DIR_PIN, RIGHT_PWM_PIN)
actuator1   = Actuator(ACT_DIR_PIN, ACT_PWM_PIN)

s1 = Pin(S1_PIN, Pin.IN)
s2 = Pin(S2_PIN, Pin.IN)
s3 = Pin(S3_PIN, Pin.IN)
s4 = Pin(S4_PIN, Pin.IN)

# VL53L0X object - created lazily
vl_sensor = None
vl_running = False

# TCS3472 colour sensor setup (create I2C and object)
# Use I2C id=1 for the colour sensor; adjust if your board requires different usage
try:
    tcs_i2c = I2C(1, sda=Pin(I2C_TCS_SDA), scl=Pin(I2C_TCS_SCL), freq=100000)
    tcs = tcs3472(tcs_i2c)
    colourpin = Pin(TCS_LED_PIN, Pin.OUT)
    colourpin.value(1)  # keep LED off/high depending on your wiring; user's code uses 1 initially
    COLOUR_AVAILABLE = True
except Exception as e:
    print("Warning: TCS3472 init failed:", e)
    tcs = None
    colourpin = Pin(TCS_LED_PIN, Pin.OUT)
    COLOUR_AVAILABLE = False

# ---------------------------
# GLOBAL STATE
# ---------------------------
junction = 0
last_pattern = (0, 0, 0, 0)
stable_count = 0
last_junction_time = ticks_ms()

# State machine states
STATE_IDLE = "IDLE"
STATE_FOLLOW = "FOLLOW_LINE"
STATE_SPUR_CHECK = "SPUR_CHECK"
STATE_ENTER_SPUR = "ENTER_SPUR"
STATE_LIFT = "LIFT"
STATE_EXIT_SPUR = "EXIT_SPUR"
STATE_GO_HOME = "GO_HOME"

state = STATE_FOLLOW

# ---------------------------
# VL53L0X DISTANCE SENSOR MANAGEMENT
# ---------------------------
def vl_sensor_start():
    global vl_sensor, vl_running
    if vl_sensor is None:
        try:
            i2c_vl = I2C(0, sda=Pin(I2C_VL_SDA), scl=Pin(I2C_VL_SCL), freq=100000)
            vl_sensor = VL53L0X(i2c_vl)
        except Exception as e:
            print("VL53 init error:", e)
            vl_sensor = None
    if vl_sensor is not None:
        try:
            vl_sensor.start()
        except Exception:
            pass
    vl_running = True

def vl_sensor_stop():
    global vl_running
    if vl_running and vl_sensor is not None:
        try:
            vl_sensor.stop()
        except Exception:
            pass
    vl_running = False

def vl_read_distance():
    """Return distance in mm or -1 on failure/non-positive."""
    global vl_sensor
    if vl_sensor is None:
        return -1
    try:
        d = vl_sensor.read()
        return d if d > 0 else -1
    except Exception:
        return -1

# ---------------------------
# TCS3472 COLOUR DETECTION
# ---------------------------
def colour_detect():
    """Return colour string: 'red', 'green', 'blue', 'yellow', or 'none'"""
    if not COLOUR_AVAILABLE or tcs is None:
        return "none"
    # turn on/enable LED if required by wiring
    # user's code used colourpin.value(0) to enable before measurement
    colourpin.value(0)
    x = 0
    red = 0
    green = 0
    blue = 0
    # average several readings
    for i in range(5):
        try:
            x += tcs.light()
            rgb = tcs.rgb()
            red += rgb[0]
            green += rgb[1]
            blue += rgb[2]
        except Exception:
            # if sensor fails, return none
            colourpin.value(1)
            return "none"
        sleep(0.02)
    # compute averages
    x = x / 5.0
    red = red / 5.0
    green = green / 5.0
    blue = blue / 5.0

    # restore LED off
    colourpin.value(1)

    # Decision logic (as per user's starter)
    if max(red, green, blue) == red and 700 > x > 500:
        return "red"
    elif max(red, green, blue) == blue:
        return "blue"
    elif 550 > x > 450:
        return "green"
    elif 1200 > x > 600:
        return "yellow"
    else:
        return "none"

# ---------------------------
# MOVEMENT HELPERS
# ---------------------------
def turn_around():
    left_motor.Forward(60)
    right_motor.Reverse(60)
    sleep(0.8)
    left_motor.off()
    right_motor.off()

def pivot_left(duration=0.5, speed=70):
    left_motor.Reverse(speed)
    right_motor.Forward(speed)
    sleep(duration)
    left_motor.off()
    right_motor.off()

def pivot_right(duration=0.5, speed=70):
    left_motor.Forward(speed)
    right_motor.Reverse(speed)
    sleep(duration)
    left_motor.off()
    right_motor.off()

def drive_forward(duration, speed=50):
    left_motor.Forward(speed)
    right_motor.Forward(speed)
    sleep(duration)
    left_motor.off()
    right_motor.off()

def drive_reverse(duration, speed=50):
    left_motor.Reverse(speed)
    right_motor.Reverse(speed)
    sleep(duration)
    left_motor.off()
    right_motor.off()

# ---------------------------
# LINE FOLLOWING
# ---------------------------
def follow_line(pattern):
    """Corrective simple behaviour based on 4-sensor pattern."""
    # Left drift -> inner left sensors see black: (1,1,0,0) or (0,1,0,0)
    if pattern in [(1,1,0,0), (0,1,0,0)]:
        left_motor.Forward(40)
        right_motor.Forward(70)
    # Right drift -> inner right sensors see black: (0,0,1,1) or (0,0,1,0)
    elif pattern in [(0,0,1,1), (0,0,1,0)]:
        left_motor.Forward(70)
        right_motor.Forward(40)
    # On line -> go straight faster
    elif pattern == (0,0,0,0):
        left_motor.Forward(65)
        right_motor.Forward(65)
    # Unknown -> safe slow forward
    else:
        left_motor.Forward(50)
        right_motor.Forward(50)

# ---------------------------
# JUNCTION HANDLER (main path)
# ---------------------------
def forward_start(pattern, now):
    """Handle main junction navigation depending on current junction number.
       Returns a tuple (action_taken: bool, next_state: str or None)"""
    global junction, last_junction_time

    # Avoid re-triggering too soon
    if ticks_diff(now, last_junction_time) < safe_gap_for_junction(junction):
        return False, None

    # Junction 1: (1,1,1,1) -> small forward
    if pattern == (1,1,1,1) and junction == 0:
        junction += 1
        last_junction_time = now
        print(">>> Junction 1")
        drive_forward(0.5, speed=20)
        return True, STATE_FOLLOW

    # Junction 2
    if pattern == (1,1,1,1) and junction == 1:
        junction += 1
        last_junction_time = now
        print(">>> Junction 2: pivot right")
        pivot_right(duration=0.5, speed=80)
        return True, STATE_FOLLOW

    # Junction 3
    if pattern == (0,0,1,1) and junction == 2:
        junction += 1
        last_junction_time = now
        print(">>> Junction 3")
        drive_forward(0.5, speed=80)
        return True, STATE_FOLLOW

    # Junction 4
    if pattern == (1,1,1,1) and junction == 3:
        junction += 1
        last_junction_time = now
        print(">>> Junction 4: pivot left")
        pivot_left(duration=0.5, speed=80)
        return True, STATE_FOLLOW

    # Spur junctions 5..11: (1,1,0,0)
    if pattern == (1,1,0,0) and 4 <= junction < 11:
        junction += 1
        last_junction_time = now
        print(f">>> Spur Junction {junction}: prepare sensor check")
        return True, STATE_SPUR_CHECK

    return False, None

# ---------------------------
# SPUR HANDLING
# ---------------------------
def handle_spur_entry():
    """Enter the spur: small left turn and drive inside."""
    pivot_left(duration=0.45, speed=70)
    drive_forward(0.5, speed=40)

def lifting_ground_floor():
    """Actuator-driven pick sequence (timings approximate; calibrate)."""

    # At this point the robot is close to the box — read colour before lifting:
    colour = colour_detect()
    print("Detected box colour:", colour)

    # Lift to ~35 mm
    print("Lifting to ~35 mm")
    actuator1.set(dir=0, speed=50)   # extend / lift
    sleep(8)
    actuator1.stop()
    sleep(0.5)

    # Advance to engage object (if necessary)
    print("Moving forward to engage object")
    drive_forward(2, speed=40)

    # Lift to > 40 mm (higher)
    print("Lifting above ~40 mm")
    actuator1.set(dir=0, speed=50)
    sleep(3.5)
    actuator1.stop()
    sleep(0.5)

    # Reverse out of spur
    print("Reversing out of spur")
    drive_reverse(2, speed=40)

    # Lower to ~20 mm
    print("Lowering to ~20 mm")
    actuator1.set(dir=1, speed=50)   # retract / lower
    sleep(7)
    actuator1.stop()
    sleep(0.5)

    # Turn around to face main track
    print("Turning around to exit spur")
    turn_around()

# ---------------------------
# GO HOME ROUTINE (integrated)
# ---------------------------
def go_home1(pattern, now):
    """
    Implements the go-home junction sequence.
    Returns True if an action was taken (junction advanced).
    """
    global junction, last_junction_time, stable_count

    # ---- HOME JUNCTION 1 ----
    if pattern == (1,1,1,1) and junction == 0:
        junction = 1
        last_junction_time = now
        print("HOME >>> Junction 1")
        left_motor.Forward(70)
        right_motor.Reverse(40)
        sleep(0.5)
        left_motor.off()
        right_motor.off()
        stable_count = 0
        return True

    # ---- HOME JUNCTION 2 ----
    if pattern == (0,0,1,1) and junction == 1:
        junction = 2
        last_junction_time = now
        print("HOME >>> Junction 2")
        left_motor.Forward(70)
        right_motor.Reverse(70)
        sleep(0.5)
        left_motor.off()
        right_motor.off()
        stable_count = 0
        return True

    # ---- HOME JUNCTION 3 ----
    if pattern == (1,1,0,0) and junction == 2:
        junction = 3
        last_junction_time = now
        print("HOME >>> Junction 3")
        left_motor.Forward(70)
        right_motor.Forward(70)
        sleep(0.5)
        left_motor.off()
        right_motor.off()
        stable_count = 0
        return True

    # ---- HOME JUNCTION 4 ----
    if pattern == (1,1,0,0) and junction == 3:
        junction = 4
        last_junction_time = now
        print("HOME >>> Junction 4")
        left_motor.Reverse(70)
        right_motor.Forward(70)
        sleep(0.5)
        turn_around()
        left_motor.off()
        right_motor.off()
        stable_count = 0
        return True

    return False

# ---------------------------
# MAIN LOOP (STATE MACHINE)
# ---------------------------
def main_loop():
    global last_pattern, stable_count, state, last_junction_time, junction

    print("Starting main loop...")
    last_pattern = (0,0,0,0)
    stable_count = 0

    while True:
        v1 = s1.value()
        v2 = s2.value()
        v3 = s3.value()
        v4 = s4.value()
        if left_motor.is_on or right_motor.is_on:
            WalkingPin.value(1)   # robot is moving
        else:
            WalkingPin.value(0) 
            
        pattern = (v1, v2, v3, v4)
        now = ticks_ms()

        # Debug print (reduce in final runs if too verbose)
        print("Sensors:", pattern, "Junction:", junction, "State:", state)

        # Stability filter
        if pattern == last_pattern:
            stable_count += 1
        else:
            stable_count = 0
        last_pattern = pattern
        if 

        # --------------------
        # FOLLOW state
        # --------------------
        if state == STATE_FOLLOW:
            follow_line(pattern)
            if stable_count >= REQUIRED_STABLE:
                action_taken, next_state = forward_start(pattern, now)
                if action_taken and next_state == STATE_SPUR_CHECK:
                    state = STATE_SPUR_CHECK
            sleep(0.02)
            continue

        # --------------------
        # SPUR CHECK state
        # --------------------
        if state == STATE_SPUR_CHECK:
            print("SPUR CHECK: powering distance sensor")
            vl_sensor_start()
            sleep(0.1)  # sensor settle
            detected = False
            for _ in range(10):
                d = vl_read_distance()
                print("Distance read (mm):", d)
                if d != -1 and d < DIST_THRESHOLD:
                    detected = True
                    break
                sleep(0.1)
            vl_sensor_stop()

            if detected:
                print("Object detected in spur -> entering spur")
                state = STATE_ENTER_SPUR
            else:
                print("No object in spur -> continue on line")
                state = STATE_FOLLOW
            continue

        # --------------------
        # ENTER SPUR state
        # --------------------
        if state == STATE_ENTER_SPUR:
            handle_spur_entry()
            state = STATE_LIFT
            continue

        # --------------------
        # LIFT state
        # --------------------
        if state == STATE_LIFT:
            lifting_ground_floor()
            state = STATE_EXIT_SPUR
            continue

        # --------------------
        # EXIT SPUR state
        # After lifting and turning around we switch to GO_HOME
        # --------------------
        if state == STATE_EXIT_SPUR:
            print("Finished spur & lift. Switching to GO_HOME mode.")
            # Reset junction counter for the go-home logic
            junction = 0
            last_junction_time = now
            stable_count = 0
            state = STATE_GO_HOME
            continue

        # --------------------
        # GO_HOME state
        # --------------------
        if state == STATE_GO_HOME:
            # Continue following line behaviour so robot stays on track while searching junctions
            follow_line(pattern)

            if stable_count >= REQUIRED_STABLE:
                acted = go_home1(pattern, now)
                if acted:
                    # If go_home1 advanced to final home junction (junction == 4) -> finish
                    if junction == 4:
                        print("Robot reached HOME junction. Stopping and switching to IDLE.")
                        left_motor.off()
                        right_motor.off()
                        state = STATE_IDLE
            sleep(0.02)
            continue

        # --------------------
        # IDLE state
        # --------------------
        if state == STATE_IDLE:
            left_motor.off()
            right_motor.off()
            sleep(0.1)
            continue

# ---------------------------
# ENTRY POINT
# ---------------------------
if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        print("Interrupted by user. Stopping everything.")
    finally:
        left_motor.off()
        right_motor.off()
        actuator1.stop()
        vl_sensor_stop()
