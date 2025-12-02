# robot_full_final_with_colour_safe.py
# Single-file robot program for MicroPython
# Motors, Actuator
# VL53L0X distance sensor (activated at spurs)
# TCS3472 colour sensor (reads colour before lifting)
# Line following with 4 sensors
# Junction handling
# Spur handling and lift sequence
# GO_HOME sequence integrated in the state machine

from machine import Pin, PWM, I2C
from utime import sleep, ticks_ms, ticks_diff

from libs.VL53L0X.VL53L0X import VL53L0X
from libs.tcs3472_micropython.tcs3472 import tcs3472

# ---------------------------
# CONFIG / PINS (ADJUST HERE)
# ---------------------------
# Motors
LEFT_DIR_PIN  = 4
LEFT_PWM_PIN  = 5
RIGHT_DIR_PIN = 7
RIGHT_PWM_PIN = 6

colour = 'None'
# Line sensors
S1_PIN = 8   # left-most
S2_PIN = 9
S3_PIN = 10
S4_PIN = 11  # right-most
WalkingPin = Pin(16, Pin.OUT)
WalkingPin.value(0)   # start OFF

# Actuator (forklift)
ACT_DIR_PIN = 0
ACT_PWM_PIN = 1

# VL53L0X I2C pins (main I2C bus id=0)
I2C_VL_SDA = 20
I2C_VL_SCL = 21

# TCS3472 I2C pins
I2C_TCS_SDA = 18
I2C_TCS_SCL = 19
TCS_LED_PIN = 17  # illumination LED

# START SIGNAL PIN
START_PIN = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Distance threshold (mm)
DIST_THRESHOLD = 250  # 400 mm = 40 cm

# Timing / stability
REQUIRED_STABLE = 2
JUNCTION_GAPS = [0, 1500, 1500, 1500]

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
        self.is_on = (speed > 0)

    def Reverse(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
        self.is_on = (speed > 0)

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

vl_sensor = None
vl_running = False

# ---------------------------
# COLOUR SENSOR INIT
# ---------------------------
tcs = None
COLOUR_AVAILABLE = False

while not COLOUR_AVAILABLE:
    try:
        colourpin = Pin(TCS_LED_PIN, Pin.OUT)
        colourpin.value(0)

        tcs_i2c = I2C(1, sda=Pin(I2C_TCS_SDA), scl=Pin(I2C_TCS_SCL), freq=10)
        tcs = tcs3472(tcs_i2c)

        colourpin.value(1)
        COLOUR_AVAILABLE = True
        print("TCS3472 initialised successfully.")

    except Exception as e:
        print("TCS init failed, retrying...", e)
        sleep(0.2)

# ---------------------------
# GLOBAL STATE
# ---------------------------
junction = 0
last_pattern = (0, 0, 0, 0)
stable_count = 0
last_junction_time = ticks_ms()

spurs_passed = 0   # NEW

STATE_IDLE = "IDLE"
STATE_FOLLOW = "FOLLOW_LINE"
STATE_SPUR_CHECK = "SPUR_CHECK"
STATE_ENTER_SPUR = "ENTER_SPUR"
STATE_LIFT = "LIFT"
STATE_EXIT_SPUR = "EXIT_SPUR"
STATE_GO_HOME = "GO_HOME"

state = STATE_FOLLOW

# ---------------------------
# VL53L0X MANAGEMENT
# ---------------------------
def vl_sensor_start():
    global vl_sensor, vl_running
    if vl_sensor is None:
        try:
            i2c_vl = I2C(0, sda=Pin(I2C_VL_SDA), scl=Pin(I2C_VL_SCL), freq=100)
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
    if vl_sensor is None:
        return -1
    try:
        d = vl_sensor.read()
        return d if d > 0 else -1
    except Exception:
        return -1

# ---------------------------
# COLOUR DETECTION
# ---------------------------

def colour_detect():
    tcs = None
    COLOUR_AVAILABLE = False
    colourpin = Pin(TCS_LED_PIN, Pin.OUT)
    while not COLOUR_AVAILABLE:
        try:
            colourpin = Pin(TCS_LED_PIN, Pin.OUT)
            colourpin.value(0)

            tcs_i2c = I2C(1, sda=Pin(I2C_TCS_SDA), scl=Pin(I2C_TCS_SCL), freq=10)
            tcs = tcs3472(tcs_i2c)

            colourpin.value(1)
            COLOUR_AVAILABLE = True
            print("TCS3472 initialised successfully.")

        except Exception as e:
            print("TCS init failed, retrying...", e)
            sleep(0.2)
    
    
        if not COLOUR_AVAILABLE or tcs is None:
            print("Colour sensor not available.")
            return "none"

    # Turn ON LED
    colourpin.value(0)

    x_sum = 0
    r_sum = 0
    g_sum = 0
    b_sum = 0
    samples = 0

    # Try 20 reads but ignore failures
    for _ in range(20):
        try:
            x_sum += tcs.light()
            r, g, b = tcs.rgb()
            r_sum += r
            g_sum += g
            b_sum += b
            samples += 1
        except Exception:
            pass
        sleep(0.02)

    # Turn OFF LED
    colourpin.value(1)

    if samples == 0:
        print("Colour read failed (0 samples).")
        return "none"

    # Averaged values
    x = x_sum / samples
    red = r_sum / samples
    green = g_sum / samples
    blue = b_sum / samples

    # ALWAYS print raw values
    print("RAW COLOUR → light:", x, "R:", red, "G:", green, "B:", blue)

    # ------------------------------
    # COLOUR CLASSIFICATION
    # ------------------------------
    if max(red, green, blue) == red and 150 > x > 0:
        return "red"
    if max(red, green, blue) == blue and 150 > x > 0:
        return "green"
    if max(red, green, blue) == blue:
        return "blue"
    if x > 400:
        return "yellow"

    return "none"

# ---------------------------
# MOVEMENT HELPERS
# ---------------------------
def turn_around(duration, speed):
    left_motor.Reverse(speed)
    right_motor.Forward(speed)
    sleep(duration)
    left_motor.off(); right_motor.off()

def pivot_left(duration=0.5, speed=80):
    left_motor.Reverse(speed)
    right_motor.Forward(speed)
    sleep(duration)
    left_motor.off(); right_motor.off()

def pivot_right(duration=0.5, speed=80):
    left_motor.Forward(speed)
    right_motor.Reverse(speed)
    sleep(duration)
    left_motor.off(); right_motor.off()

def drive_forward(duration, speed=50):
    end = ticks_ms() + int(duration*1000)
    while ticks_diff(end, ticks_ms()) > 0:
        left_motor.Forward(speed)
        right_motor.Forward(speed)
        pattern = (s1.value(), s2.value(), s3.value(), s4.value())
        follow_line(pattern)
        sleep(0.01)
    left_motor.off(); right_motor.off()

def drive_reverse(duration, speed=50):
    left_motor.Reverse(speed)
    right_motor.Reverse(speed)
    sleep(duration)
    left_motor.off(); right_motor.off()

# ---------------------------
# LINE FOLLOWING
# ---------------------------
def follow_line(pattern):
    if pattern in [(1,1,0,0),(0,1,0,0)]:
        left_motor.Forward(5); right_motor.Forward(80)
    elif pattern in [(0,0,1,1),(0,0,1,0)]:
        left_motor.Forward(80); right_motor.Forward(5)
    elif pattern==(0,0,0,0):
        left_motor.Forward(60); right_motor.Forward(60)
    else:
        left_motor.Forward(60); right_motor.Forward(60)

# ---------------------------
# JUNCTION HANDLER
# ---------------------------
def forward_start(pattern, now):
    global junction, last_junction_time
    if ticks_diff(now, last_junction_time)<safe_gap_for_junction(junction):
        return False, None

    if pattern==(1,1,1,1) and junction==0:
        junction+=1; last_junction_time=now
        drive_forward(0.3,20); return True, STATE_FOLLOW

    if pattern==(1,1,1,1) and junction==1:
        junction+=1; last_junction_time=now
        drive_forward(0.3,80)
        pivot_right(0.9,60); return True, STATE_FOLLOW

    if pattern in [(0,0,1,1),(0,1,1,1)] and junction==2:
        junction+=1; last_junction_time=now
        drive_forward(0.5,80); return True, STATE_FOLLOW

    if pattern==(1,1,1,1) and junction==3:
        junction+=1; last_junction_time=now
        drive_forward(0.6,70)
        pivot_left(1.2,60); return True, STATE_FOLLOW

    if pattern in [(1,1,0,0),(1,1,1,0)] and 4<=junction<11:
        junction+=1; last_junction_time=now
        return True, STATE_SPUR_CHECK

    return False,None

# ---------------------------
# SPUR HANDLING
# ---------------------------
def handle_spur_entry():
    pattern = (s1.value(),s2.value(),s3.value(),s4.value())
    drive_forward(1,60)
    pivot_left(1,70)
    left_motor.off(); right_motor.off()
    sleep(3)
    follow_line(pattern)
    left_motor.off(); right_motor.off()

def lifting_ground_floor():
    global colour
    left_motor.off(); right_motor.off()
    sleep(2)
    actuator1.set(1,50); sleep(8.1); actuator1.stop()
    drive_forward(1.8,30)
    actuator1.set(1,65); sleep(1.4); actuator1.stop(); sleep(0.5)
    colourpin.value(0)
    colour_detect();
    colour=colour_detect(); print("Detected box colour:",colour)
    drive_reverse(1,80)
    actuator1.set(0,70); sleep(2); actuator1.stop()
    turn_around(1, 60)

# ---------------------------
# GO HOME ROUTINE
# ---------------------------
def go_home(pattern, now):
    if colour == "blue":
        go_home_blue1(pattern, now)
    elif colour == "red":
        go_home_red1(pattern, now)
    elif colour == " green":
        go_home_green1(pattern, now)
    elif colour == "yellow":
        go_home_yellow1(pattern, now)

    # If you later add the second set:
    elif colour == "blue2":
        return go_home_blue2(pattern, now)
    elif colour == "red2":
        return go_home_red2(pattern, now)
    elif colour == "green2":
        return go_home_green2(pattern, now)
    elif colour == "yellow2":
        return go_home_yellow2(pattern, now)

    return False
def go_home_blue1(pattern, now):
    global junction, last_junction_time, stable_count

    if junction == 0:
        if ticks_diff(now, last_junction_time) < 2000:
            return False

    # J0
    if pattern in [(0,1,1,1),(0,0,1,1)] and junction == 0:
        drive_forward(0.5,80)
        pivot_right(1,65)
        junction += 1
        
        return True
    # J1
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 1:
        drive_forward(1,80)
        junction += 1
        return True

    # J2
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 2:
        drive_forward(1,80)
        junction += 1
        return True

    # J3
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 3:
        drive_forward(1,80)
        junction += 1
        return True

    # J4
    if pattern == (1,1,1,1) and junction == 4:
        drive_forward(0.5,80)
        pivot_left(1,65)
        sleep(0.5)
        junction += 1

    if pattern == (1,1,1,1) and junction == 5:
        left_motor.off(); right_motor.off()
        sleep(1)
        actuator1.set(0,70); sleep(7); actuator1.stop()
        drive_reverse(1.2,60)
        turn_around(2.1,60)
        
        return True

    return False



def go_home_red1(pattern, now):
    global junction, last_junction_time, stable_count

    if junction == 0:
        if ticks_diff(now, last_junction_time) < 2000:
            return False

    # J0
    if pattern in [(0,1,1,1),(0,0,1,1)] and junction == 0:
        drive_forward(1, 70); sleep(0.5)
        junction += 1
        return True

    # J1
    if pattern == (1,1,1,1):
        left_motor.off(); right_motor.off()
        sleep(1)
        actuator1.set(0,70); sleep(7); actuator1.stop()
        drive_reverse(1.2,60)
        turn_around(2.1,60)
        return True
    return False



def go_home_green1(pattern, now):
    global junction, last_junction_time, stable_count

    if junction == 0:
        if ticks_diff(now, last_junction_time) < 2000:
            return False

    # J0
    if pattern in [(0,1,1,1),(0,0,1,1)] and junction == 0:
        left_motor.Forward(70); right_motor.Reverse(70); sleep(0.5)
        junction += 1
        return True

    # J1
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 1:
        drive_forward(1, 70)
        junction += 1
        return True

    # J2
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 2:
        drive_forward(1, 70)
        junction += 1
        return True

    # J3
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 3:
        drive_forward(0.5,80)
        pivot_left(1,65)
        sleep(0.5)
        junction += 1

      

    if pattern == (1,1,1,1):
        left_motor.off(); right_motor.off()
        sleep(1)
        actuator1.set(0,70); sleep(7); actuator1.stop()
        drive_reverse(1.2,60)
        turn_around(2.1,60)
        return True

    return False



def go_home_yellow1(pattern, now):
    global junction, last_junction_time, stable_count
    print("returning to yellow base")

    if junction == 0:
        if ticks_diff(now, last_junction_time) < 2000:
            return False

    # J0
    if pattern in [(0,1,1,1),(0,0,1,1)] and junction == 0:
        drive_forward(0.5,80)
        pivot_right(1,65)
        junction += 1
        
        return True

    # J1
    if pattern in [(1,1,1,0),(1,1,0,0)] and junction == 1:
        drive_forward(0.5,80)
        pivot_left(1,65)
        sleep(0.5)
        junction += 1


    if pattern == (1,1,1,1):
        left_motor.off(); right_motor.off()
        sleep(1)
        actuator1.set(0,70); sleep(7); actuator1.stop()
        drive_reverse(1.2,60)
        turn_around(2.1,60)
        return True

    return False



# EMPTY TEMPLATES FOR YOU TO FILL
def go_home_red2(pattern, now):
    global junction, last_junction_time, stable_count
    return False

def go_home_blue2(pattern, now):
    global junction, last_junction_time, stable_count
    return False

def go_home_yellow2(pattern, now):
    global junction, last_junction_time, stable_count
    return False

def go_home_green2(pattern, now):
    global junction, last_junction_time, stable_count
    return False

# ---------------------------
# MAIN LOOP
# ---------------------------
def main_loop():
    global last_pattern, stable_count, state, last_junction_time, junction, spurs_passed
    last_pattern=(0,0,0,0); stable_count=0

    actuator1.set(0,100); sleep(10); actuator1.stop()
    print("Starting main loop...")

    while True:
        pattern = (s1.value(),s2.value(),s3.value(),s4.value())
        WalkingPin.value(left_motor.is_on or right_motor.is_on)
        now=ticks_ms()

        print("Sensors:",pattern,"Junction:",junction,"State:",state)

        stable_count = stable_count+1 if pattern==last_pattern else 0
        last_pattern = pattern

        if state==STATE_FOLLOW:
            follow_line(pattern)
            if stable_count>=REQUIRED_STABLE:
                action,next_state = forward_start(pattern,now)
                if action and next_state==STATE_SPUR_CHECK:
                    state=STATE_SPUR_CHECK
            sleep(0.01); continue

        if state==STATE_SPUR_CHECK:
            print("SPUR DETECTED — stopping immediately.")
            left_motor.off(); right_motor.off()
            WalkingPin.value(0)

            vl_sensor_start()
            sleep(0.15)

            detected=False
            consecutive=0
            required=3

            for _ in range(5):
                d=vl_read_distance()
                print("Distance read:",d)
                if d!=-1 and d < DIST_THRESHOLD:
                    consecutive+=1
                    if consecutive>=required:
                        detected=True; break
                else:
                    consecutive=0
                sleep(0.05)

            vl_sensor_stop()

            if detected:
                print("Box detected → ENTER SPUR")
                state=STATE_ENTER_SPUR
            else:
                print("No box detected → CONTINUE LINE")
                spurs_passed+=1
                drive_forward(0.45,60)
                state=STATE_FOLLOW
            continue

        if state==STATE_ENTER_SPUR:
            follow_line(pattern)
            handle_spur_entry()
            state=STATE_LIFT
            continue

        if state==STATE_LIFT:
            follow_line(pattern)
            lifting_ground_floor()
            state=STATE_EXIT_SPUR
            continue

        if state==STATE_EXIT_SPUR:
            print("Finished spur & lift. Switching to GO_HOME mode.")
            junction=0
            last_junction_time=now
            stable_count=0
            state=STATE_GO_HOME
            continue

        if state==STATE_GO_HOME:
            follow_line(pattern)
            if stable_count>=REQUIRED_STABLE:
                acted=go_home(pattern,now)
                if acted and junction==4:
                    print("Robot reached HOME junction. Stopping and switching to IDLE.")
                    left_motor.off(); right_motor.off()
                    state=STATE_IDLE
            sleep(0.02); continue

        if state==STATE_IDLE:
            left_motor.off(); right_motor.off()
            sleep(0.1)
            continue

# ---------------------------
# ENTRY POINT
# ---------------------------
if __name__=="__main__":

    # ------------------------------------------------------
    # *** WAIT FOR START SIGNAL ON GPIO22 ***
    # ------------------------------------------------------
    print("Waiting for HIGH signal on GPIO22 to start robot...")
    while START_PIN.value() == 0:
        sleep(0.05)
    print("Start signal received! Beginning operation.")

    try:
        main_loop()
    except KeyboardInterrupt:
        print("Interrupted by user. Stopping everything.")
    finally:
        left_motor.off()
        right_motor.off()
        actuator1.stop()
        vl_sensor_stop()
