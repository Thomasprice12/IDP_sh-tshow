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

# TCS3472 I2C pins (can be another bus id=1)
I2C_TCS_SDA = 18
I2C_TCS_SCL = 19
TCS_LED_PIN = 17  # LED/enable pin for colour sensor illumination

START_PIN = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Distance threshold (mm)
DIST_THRESHOLD = 250  # 400 mm = 40 cm

# Timing / stability
REQUIRED_STABLE = 2  # number of consecutive identical patterns to consider stable
JUNCTION_GAPS = [0, 1500, 1500, 1500]  # ms gap per junction index

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
        sleep(0.2)   # prevent rapid retry flood

# ---------------------------
# GLOBAL STATE
# ---------------------------
junction = 0
last_pattern = (0, 0, 0, 0)
stable_count = 0
last_junction_time = ticks_ms()

STATE_IDLE = "IDLE"
STATE_FOLLOW = "FOLLOW_LINE"
STATE_SPUR_CHECK = "SPUR_CHECK"
STATE_ENTER_SPUR = "ENTER_SPUR"
STATE_LIFT = "LIFT"
STATE_EXIT_SPUR = "EXIT_SPUR"
STATE_GO_HOME = "GO_HOME"

state = STATE_FOLLOW

# ---------------------------
# VL53L0X SENSOR MANAGEMENT
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
    if not COLOUR_AVAILABLE or tcs is None:
        return "none"
    colourpin.value(0)
    x = red = green = blue = 0
    for _ in range(5):
        try:
            x += tcs.light()
            r,g,b = tcs.rgb()
            red += r; green += g; blue += b
        except Exception:
            colourpin.value(1)
            return "none"
        sleep(0.02)
    x /= 5; red/=5; green/=5; blue/=5
    colourpin.value(1)

    if max(red,green,blue) == red and 700>x>500:
        return "red"
    elif max(red,green,blue)==blue:
        return "blue"
    elif 550>x>450:
        return "green"
    elif 1200>x>600:
        return "yellow"
    else:
        return "none"

# ---------------------------
# MOVEMENT HELPERS (safe stop integrated)
# ---------------------------
def turn_around():
    left_motor.Reverse(60)
    right_motor.Forward(60)
    sleep(1.9)
    left_motor.off()
    right_motor.off()

def pivot_left(duration=0.5, speed=80):
    left_motor.Reverse(speed)
    right_motor.Forward(speed)
    sleep(duration)
    left_motor.off()
    right_motor.off()

def pivot_right(duration=0.5, speed=80):
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
    if pattern in [(1,1,0,0),(0,1,0,0)]:
        left_motor.Forward(5); right_motor.Forward(80)
    elif pattern in [(0,0,1,1),(0,0,1,0)]:
        left_motor.Forward(80); right_motor.Forward(5)
    elif pattern==(0,0,0,0):
        left_motor.Forward(80); right_motor.Forward(80)
    else:
        left_motor.Forward(80); right_motor.Forward(80)

# ---------------------------
# JUNCTION HANDLER
# ---------------------------
def forward_start(pattern, now):
    global junction, last_junction_time
    if ticks_diff(now, last_junction_time)<safe_gap_for_junction(junction):
        return False, None

    if pattern==(1,1,1,1) and junction==0:
        junction+=1; last_junction_time=now
        drive_forward(0.5,20); return True, STATE_FOLLOW
    if pattern==(1,1,1,1) and junction==1:
        junction+=1; last_junction_time=now
        drive_forward(0.6,80)
        pivot_right(0.9,80); return True, STATE_FOLLOW
    if pattern in [(0,0,1,1), (0,1,1,1)] and junction==2:
        junction+=1; last_junction_time=now
        drive_forward(0.5,80); return True, STATE_FOLLOW
    if pattern==(1,1,1,1) and junction==3:
        junction+=1; last_junction_time=now
        drive_forward(0.6,80)
        pivot_left(1,80); return True, STATE_FOLLOW
    if pattern in [(1,1,0,0), (1,1,1,0)] and 4<=junction<11:
        junction+=1; last_junction_time=now
        return True, STATE_SPUR_CHECK
    return False,None

# ---------------------------
# SPUR HANDLING
# ---------------------------
def handle_spur_entry():
    pattern = (s1.value(),s2.value(),s3.value(),s4.value())
    drive_forward(1,60)
    pivot_left(1.1,80)
    follow_line(pattern)
    drive_forward(1,10)
    left_motor.off()
    right_motor.off()

def lifting_ground_floor():
    left_motor.off(); right_motor.off()
    actuator1.set(dir = 1, speed = 60); sleep(7); actuator1.stop()
    drive_forward(9,50)
    actuator1.set(dir = 1,speed = 60); sleep(1.8); actuator1.stop(); sleep(0.5)
    colour = colour_detect(); sleep(1); print("Detected box colour:",colour)
    drive_reverse(5.5,40)
    actuator1.set(0,100); sleep(5); actuator1.stop()
    pivot_left(1.1,80)

# ---------------------------
# GO HOME ROUTINE
# ---------------------------
def go_home1(pattern, now):
    global junction, last_junction_time, stable_count
   
    if pattern in [(0,0,1,1) or (0,1,1,1)] and junction==0:
        junction=1; last_junction_time=now
        drive_forward(0.2, speed=30)
        left_motor.Forward(70); right_motor.Reverse(70); sleep(0.5)
        left_motor.off(); right_motor.off(); stable_count=0; return True
    if pattern==(1,1,0,0) and junction==1:
        junction=2; last_junction_time=now
        left_motor.Forward(70); right_motor.Forward(70); sleep(0.5)
        left_motor.off(); right_motor.off(); stable_count=0; return True
    if pattern==(1,1,0,0) and junction==2:
        drive_forward(0.8, speed=60)
        junction=3; last_junction_time=now
        left_motor.Reverse(70); right_motor.Forward(70); sleep(0.5)
        turn_around(); stable_count=0; return True
    return False

# ---------------------------
# MAIN LOOP
# ---------------------------
def main_loop():
    
    global last_pattern, stable_count, state, last_junction_time, junction
    last_pattern = (0,0,0,0); stable_count=0
    actuator1.set(0,100); sleep(10); actuator1.stop()
    print("Starting main loop...")
    while True:
        pattern = (s1.value(),s2.value(),s3.value(),s4.value())
        WalkingPin.value(left_motor.is_on or right_motor.is_on)
        now = ticks_ms()
        print("Sensors:", pattern,"Junction:",junction,"State:",state)
        stable_count = stable_count+1 if pattern==last_pattern else 0
        last_pattern = pattern

        if state==STATE_FOLLOW:
            follow_line(pattern)
            if stable_count>=REQUIRED_STABLE:
                action,next_state = forward_start(pattern,now)
                if action and next_state==STATE_SPUR_CHECK: state=STATE_SPUR_CHECK
            sleep(0.01); continue
        
        if state == STATE_SPUR_CHECK:
            print("SPUR DETECTED — stopping immediately.")
            left_motor.off()
            right_motor.off()
            WalkingPin.value(0)

            # Start distance sensor immediately
            vl_sensor_start()
            sleep(0.15)  # short warm-up; MUCH faster than before

            detected = False
            consecutive_reads = 0
            REQUIRED_CONSECUTIVE = 3

            # Robot stays still while checking distance
            for _ in range(15):   # ~15 quick samples (0.05s each = 0.75s max)
                d = vl_read_distance()
                print("Distance read:", d)

                if d != -1 and d < DIST_THRESHOLD:
                    consecutive_reads += 1
                    if consecutive_reads >= REQUIRED_CONSECUTIVE:
                        detected = True
                        break
                else:
                    consecutive_reads = 0

                sleep(0.05)

            vl_sensor_stop()

            # Move to next state based on detection
            if detected:
                print("Box detected → ENTER SPUR")
                state = STATE_ENTER_SPUR
            else:
            
                print("No box detected → CONTINUE LINE")
                drive_forward(0.45, 60)
                
                state = STATE_FOLLOW
                

            continue

        if state==STATE_ENTER_SPUR:
            pattern = (s1.value(),s2.value(),s3.value(),s4.value())
            follow_line(pattern)
            handle_spur_entry(); state=STATE_LIFT; continue

        if state==STATE_LIFT:
            pattern = (s1.value(),s2.value(),s3.value(),s4.value())
            follow_line(pattern)
            lifting_ground_floor(); state=STATE_EXIT_SPUR; continue

        if state==STATE_EXIT_SPUR:
            print("Finished spur & lift. Switching to GO_HOME mode.")
            junction=0; last_junction_time=now; stable_count=0; state=STATE_GO_HOME; continue

        if state==STATE_GO_HOME:
            follow_line(pattern)
            if stable_count>=REQUIRED_STABLE:
                acted=go_home1(pattern,now)
                if acted and junction==4:
                    print("Robot reached HOME junction. Stopping and switching to IDLE.")
                    left_motor.off(); right_motor.off(); state=STATE_IDLE
            sleep(0.02); continue

        if state==STATE_IDLE:
            left_motor.off(); right_motor.off(); sleep(0.1); continue

# ---------------------------
# ENTRY POINT
# ---------------------------
if __name__=="__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        print("Interrupted by user. Stopping everything.")
    finally:
        left_motor.off(); right_motor.off(); actuator1.stop(); vl_sensor_stop()
