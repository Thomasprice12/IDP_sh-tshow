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
        speed = max(0, min(speed, 100))
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=100):
        speed = max(0, min(speed, 100))
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

# --- Setup motors & sensors ---
left_motor = Motor(4, 5)
right_motor = Motor(7, 6)

s1 = Pin(8, Pin.IN)
s2 = Pin(9, Pin.IN)
s3 = Pin(10, Pin.IN)
s4 = Pin(11, Pin.IN)

junction = 0
last_junction_time = ticks_ms()
last_pattern = (0,0,0,0)

# --- Stability filter ---
stable_count = 0
REQUIRED_STABLE = 2

# --- Per-junction minimum timing ---
JUNCTION_GAPS = [
    0, 1500, 1500, 5000, 1000, 2000, 500, 3500, 1000, 100,
    100, 100, 100, 100, 100, 100
]

# --- Main loop ---
while True:
    # --- Read sensors ---
    v1 = s1.value()
    v2 = s2.value()
    v3 = s3.value()
    v4 = s4.value()
    pattern = (v1, v2, v3, v4)

    print("Sensors:", pattern, "Junction:", junction)
    '''white = 1 , black = 0'''
    sleep(0.1)
    # --- Line following ---
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

    # --- Pattern stability ---
    if pattern == last_pattern:
        stable_count += 1
    else:
        stable_count = 0
    last_pattern = pattern

    # --- Only detect junction if pattern stable ---
    if stable_count >= REQUIRED_STABLE:
        now = ticks_ms()

        # ---- Per-junction delay check ----
        skip_junction = False
        if junction < len(JUNCTION_GAPS):
            if ticks_diff(now, last_junction_time) < JUNCTION_GAPS[junction]:
                skip_junction = True

        # --- Junction logic ---
        if not skip_junction:
            if pattern == (1,1,1,1) and junction == 0:
                junction += 1
                last_junction_time = now
                print(">>> Junction 1")
                left_motor.Forward(20)
                right_motor.Forward(20)
                sleep(0.5)
                stable_count = 0

            elif pattern == (1,1,1,1) and junction == 1:
                junction += 1
                last_junction_time = now
                print(">>> Junction 2")
                left_motor.Forward(80)
                right_motor.Reverse(80)
                sleep(0.5)
                stable_count = 0

            elif pattern == (1,1,1,1) and junction == 2:
                junction += 1
                last_junction_time = now
                print(">>> Junction 3")
                right_motor.Forward(80)
                left_motor.Reverse(80)
                sleep(0.5)
                stable_count = 0

            elif pattern == (0,1,1,0) and junction == 3:
                junction += 1
                last_junction_time = now
                print(">>> Junction 4")
                left_motor.Forward(50)
                right_motor.Forward(50)
                sleep(0.5)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 4:
                junction += 1
                last_junction_time = now
                print(">>> Junction 5")
                right_motor.Forward(100)
                left_motor.Reverse(100)
                sleep(0.5)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 5:
                junction += 1
                last_junction_time = now
                print(">>> Junction 6")
                left_motor.Forward(20)
                right_motor.Forward(20)
                sleep(0.65)
                stable_count = 0

            elif pattern == (0,1,1,0) and junction == 6:
                junction += 1
                last_junction_time = now
                print(">>> Junction 7")
                left_motor.Reverse(20)
                right_motor.Forward(20)
                sleep(0.5)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 7:
                junction += 1
                last_junction_time = now
                print(">>> Junction 8")
                right_motor.Forward(50)
                left_motor.Reverse(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 8:
                junction += 1
                last_junction_time = now
                print(">>> Junction 9")
                right_motor.Forward(50)
                left_motor.Forward(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 9:
                junction += 1
                last_junction_time = now
                print(">>> Junction 10")
                right_motor.Forward(50)
                left_motor.Forward(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 10:
                junction += 1
                last_junction_time = now
                print(">>> Junction 11")
                right_motor.Forward(50)
                left_motor.Forward(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 11:
                junction += 1
                last_junction_time = now
                print(">>> Junction 12")
                right_motor.Forward(50)
                left_motor.Forward(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 12:
                junction += 1
                last_junction_time = now
                print(">>> Junction 13")
                right_motor.Forward(50)
                left_motor.Forward(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (1,1,0,0) and junction == 13:
                junction += 1
                last_junction_time = now
                print(">>> Junction 14")
                right_motor.Forward(80)
                left_motor.Reverse(50)
                sleep(0.65)
                stable_count = 0

            elif pattern == (0,0,1,1) and junction == 14:
                junction += 1
                last_junction_time = now
                print(">>> Junction 15")
                right_motor.Reverse(50)
                left_motor.Forward(80)
                sleep(0.65)
                stable_count = 0

            elif pattern == (0,0,0,0) and junction == 15:
                junction += 1
                last_junction_time = now
                print(">>> Junction 16 → STOP")
                left_motor.off()
                right_motor.off()
                stable_count = 0

    sleep(0.05)

