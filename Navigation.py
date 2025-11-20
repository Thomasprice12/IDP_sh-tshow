from machine import Pin, PWM
from utime import sleep, ticks_ms, ticks_diff

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












def set1(pattern):
    """
    explored = 1
    unexplored = 0
    """
    Gay_seeker = [0, 0, 0, 0, 0, 0]   # 6 spur states

    for x in range(6):  
        while True:

            # -------- SPUR DETECTED ON THE LEFT --------
            if pattern == (1,1,0,0) and Gay_seeker[x] == 0:

                # rotate right to enter spur
                right_motor.Forward(70)
                left_motor.Reverse(40)
                
                # mark spur as explored
                Gay_seeker[x] = 1   

                # ----- PICKUP CHECK -----
                # if box present, pick up box
                # if no box, ignore and exit spur
                # (your code here)

                # TURN AROUND (placeholder)
                turn_around()

                # If box → return to main path the way we came
                # If not → carry on to next junction
                # (your code here)

                break   # exit while True for this spur

            # No matching spur → break and move to next
            else:
                break


      
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


         

         
        
       

        


       











