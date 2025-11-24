def lifting_ground_floor():

    # robot turns
    
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

  #lower forklift to 20mm
    actuator1.set(dir = 1, speed=50) 
    sleep(7)
    actuator1.set(dir = 1, speed=0)
    sleep(10)
    
    # reverse
    left_motor.Reverse(y)
    right_motor.Reverse(y)
    sleep(2)
    u_turn()
    
    
