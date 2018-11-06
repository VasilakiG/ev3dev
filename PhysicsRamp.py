#!/usr/bin/env python3
from ev3dev.ev3 import *
import time
import math
import os

light_cal = 0
light_cal_fall_min = 1
light_cal_threshold = 1000

loop_on = True

### ------ Setting up the motors and sensors -------
os.system("clear")
print("Connecting the IO ports...")
main_motor = MediumMotor("outA")
gyro_sensor = GyroSensor()
gyro_sensor.mode = 'GYRO-ANG'
ramp_level = TouchSensor("in4")
obj_start_sensor = Sensor("in1")
obj_fall_sensor = ColorSensor("in3")
obj_fall_sensor.mode = 'COL-REFLECT'
### ------------------------------------------------

def setup():
    print("BOOTING...")
    print("Make sure that your robot is leveled!")
    input("If your robot is leveled, press any key to continue!")
    
    assert main_motor.connected
    assert gyro_sensor.connected
    assert ramp_level.connected
    assert obj_start_sensor.connected
    assert obj_fall_sensor.connected

    print("Dumping the Gyro Sensor first time data...");
    dump = gyro_sensor.value()
    time.sleep(1)

    while loop_on:
        loop()
        
def calibrateRamp():
    print("Calibrating the ramp...")
    main_motor.run_forever(speed_sp = 100)
    while True:
        if ramp_level.value():
            print("Successfully calibrated the ramp!")
            main_motor.stop()
            break
    time.sleep(1)

def calibrateLight():
    input("Press any key if the light source is set up correctly...")
    global light_cal
    light_cal = obj_start_sensor.value()
    print("Light sensor threshold value set to", light_cal - light_cal_threshold)

def loop():
    calibrateRamp()
    calibrateLight()
    input("=> Press any key to begin the experiment!")
    experiment()
    
def experiment():
    print("Starting the experiment!")

    prev = 0
    angle = 0
    speed = 100
    firsttime = True
    working = True
    direction = -1
    startTime = None
    prevTime = time.time()
    timeCounter = 0
    prevTimeCounter = 0
    motor_working = True
    
    while working:
        # ----------- Time Handling ---------------
        currTime = time.time()
        timeCounter += (currTime - prevTime)
        prevTime = currTime
        # -----------------------------------------
        
        # ----------- Start Sensor Handling ---------------
        if obj_start_sensor.value() < light_cal - light_cal_threshold:
            print("Sensor Threshold Hit!")
            main_motor.stop()
            motor_working = False
            startTime = timeCounter
        
        if motor_working:
            main_motor.run_forever(speed_sp = -speed)
        # --------------------------------------------------
        
        # ----------- Gyro Sensor -------------
        gyro_value = gyro_sensor.value()
        currValue = gyro_value - prev
        
        if not firsttime:
            angle += abs(currValue)
        
        prev = gyro_value
        # -------------------------------------
        
        if (int(timeCounter * 1000) - prevTimeCounter) > 300:
            prevTimeCounter = timeCounter * 1000
            os.system("clear")
            print("Is it the first loop?: ", firsttime)
            print("Time Counter: ", timeCounter)
            print("Start Time: ", startTime)
            print("Ramp Level Sensor Value: ", ramp_level.value())
            print("OBJ Start Sensor Value: ", obj_start_sensor.value())
            print("OBJ Fall Sensor Value: ", obj_fall_sensor.reflected_light_intensity)
            print("GYRO Sensor Value RAW: ", gyro_value)
            print("GYRO Sensor Value: ", currValue)
            print("Angle: ", angle)
            print("Motor Speed: ", speed)
        
        if angle >= 60 and not firsttime:
            main_motor.stop()
            working = False

        # ----------- End Sensor Handling ---------------
        if obj_fall_sensor.reflected_light_intensity > light_cal_fall_min:
            main_motor.stop()
            print(" ----- DONE ----- ")
            print("Time Counter since the experiment started:", timeCounter)
            print("Start Time:", startTime)
            print("Time took to fall down:", timeCounter - startTime)
            print("Angle at which it fell down", angle)
            
            # ------------ Calculations ---------------
            t = timeCounter - startTime
            s = 0.3
            a = 2*s/(t*t)
            g = 9.81
            ms = math.tan(math.radians(angle))
            m = ms - (a / (g * math.cos(math.radians(angle))))
            
            print("The static coefficient of friction is:", ms)
            print("The kinetic coefficient of friction is:", m)
            # -----------------------------------------
            
            time.sleep(10)
            input("Press any key to continue!")
            break
            
        # --------------------------------------------------
        
        firsttime = False 

setup()
