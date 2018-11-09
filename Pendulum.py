#!/usr/bin/env python3
from ev3dev.ev3 import *
import time
import math
import os

pendulum_sensor = ColorSensor("in4")
pendulum_sensor.mode = 'COL-AMBIENT'
gate = Sensor("in2")

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def loop():
  while True:
    setup()

def setup():
  os.system("clear")
  input("Press any key if the object is in the equilibrium state...")
  print("Calibrating the sensor...")

  eq_state_light = pendulum_sensor.value() - 2;
  eq_state = True

  while True:
    if (pendulum_sensor.value() < eq_state_light):
      print("Ball left the eq state")
      eq_state = False
      break
  
    os.system("clear")
    print("Color Sensor RAW:", pendulum_sensor.value())

  timeCounter = 0
  prevTime = time.time()
  hits = []

  while True:
    # ----------- Time Handling ---------------
    currTime = time.time()
    timeCounter += (currTime - prevTime)
    prevTime = currTime
    # -----------------------------------------
  
    if (pendulum_sensor.value() >= eq_state_light):
      hits.append(timeCounter)
      print("Hit RAW:", timeCounter)
      time.sleep(0.5)
      if (len(hits) > 10):
        break
  
  os.system("clear")
  firstHit = hits[0]
  print("-------- DONE --------")
  print("Hits shifted by the first hit value...")
  for x in range(1, len(hits)):
    hits[x] -= firstHit
    print("Hit:", hits[x])
  
  halfPeriod = hits[1] - hits[0]
  for x in range(1, len(hits) - 1):
    halfPeriod = (halfPeriod + (hits[x + 1] - hits[x])) / 2
  
  period = halfPeriod * 2
  PI = 3.1415
  length = period*period*9.81/(4*PI*PI)

  print("Average Period:", bcolors.OKGREEN, period, bcolors.ENDC, "s")
  print("Length of the pendulum is", bcolors.OKGREEN, length, bcolors.ENDC, "m")
  print("Length of the pendulum is", bcolors.OKGREEN, length * 100, bcolors.ENDC, "cm")
  
  input("Press any key to restart the experiment... ")

loop()
