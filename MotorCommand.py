#!/usr/bin/env python



import os
import sys
import rospy
import numpy
import time
import navio2ros.madgwickpubAHRS
import math
import navio/Util.h as util
import navio2ros.leds as led

# import custom messages that we will have to use (Shawn's Code)
from navio2ros.msg import RC # for reading in RC values from TX
from navio2ros.msg import PWM # for outputting values to the servo rail

#RC channel
rcin = 0
pwmout = PWM()
motor0 = 0 
motor1 = 0
motor2 = 0
motor3 = 0
outval= [motor0,motor1,motor2,motor3]
roll_channel_in = 0
pitch_channel_in = 1
throttle_channel_in = 2
yaw_channel_in = 3
kill_channel_in = 4

class PID: #https://github.com/ivmech/ivPID/blob/master/PID.py
    """PID Controller
    """
    def __init__(self, P, I, D, current_time=None):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
    def update(self, feedback_value, current_time=None):
        error = self.SetPoint - feedback_value
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain
    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain
    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain
    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup
    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def rc_calibration(): #idea is to use linear approx for rc inputs however if an rc is changed or replaced this allows for quick calibration
        if rcin[0] == 0:
            while rcin[0] == 0: #looping while no RC information
                #led.setColor('Red')
                time.sleep(.5)
                #led.setColor('Yellow')
                time.sleep(.5)
            #led.setColor('Green')
            rc_roll_init=rcin[0]
            rc_pitch_init=rcin[1]
            rc_throttle_init=1500 #this locks idle position in center stick location
            rc_yaw_init=rcin[3]
            global roll_channel
            global pitch_channel
            global throttle_channel
            global yaw_channel
            #led.setColor('Green')
            timer=time.time()+3
            while time < timer:
                roll_channel= [rcin[0],min(rcin[0]),max(rcin[0]),rc_roll_init]
                pitch_channel = [rcin[1],min(rcin[1]),max(rcin[1]),rc_pitch_init]
                throttle_channel = [rcin[2],min(rcin[2]),max(rcin[2]),rc_throttle_init]
                yaw_channel = [rcin[3],min(rcin[3]),max(rcin[3]), rc_yaw_init]
                #led.setColor('Blue')
    
def callback_rc(data):
	global rcin
	rcin[0]=data.channel[roll_channel_in]
	rcin[1]=[pitch_channel_in]
	rcin[2]=[throttle_channel_in]
	rcin[3]=[yaw_channel_in]
	rcin[4]=[kill_channel_in]

sub = rospy.Subscriber('/rcpub',RC,callback_rc, queue_size=10)
pub = rospy.Publisher('/motorcommand',PWM, queue_size=10)
rospy.init_node('MotorCommand', anonymous=True) # register the node

#IMU setup
imu=IMU.MPU9250()
accel, gyro, mag = imu.getMotion9()
pitch_base = math.atan2(accel[1],accel[2])
roll_base = math.atan2(accel[0],accel[2])
throttle_base = 0
yaw_base = math.atan2((-mag[1] * math.cos(roll_base) + mag[2] * math.sin(roll)),(mag[0] * math.cos(pitch_base) + mag[1] * math.sin(roll_base) + mag[2] * math.sin(pitch_base) * math.cos(roll_base)))

#rc commands linear approximations
rc_roll = (roll_channel[0] - roll_channel[3])/(roll_channel[2]-roll_channel[2])
rc_pitch = (pitch_channel[0] - pitch_channel[3])/(pitch_channel[2]-pitch_channel[2])
rc_throttle = ((1500 if throttle_channel > 1480 and throttle_channel < 1520 else throttle_channel[0]) - throttle_channel[3])/(throttle_channel[2]-throttle_channel[2]) #since the throttle center isnt self-centering giving a small range of throttle idle helps make flight easier
rc_yaw = (yaw_channel[0] - yaw_channel[3])/(yaw_channel[2]-yaw_channel[2])

#pid call from PID - this is where you change PID coeffecients 
pid_throttle= PID(throttle_base, P=1,I=1,D=0)
pid_pitch= PID(pitch_base,P=1,I=0,D=0)
pid_roll= PID(roll_base, P=1,I=0,D=0)
pid_yaw = PID(yaw_base, P=1, I=0, D=0)

idle = 0
pitch = pitch_base + pid_pitch + rc_pitch
roll = roll_base + pid_roll +rc_roll
base = throttle_base + idle
yaw = yaw_base + pid_yaw + rc_yaw

#front motor CCW
motor0= base + pitch - roll + yaw
#rear motor (opposite of front motor) CCW
motor2= base - pitch + roll - yaw
#left motor CW
motor1= base + pitch + roll + yaw
#right motor CW
motor3= base - pitch - roll - yaw

if __name__ == '__main__':
    try: # try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
        while not rospy.is_shutdown():
            rate = rospy.Rate(50)
            if rcin[0] == 0:
               rc_calibration()
            if kill_channel >= 1200: #in case things go wild
                while kill_channel >= 1200:  
                    pwmout.channel[i] = 1.0
                    continue #brings us back to start of while loop
            for i in range(len(pwmout.channel)):
                pwmout.channel[i] = outval/1000.0 # rc values are integers (1000-2000), we want 1.0-2.0
                if rcin[0] == 0:
                    rc_calibration
                    
            # publish the topic to motor command
            pub.publish(pwmout)

            # this is ros magic, basically just a sleep function with the specified dt
            rate.sleep()

    # as stated before, try/except is used to nicely quit the program using ctrl+c
    except rospy.ROSInterruptException:
        for i in range(len(pwmout.channel)): # before shutting down, turn all outputs back to 1 for safety
            pwmout.channel[i] = 1.0
        pub.publish(pwmout) # publish the topic before closing

        pass
