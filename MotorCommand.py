#!/usr/bin/env python



import os
import sys
import rospy
import numpy
import time
import math


# import custom messages that we will have to use (Shawn's Code)
from navio2ros.msg import AHRS
from navio2ros.msg import RC # for reading in RC values from TX
from navio2ros.msg import PWM # for outputting values to the servo rail
from navio2ros.msg import Vehicle


#RC channel
veh=Vehicle()
rcin = RC()
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


"""def rc_calibration(): #idea is to use linear approx for rc inputs however if an rc is changed or replaced this allows for quick calibration
        if rcin[0] == 0:
            while rcin[0] == 0: #looping while no RC information
                #led.setColor('Red')
                time.sleep(.5)
                #led.setColor('Yellow')
                time.sleep(.5)
            #led.setColor('Green')
            rc_roll_init=roll_channel
            rc_pitch_init=rcin[1]
            rc_throttle_init=1500 #this locks idle position in center stick location
            rc_yaw_init=rcin[3]
            #led.setColor('Green')
            timer=time.time()+3
            while time < timer:
                
                #led.setColor('Blue')"""
    
def callback_rc(data):
	global roll_channel
	global pitch_channel
	global throttle_channel
	global yaw_channel
	global kill_channel
	roll_channel=data.channel[roll_channel_in]
	pitch_channel=data.channel[pitch_channel_in]
	throttle_channel=data.channel[throttle_channel_in]
	yaw_channel=data.channel[yaw_channel_in]
	kill_channel=data.channel[kill_channel_in]
	print "callback_rc"


def callback_imu(data):
	global veh
	global yaw
	veh.imu.accelerometer.x = data.accelerometer.x
	veh.imu.accelerometer.y = data.accelerometer.y
	veh.imu.accelerometer.z = data.accelerometer.z
	veh.imu.gyroscope.x = data.gyroscope.x
	veh.imu.gyroscope.y = data.gyroscope.y
	veh.imu.gyroscope.z = data.gyroscope.z
	print "callback_imu"

def callback_angular(data):
	global roll
	global pitch
	global yaw
	roll = data.angular.roll
	pitch = data.angular.pitch
	yaw = data.angular.yaw
	print "roll and pitch"

sub = rospy.Subscriber('/rcpub',RC,callback_rc, queue_size=10)
pub = rospy.Publisher('/motorcommand',PWM, queue_size=10)
subdata = rospy.Subscriber('/madgwickpub', AHRS, callback_angular, queue_size=3)
rospy.init_node('MotorCommand', anonymous=True) # register the node

roll_channel_values= [1000.0,1996.0,1495.0]
pitch_channel_values = [1000.0,1999.0,1499.0]
throttle_channel_values = [1010.0,1996.0,1480.0]
yaw_channel_values = [1001.0,1991.0,1500.0]

#rc commands linear approximations
rc_roll = (roll_channel - roll_channel_values[2])/(roll_channel_values[1]-roll_channel_values[0])
rc_pitch = (pitch_channel - pitch_channel_values[2])/(pitch_channel_values[1]-roll_channel_values[0])
rc_throttle = ((1480 if throttle_channel > 1470 and throttle_channel < 1490 else throttle_channel) - throttle_channel_values[2])/(throttle_channel_values[1]-throttle_channel_values[0]) #since the throttle center isnt self-centering giving a small range of throttle idle helps make flight easier
rc_yaw = (yaw_channel - yaw_channel_values[2])/(yaw_channel_values[1]-yaw_channel_values[0])

#pid call from PID - this is where you change PID coeffecients 
pid_pitch= PID(pitch,1,0,0)
pid_roll= PID(roll,1,0,0)
pid_yaw = PID(yaw,1,0,0)
idle = 1.0
base = throttle_channel + idle
#front motor CCW
motor0= base + pid_pitch - pid_roll + pid_yaw
#rear motor (opposite of front motor) CCW
motor2= base - pid_pitch + pid_roll - pid_yaw
#left motor CW
motor1= base + pid_pitch + pid_roll + pid_yaw
#right motor CW
motor3= base - pid_pitch - pid_roll - pid_yaw

if __name__ == '__main__':
    try: # try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
        while not rospy.is_shutdown():
            rate = rospy.Rate(50)
            if kill_channel >= 1200: #in case things go wild
                while kill_channel >= 1200:  
                    pwmout.channel[i] = 1.0
                    continue #brings us back to start of while loop
            for i in range(len(pwmout.channel)):
                pwmout.channel[i] = outval/1000.0 # rc values are integers (1000-2000), we want 1.0-2.0
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
