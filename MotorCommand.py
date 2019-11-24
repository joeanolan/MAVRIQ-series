#!/usr/bin/env python
#
#
#
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

def callback_imu(data):
	global veh
	veh.imu.accelerometer.x = data.accelerometer.x
	veh.imu.accelerometer.y = data.accelerometer.y
	veh.imu.accelerometer.z = data.accelerometer.z
	veh.imu.gyroscope.x = data.gyroscope.x
	veh.imu.gyroscope.y = data.gyroscope.y
	veh.imu.gyroscope.z = data.gyroscope.z

def callback_angular(data):
	global roll
	global pitch
	global yaw
	roll = data.angular.roll
	pitch = data.angular.pitch
	yaw = data.angular.yaw

def pitchPID(p,i,d, PcurrentTime = None):
        #this section sets values to start PID calculations
        global pid_pitch
        currentPitch = pitch
        PcurrentTime= PcurrentTime if PcurrentTime is not None else time.time()
        desiredPitch = pitch*(rc_pitch)
        Perror = desiredPitch - currentPitch
        PlastError = Perror
        PlastTime = PcurrentTime
        PitchI=0
        Pend=200
        pi=1
        while pi in range(1,Pend):
                global pid_pitch
                time.sleep(0.02) #50hz hold
                currentPitch = pitch
                desiredPitch = currentPitch*(rc_pitch)
                PcurrentTime = time.time()
                Perror = desiredPitch - currentPitch
                PdeltaTime = PcurrentTime - PlastTime
                PdeltaError = Perror - PlastError
                pitchI = Perror * PdeltaTime
                pitchD = PdeltaError/PdeltaTime
                pid_pitch = (p *Perror) + (i * pitchI) + (d * pitchD)
                PlastError = Perror
                PlastTime = PcurrentTime
                Pend+=1
                print pid_pitch
                

def RollPID(p,i,d, RcurrentTime = None):
        #this section sets values to start PID calculations
        global pid_Roll
        currentRoll = roll
        RcurrentTime= RcurrentTime if RcurrentTime is not None else time.time()
        desiredRoll = roll*(rc_roll)
        Rerror = desiredRoll - currentRoll
        RlastError = Rerror
        RlastTime = RcurrentTime
        RollI=0
        Rend=200
        Ri = 1
        while Ri in range(1,Rend):
                global pid_Roll
                time.sleep(0.02) #50hz hold
                currentRoll = roll
                desiredRoll = currentRoll*(rc_roll)
                RcurrentTime = time.time()
                Rerror = desiredRoll - currentRoll
                RdeltaTime = RcurrentTime - RlastTime
                RdeltaError = Rerror - RlastError
                RollI = Rerror * RdeltaTime
                RollD = RdeltaError/RdeltaTime
                pid_Roll = (p *Rerror) + (i * RollI) + (d * RollD)
                RlastError = Rerror
                RlastTime = RcurrentTime
                Rend+=1
                
                

def YawPID(p,i,d, YcurrentTime = None):
        #this section sets values to start PID calculations
        global pid_Yaw
        currentYaw = yaw
        YcurrentTime= YcurrentTime if YcurrentTime is not None else time.time()
        desiredYaw = yaw*(rc_yaw)
        Yerror = desiredYaw - currentYaw
        YlastError = Yerror
        YlastTime = YcurrentTime
        YawI=0
        Yend=200
        Yi = 1
        while Yi in range(1,Yend):
                global pid_Yaw
                time.sleep(0.02) #50hz hold
                currentYaw = yaw
                desiredYaw = currentYaw*(rc_yaw)
                YcurrentTime = time.time()
                Yerror = desiredYaw - currentYaw
                YdeltaTime = YcurrentTime - YlastTime
                YdeltaError = Yerror - YlastError
                YawI = Yerror * YdeltaTime
                YawD = YdeltaError/YdeltaTime
                pid_Yaw = (p * Yerror) + (i * YawI) + (d * YawD)
                YlastError = Yerror
                YlastTime = YcurrentTime
                Yend+=1
                

def motor():
        rc_commands()
        pitchPID(0,0,0) #P,I,D
        RollPID(0,0,0) #P,I,D
        YawPID(0,0,0) #P,I,D
        i = 1
        end = 200
        for i in range(1,end):
                base = rc_throttle + idle
                #front motor CCW
                motor0= base + pid_pitch - pid_Roll + pid_Yaw
                #rear motor (opposite of front motor) CCW
                motor2= base - pid_pitch + pid_Roll - pid_Yaw
                #left motor CW
                motor1= base + pid_pitch + pid_Roll + pid_Yaw
                #right motor CW
                motor3= base - pid_pitch - pid_Roll - pid_Yaw
                print(motor0,motor1,motor2,motor3)
                i+=1

def rc_commands():
        global rc_roll
        global rc_pitch
        global rc_throttlesudo
        global rc_yaw
        rci = 1
        rcend = 200
        for rci in range(rci,rcend):
                rc_roll = (roll_channel - roll_channel_values[2])/(roll_channel_values[1]-roll_channel_values[0])
                rc_pitch = (pitch_channel - pitch_channel_values[2])/(pitch_channel_values[1]-roll_channel_values[0])
                rc_throttle = ((1480 if throttle_channel > 1470 and throttle_channel < 1490 else throttle_channel) - throttle_channel_values[2])/(throttle_channel_values[1]-throttle_channel_values[0]) #since the throttle center isnt self-centering giving a small range of throttle idle helps make flight easier
                rc_yaw = (yaw_channel - yaw_channel_values[2])/(yaw_channel_values[1]-yaw_channel_values[0])
                idle = 1.0
                rcend += 1



    
sub = rospy.Subscriber('/rcpub',RC,callback_rc, queue_size=10)
pub = rospy.Publisher('/motorcommand',PWM, queue_size=10)
subdata = rospy.Subscriber('/madgwickpub', AHRS, callback_angular, queue_size=3)
rospy.init_node('MotorCommand', anonymous=True) # register the node

roll_channel_values= [1000.0,1996.0,1495.0]
pitch_channel_values = [1000.0,1999.0,1499.0]
throttle_channel_values = [1010.0,1996.0,1480.0]
yaw_channel_values = [1001.0,1991.0,1500.0]

#rc commands linear approximations



if __name__ == '__main__':
    try: 
        while not rospy.is_shutdown():
                motor()
                rate = rospy.Rate(50)
                if kill_channel >= 1200: #in case things go wild
                        while kill_channel >= 1200:  
                            pwmout.channel[i] = 1.0
                for i in range(len(pwmout.channel)):
                        pwmout.channel[0] = outval[0]/1000.0
                        pwmout.channel[1] = outval[1]/1000.0
                        pwmout.channel[2] = outval[2]/1000.0
                        pwmout.channel[3] = outval[3]/1000.0
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
