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
from navio2ros.msg import IMU

#RC channel
veh=Vehicle()
rcin = RC()
pwmout = PWM()
motor0 = 0.0
motor1 = 0.0
motor2 = 0.0
motor3 = 0.0



#outval= [motor0,motor1,motor2,motor3]
roll_channel_in = 0
pitch_channel_in = 1
throttle_channel_in = 2
yaw_channel_in = 3
kill_channel_in = 4
idle = 1400.0
roll_channel_values= [1004.0,1995.0,1497.5]
pitch_channel_values = [1000.0,1999.0,1499.0]
throttle_channel_values = [1010.0,1996.0,1480.0]
yaw_channel_values = [1001.0,1985.0,1500.5]



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
	#veh.imu.accelerometer.x = data.accelerometer.x
	#veh.imu.accelerometer.y = data.accelerometer.y
	#veh.imu.accelerometer.z = data.accelerometer.z
	veh.imu.gyroscope.x = data.gyroscope.x
	veh.imu.gyroscope.y = data.gyroscope.y
	veh.imu.gyroscope.z = data.gyroscope.z
	#print(veh.imu.gyroscope.z)
	

def callback_angular(data):
	global roll
	global pitch
	global yaw
	global veh
	roll = data.angular.roll 
	pitch = data.angular.pitch
	#print(pitch,roll)

pitchI = 0
PlastError = 0
PlastTime = time.time()

def pitchPID(p,i,d):
        #this section sets values to start PID calculations
        global pid_pitch
	global pitchI, PlastError, PlastTime
        desiredPitch = (rc_pitch * 40.0)
        PcurrentTime = time.time()
        Perror = desiredPitch - pitch
        PdeltaTime = PcurrentTime - PlastTime
        PdeltaError = Perror - PlastError
        pitchI += ((PlastError+Perror)/2) * PdeltaTime
        pitchD = PdeltaError/PdeltaTime
        pid_pitch = (p *Perror) + (i * pitchI) + (d * pitchD)
        PlastError = Perror
        PlastTime = PcurrentTime
        #print(PlastTime,PlastError,PcurrentTime,Perror,pitchI,pitchD,pid_pitch)

RollI = 0
RlastError = 0
RlastTime = time.time()

def RollPID(p,i,d):
#def RollPID(p,i,d, RlastTime = 0,RollI = 0.0,RlastError=0):

        #this section sets values to start PID calculations
        global pid_Roll

	global RollI, RlastError, RlastTime

#	print(RlastTime, RollI, RlastError)

        desiredRoll = (rc_roll * 40.0)
        RcurrentTime = time.time()
        Rerror = desiredRoll - roll
        RdeltaTime = RcurrentTime - RlastTime
        RdeltaError = Rerror - RlastError
        RollI += ((RlastError+Rerror)/2) * RdeltaTime
        RollD = RdeltaError/RdeltaTime
        pid_Roll = (p *Rerror) + (i * RollI) + (d * RollD)

#	print(RcurrentTime,RlastTime,RdeltaTime)

        RlastError = Rerror
        RlastTime = RcurrentTime
        #print(desiredRoll,roll,RollD,RollI)
	#print(p,i,d,Rerror,RollI,RollD,p*Rerror,i*RollI,d*RollD)

#	print(RollI,RlastError,Rerror)


YawI = 0
YlastError = 0
YlastTime = time.time()

def YawPID(p,i,d):
        #this section sets values to start PID calculations
        global pid_Yaw, YawI, YlastError, YlastTime
        #YawI=0
        #Yend=200
        #Yi = 1
        #for Yi in range(1,Yend):
        global pid_Yaw
        currentYawRate = veh.imu.gyroscope.z - .018
        desiredYawRate = (rc_yaw * 120)
        YcurrentTime = time.time()
        yawError = desiredYawRate - currentYawRate
        YdeltaTime = YcurrentTime - YlastTime
        YdeltaError = yawError - YlastError
        YawI += ((YlastError+yawError)/2) * YdeltaTime
        YawD = YdeltaError/YdeltaTime
        pid_Yaw = (p * yawError) + (i * YawI) + (d * YawD)
        YlastError = yawError
        YlastTime = YcurrentTime

	#print(currentYawRate)

	#print(p,i,d,yawError,YawI,YawD,p*yawError,i*YawI,d*YawD)
	#print(YdeltaTime)

	print(yawError)



def rc_commands(): #rc commands linear approximations
        global rc_roll
        global rc_pitch
        global rc_throttle
        global rc_yaw
        rc_roll = (roll_channel - roll_channel_values[2])/(roll_channel_values[1]-roll_channel_values[0])
        rc_pitch = -(pitch_channel - pitch_channel_values[2])/(pitch_channel_values[1]-roll_channel_values[0])
        #rc_throttle = ((1480 if throttle_channel > 1460 and throttle_channel < 1500 else throttle_channel) - throttle_channel_values[2])/(throttle_channel_values[1]-throttle_channel_values[0]) 
        rc_yaw = (yaw_channel - yaw_channel_values[2])/(yaw_channel_values[1]-yaw_channel_values[0])
	#print(rc_yaw)

def motor():
        global motor0
        global motor1
        global motor2
        global motor3
        base = throttle_channel
        #front motor CCW
        motor0=(base + pid_pitch + pid_Yaw)/1000 if ((base + pid_pitch + pid_Yaw)/1000) <=2.0 else 2.0
        #rear motor (opposite of front motor) CCW
        motor1=(base - pid_pitch + pid_Yaw)/1000 if ((base - pid_pitch + pid_Yaw)/1000) <=2.0 else 2.0
        #left motor CW
        motor2= (base + pid_Roll - pid_Yaw)/1000 if ((base + pid_Roll - pid_Yaw)/1000) <=2.0 else 2.0
        #right motor CW
        motor3= (base - pid_Roll - pid_Yaw)/1000 if ((base - pid_Roll - pid_Yaw)/1000) <=2.0 else 2.0
        #print(pid_Roll)

sub = rospy.Subscriber('/rcpub',RC,callback_rc, queue_size=10)
sub = rospy.Subscriber('/imumpupub', IMU, callback_imu)
pub = rospy.Publisher('/motorcommand',PWM, queue_size=10)
subdata = rospy.Subscriber('/madgwickpub', AHRS, callback_angular, queue_size=3)
rospy.init_node('MotorCommand', anonymous=True) # register the node

if __name__ == '__main__':
    try:

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

                rc_commands()


                pitchPID(2.0,0.0,0.5) #P,I,D
                RollPID(2.0,0.0,0.5) #P,I,D

		#pitchPID(0,0,0)
		#RollPID(0,0,0)

                YawPID(2.0,.0,.3) #P,I,D

                motor()


                if kill_channel >= 1200: #in case things go wild
                      for i in range(0,4):
				pwmout.channel[i] = 1.0

               	else:
			pwmout.channel[0] = motor0 if motor0 >= 1.0 else 1.0
	                pwmout.channel[1] = motor1 if motor1 >= 1.0 else 1.0
	                pwmout.channel[2] = motor2 if motor2 >= 1.0 else 1.0
	                pwmout.channel[3] = motor3 if motor3 >= 1.0 else 1.0
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
