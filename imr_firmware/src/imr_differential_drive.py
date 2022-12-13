#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time
from math import pi

leftEn = 33        #   PWM2
rightEn = 32        #   PWM1


leftForward = 35    #   DIR2
rightForward = 37   #   DIR1


motor_rpm = 40           #   max rpm of motor on full voltage 
wheel_diameter = 0.15      #   in meters
wheel_separation = 0.45     #   in meters

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)


pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)

def stop():
    #print('stopping')
    pwmL.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(rightForward, GPIO.HIGH)

def forward(left_speed, right_speed):
    #print('going forward')
    lspeedPWM = min(((left_speed/max_speed)*100),100)
    rspeedPWM = min(((right_speed/max_speed)*100),100)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)


def backward(left_speed, right_speed):
    #print('going backward')
    lspeedPWM = min(((left_speed/max_speed)*100),100)
    rspeedPWM = min(((right_speed/max_speed)*100),100)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)


def left(left_speed, right_speed):
    #print('turning left')
    lspeedPWM = min(((left_speed/max_speed)*100),100)
    rspeedPWM = min(((right_speed/max_speed)*100),100)
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.HIGH)
    

def right(left_speed, right_speed):
    #print('turning right')
    lspeedPWM = min(((left_speed/max_speed)*100),100)
    rspeedPWM = min(((right_speed/max_speed)*100),100)
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.LOW)

    
def callback(data):
    
    global wheel_radius
    global wheel_separation
    
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    #print(str(linear)+"\t"+str(angular))
    
    rplusl  = ( 2 * linear_vel ) / wheel_radius
    rminusl = ( angular_vel * wheel_separation ) / wheel_radius
    
    right_omega = ( rplusl + rminusl ) / 2
    left_omega  = rplusl - right_omega 
    
    right_vel = right_omega * wheel_radius
    left_vel  = left_omega  * wheel_radius
    
    #print (str(left_vel)+"\t"+str(right_vel))
    '''
    left_speed  = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    right_speed = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    '''
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
    elif (left_vel >= 0.0 and right_vel >= 0.0):
        forward(abs(left_vel), abs(right_vel))
    elif (left_vel <= 0.0 and right_vel <= 0.0):
        backward(abs(left_vel), abs(right_vel))
    elif (left_vel < 0.0 and right_vel > 0.0):
        left(abs(left_vel), abs(right_vel))
    elif (left_vel > 0.0 and right_vel < 0.0):
        right(abs(left_vel), abs(right_vel))
    else:
        stop()
        
def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('IMR Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    listener()
