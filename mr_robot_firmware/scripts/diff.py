#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from math import pi



def stop():
    #print('stopping')
    lapwm.ChangeDutyCycle(0)
    lbpwm.ChangeDutyCycle(0)
    rapwm.ChangeDutyCycle(0)
    rbpwm.ChangeDutyCycle(0)

def get_pwm(left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    lspeedPWM = min((left_speed/max_speed)*max_pwm_val, max_pwm_val)
    rspeedPWM = min((right_speed/max_speed)*max_pwm_val, max_pwm_val)

    return abs(lspeedPWM), abs(rspeedPWM)
    
def callback(data):
    global wheel_radius
    global wheel_separation
   
    linear_vel = data.linear.x                  # Linear Velocity of Robot
    angular_vel = data.angular.z                # Angular Velocity of Robot
    
    print('linear and angular: ', linear_vel, angular_vel)
    right_vel = linear_vel + (angular_vel * wheel_separation) / 2       # right wheel velocity
    left_vel  = linear_vel - (angular_vel * wheel_separation) / 2              # left wheel velocity
    print('left speed and right speed: ', left_vel, right_vel)

    l_pwm, r_pwm = get_pwm(left_vel, right_vel)

    print('left pwm and right pwm: ', l_pwm, r_pwm)
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
        print("stopping")

    elif (left_vel >= 0.0 and right_vel >= 0.0):
        lapwm.ChangeDutyCycle(l_pwm)
        lbpwm.ChangeDutyCycle(0)
        rapwm.ChangeDutyCycle(r_pwm)
        rbpwm.ChangeDutyCycle(0)
        print("moving forward")

    elif (left_vel <= 0.0 and right_vel <= 0.0):

        lapwm.ChangeDutyCycle(0)
        lbpwm.ChangeDutyCycle(l_pwm)
        rapwm.ChangeDutyCycle(0)
        rbpwm.ChangeDutyCycle(r_pwm)
        print("moving backward")

    elif (left_vel < 0.0 and right_vel > 0.0):
        lapwm.ChangeDutyCycle(0)
        lbpwm.ChangeDutyCycle(l_pwm)
        rapwm.ChangeDutyCycle(r_pwm)
        rbpwm.ChangeDutyCycle(0)
        print("turning left")

    elif (left_vel > 0.0 and right_vel < 0.0):
        lapwm.ChangeDutyCycle(l_pwm)
        lbpwm.ChangeDutyCycle(0)
        rapwm.ChangeDutyCycle(0)
        rbpwm.ChangeDutyCycle(r_pwm)
        print("turning right")
        
    else:
        stop()
        
def listener():
    print("initializing...")
    rospy.init_node('cmdvel_listener', anonymous=False)
    print("initialized")
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

motor_rpm = 100              #   max rpm of motor on full voltage 
wheel_diameter = 0.095      #   in meters
wheel_separation = 0.22     #   in meters
max_pwm_val = 100           #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 30            #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec

left_a = 12
left_b = 13
right_a = 21
right_b = 22
left_en = 6
right_en = 26

GPIO.setmode(GPIO.BCM)
#GPIO.cleanup()

GPIO.setup(left_a, GPIO.OUT)
GPIO.setup(left_b, GPIO.OUT)
GPIO.setup(right_a, GPIO.OUT)
GPIO.setup(right_b, GPIO.OUT)
GPIO.setup(left_en, GPIO.OUT)
GPIO.setup(right_en, GPIO.OUT)

lapwm = GPIO.PWM(left_a, 1000)
lbpwm = GPIO.PWM(left_b, 1000)
rapwm = GPIO.PWM(right_a, 1000)
rbpwm = GPIO.PWM(right_b, 1000)

lapwm.start(0)
lbpwm.start(0)
rapwm.start(0)
rbpwm.start(0)

try:
    print('MR_Robot Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    listener()

except KeyboardInterrupt:
    print("keyboard intrupt")

except:
    print("Other error or exception occurred!")

finally:
    print("Cleaning up GPIO")
    GPIO.cleanup() 