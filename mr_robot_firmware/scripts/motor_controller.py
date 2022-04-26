#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy

class Robot_Controller:
    #initialised values
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.Joy_callback)

        self.LA_Pin = 20
        self.LB_Pin = 16

        self.RA_Pin = 6
        self.RB_Pin = 5

        self.ya_axis = 0
        self.yb_axis = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LA_Pin, GPIO.OUT)
        GPIO.setup(self.LB_Pin, GPIO.OUT)
        GPIO.setup(self.RA_Pin, GPIO.OUT)
        GPIO.setup(self.RB_Pin, GPIO.OUT)

        self.lapwm = GPIO.PWM(self.LA_Pin, 1000)
        self.lbpwm = GPIO.PWM(self.LB_Pin, 1000)
        self.rapwm = GPIO.PWM(self.RA_Pin, 1000)
        self.rbpwm = GPIO.PWM(self.RB_Pin, 1000)

        self.lapwm.start(0)
        self.lbpwm.start(0)
        self.rapwm.start(0)
        self.rbpwm.start(0)

        GPIO.output(self.LA_Pin, GPIO.LOW)
        GPIO.output(self.LB_Pin, GPIO.LOW)
        GPIO.output(self.RA_Pin, GPIO.LOW)
        GPIO.output(self.RB_Pin, GPIO.LOW)
        

        print("starting node")

    def Joy_callback(self, data):
        axes = data.axes
        buttons = data.buttons
        self.button_a = buttons[0]
        self.ya_axis = axes[1]
        self.yb_axis = axes[4]
        self.provide_pwm()

    def left_pwm(self):
        if self.ya_axis >= 0:
            print("f = " + str(self.ya_axis))
            self.lapwm.ChangeDutyCycle(abs(self.ya_axis) * 100)
            self.lbpwm.ChangeDutyCycle(0)
        else:
            print("b = " + str(self.ya_axis))
            self.lapwm.ChangeDutyCycle(0)
            self.lbpwm.ChangeDutyCycle(abs(self.ya_axis) * 100)

    def right_pwm(self):
        if self.yb_axis >= 0:
            print("f = " + str(self.yb_axis))
            self.rapwm.ChangeDutyCycle(abs(self.yb_axis) * 100)
            self.rbpwm.ChangeDutyCycle(0)
        else:
            print("b = " + str(self.yb_axis))
            self.rapwm.ChangeDutyCycle(0)
            self.rbpwm.ChangeDutyCycle(abs(self.yb_axis) * 100)

    def provide_pwm(self):
        if self.ya_axis != 0:
            self.left_pwm()

        if self.ya_axis == 0:
            print("left stopping")
            self.lapwm.ChangeDutyCycle(0)
            self.lbpwm.ChangeDutyCycle(0)

        if self.yb_axis != 0:
            self.right_pwm()

        if self.yb_axis == 0:
            print("right stopping")
            self.rapwm.ChangeDutyCycle(0)
            self.rbpwm.ChangeDutyCycle(0)

        if self.button_a != 0:
            GPIO.output(self.LA_Pin, GPIO.LOW)
            GPIO.output(self.LB_Pin, GPIO.LOW)
            GPIO.cleanup()
            print("clearing gpio")

if __name__ == "__main__":
    Robot = Robot_Controller()
    rospy.spin()