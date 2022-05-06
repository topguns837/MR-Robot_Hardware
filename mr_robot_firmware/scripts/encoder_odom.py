#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep
import rospy
import std_msgs.msg import Int16


lft_counter = 0
rgt_counter = 0

lft_Enc_A = 17
lft_Enc_B = 27

rgt_Enc_A = 18
rgt_Enc_B = 28

rospy.init_node("EncoderOdom", anonymous=True)

right_pub = rospy.Publisher('right_count', Int16, queue_size=10)
left_pub = rospy.Publisher('left_count', Int16, queue_size=10)


def right_callback(r_data):
    rospy.loginfo("Right_Wheel_ticks = %d\n", r_data.data)

def left_callback(l_data):
    rospy.loginfo("Left_Wheel_ticks = %d\n", l_data.data)

def init():
    print("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    right_pub.publish(rgt_counter)
    left_pub.publish(lft_counter)

    GPIO.setup(lft_Enc_A, GPIO.IN)
    GPIO.setup(lft_Enc_B, GPIO.IN)
    GPIO.setup(rgt_Enc_A, GPIO.IN)
    GPIO.setup(rgt_Enc_B, GPIO.IN)

    rospy.Subscriber("right_count", Int16, right_callback)
    rospy.Subscriber("left_count", Int16, left_callback)

    GPIO.add_event_detect(lft_Enc_A, GPIO.RISING, callback=lft_rotation_decode, bouncetime=10)
    GPIO.add_event_detect(rgt_Enc_A, GPIO.RISING, callback=rgt_rotation_decode, bouncetime=10)

    return


def lft_rotation_decode(lft_Enc_A):
    global lft_counter
    sleep(0.002)
    val_A = GPIO.input(lft_Enc_A)
    val_B = GPIO.input(lft_Enc_B)

    if (val_A == 1) and (val_B == 0):
        lft_counter += 1
        print("direction -> ", lft_counter)
        while val_B == 0:
            val_B = GPIO.input(lft_Enc_B)
        while val_B == 1:
            val_B = GPIO.input(lft_Enc_B)
        return

    elif (val_A == 1) and (val_B == 1):
        lft_counter -= 1
        print("direction <- ", lft_counter)
        while val_A == 1:
            val_A = GPIO.input(lft_Enc_A)
        return
    else:
        return


def rgt_rotation_decode(rgt_Enc_A):
    global rgt_counter
    sleep(0.002)
    val_A = GPIO.input(rgt_Enc_A)
    val_B = GPIO.input(rgt_Enc_B)

    if (val_A == 1) and (val_B == 0):
        rgt_counter += 1
        print("direction -> ", rgt_counter)
        while val_B == 0:
            val_B = GPIO.input(rgt_Enc_B)
        while val_B == 1:
            val_B = GPIO.input(rgt_Enc_B)
        return

    elif (val_A == 1) and (val_B == 1):
        rgt_counter -= 1
        print("direction <- ", rgt_counter)
        while val_A == 1:
            val_A = GPIO.input(lft_Enc_A)
        return
    else:
        return


def main():


    try:
        init()
        while True:
            sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == '__main__':
    main()