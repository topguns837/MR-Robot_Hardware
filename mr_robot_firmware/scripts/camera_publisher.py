#! /usr/bin/env python3
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        rospy.init_node('Camera_Publisher')

        self.pub = rospy.Publisher('video_frames', Image, queue_size=10)
        self.br = CvBridge()

        self.rate = rospy.Rate(10)
        self.publish_message()
    
    def publish_message(self):

        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        raw_capture = PiRGBArray(camera, size=(640, 480))

        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):

            image = frame.array
            #rospy.loginfo('publishing video frame')

            self.pub.publish(self.br.cv2_to_imgmsg(image, "bgr8"))

            key = cv2.waitKey(1) & 0xFF
            raw_capture.truncate(0)
            if key == ord("q"):
                break

            self.rate.sleep()


if __name__ == '__main__':
    Camera = CameraPublisher()
