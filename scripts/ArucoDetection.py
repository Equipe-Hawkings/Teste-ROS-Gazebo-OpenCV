#!/usr/bin/env python3

''' Detecção de Aruco por meio da câmera do drone iris utilizado dentro do Gazebo.
	Recolhe-se os frames da stream por meio de topics específicos do ROS. '''

''' Código por enquanto inutilizável'''


from __future__ import print_function

import roslib
roslib.load_manifest('testes_cv')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
import imutils
import time




class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/iris/cam_frontal/image_raw",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris/cam_frontal/image_raw",Image,self.callback)

    def callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def detectingAruco():
        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-t", "--type", type=str,
	    default="DICT_ARUCO_ORIGINAL",
	    help="type of ArUCo tag to detect")
        args = vars(ap.parse_args())

        # define names of each possible ArUco tag OpenCV supports
        ARUCO_DICT = {
	        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)
# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()





def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    ic.

  
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
