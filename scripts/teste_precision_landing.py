#!/usr/bin/env python3

from socket import IP_MULTICAST_LOOP
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import math
import time
import sys
import imutils
import numpy as np
from drone_control.motion_control import MotionControl 

# Define uma classe para poder utilizar as funcoes da MotionControl
class Drone(MotionControl):

    def __init__(self):
        MotionControl.__init__(self)
        self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
        self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris/cam_ventral/image_raw", Image, self.callback_cvbridge)

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
        if ARUCO_DICT.get("DICT_4X4_50", None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format("DICT_4X4_50"))
            sys.exit(0)
    
    def callback_cvbridge(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
       
        markers_img, ids_list = self.pose_estimation(self.cv_image)

        if ids_list is None:
            self.id_pub.publish(ids_list)
        else:
            ids_str = ''.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
        

    def detect_aruco(self,img):
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.corners, self.ids, _ = aruco.detectMarkers(self.gray, self.aruco_dict, parameters = self.parameters)
        self.output = aruco.drawDetectedMarkers(img, self.corners, self.ids)  # detect the sruco markers and display its aruco id.
        return self.output, self.ids

    def pose_estimation(self, img):
        camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
        distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 1000 pixels
        self.frame = self.cv_image
        self.frame = imutils.resize(self.frame, width=1000)
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.frame, self.aruco_dict, parameters=self.parameters)
        self.output = aruco.drawDetectedMarkers(img, self.corners, self.ids)  # detect the sruco markers and display its aruco id.
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            #estimando posição e desenhando os eixos
            ret = aruco.estimatePoseSingleMarkers(corners, 5, camera_matrix, distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            aruco.drawAxis(frame, camera_matrix, distortion, rvec, tvec, 5)


            font =cv2.FONT_HERSHEY_COMPLEX
            str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0,100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            if (tvec[0] > 5):
                print("foi")

        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


   # Define as coordenadas e as mensagens para cada destino do drone
    def trajeto(self, x, y, z, erro, msg_inicial = None, msg_final = None):
        if (msg_inicial is not None):
            rospy.loginfo(msg_inicial)
        while True:
            self.setpoint(x, y, z)
            if self.chegou(erro) == True:
                if (msg_final is not None):
                    rospy.loginfo(msg_final)
                    break
   
   # Missao a ser realizada
    def run(self):
        for i in range (150):
            self.setpoint(0, 0, 0)
        self.armar(True)
        
        while not rospy.is_shutdown():

            erro_TakeOff_Land = 0.2
            if self.modoDeVoo.mode != "OFFBOARD":
                self.setModoDeVoo("OFFBOARD")
            self.trajeto(0, 0, 2, erro_TakeOff_Land, "Decolando...")

            if ()



if __name__ == "__main__":
   try:
      rospy.init_node('mission', anonymous=True)
      Interprise = Drone()
      Interprise.run()
   except rospy.ROSInterruptException:
      pass

