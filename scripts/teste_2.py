#!/usr/bin/env python3

''' Teste de detecção de Aruco por meio da câmera do drone iris utilizado dentro do Gazebo.
	Recolhe-se os frames da stream em forma de vídeo por meio de topics específicos do ROS. '''

''' A stream é mostrada pela própria janela do OpenCV'''

''' Código não funcional devido ao tratamento do cv_image como vídeo, em que deveria ser tratado como imagens'''


from __future__ import print_function

import roslib
roslib.load_manifest('testes_cv')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/iris/cam_frontal/image_raw",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris/cam_frontal/image_raw",Image,self.callback)

  def callback(self,data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    
    
    video = cv2.VideoCapture(cv_image)
    frame_width = int(video.get(3))
    frame_height = int(video.get(4))
    frame_size = (frame_width,frame_height)
    fps = 20
    cv_image.release()



def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

  
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
