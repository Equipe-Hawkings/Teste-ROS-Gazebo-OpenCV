#!/usr/bin/env python3
from queue import Empty
from std_msgs.msg import Empty, UInt8, Bool
from sensor_msgs.msg import *
import rospy
import time

def camera():
    rate = rospy.Rate(60)
    decolar = rospy.Subscriber("/tello/image_raw", Image, imagem)
    

def imagem(data):

    imagem_camera = data

if __name__ == '__main__':
    try:
        rospy.init_node('teste_inicial_tello')
        camera()
    except rospy.ROSInterruptException:
        pass