#!/usr/bin/env python3
from queue import Empty
from std_msgs.msg import Empty, UInt8, Bool
import rospy
import time

def voo():
    rate = rospy.Rate(60)
    decolar = rospy.Publisher("/tello/takeoff", Empty,queue_size=10)
    for i in range(120):
        decolar.publish()
        rate.sleep()
    pousar = rospy.Publisher("/tello/land", Empty, queue_size=10)
    for i in range(120):
        pousar.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('teste_inicial_tello')
        voo()
    except rospy.ROSInterruptException:
        pass