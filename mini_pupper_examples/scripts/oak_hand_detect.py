#!/usr/bin/env python3
import rospy
#from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection2DArray
from depthai_ros_msgs.msg import HandLandmarkArray, HandLandmark
import math
import numpy as np

height = 300
width = 300
roll=0
pitch=0
yaw = 0
yaw_increment=0
pitch_increment=0
pose = Pose()

#mobilenet object list
#0: background
#1: aeroplane
#2: bicycle
#3: bird
#4: boat
#5: bottle
#6: bus
#7: car
#8: cat
#9: chair
#10: cow
#11: diningtable
#12: dog
#13: horse
#14: motorbike
#15: person
#16: pottedplant
#17: sheep
#18: sofa
#19: train
#20: tvmonitor

def toward_obj(hand):
    global pose,roll,pitch,yaw,yaw_increment,pitch_increment
    rate = rospy.Rate(200) # 200hz
    yaw_increment = 0
    pitch_increment = 0
    
    middle_finger_mcp = hand.landmark[9]
    position = hand.position
    # for i in hand.landmark:
    #     bb = i.bbox #BoundingBoxes
    #     si = i.source_img #SourceImage
    #     r = i.results #Results
    #     rr = r[0] #RealResults
    #     #print(rr.id)
    #     if(rr.id == obj_class):
    #         print(11111)

    yaw_increment=(width/2 - middle_finger_mcp.x)*0.0002
    pitch_increment=-(height/2 - middle_finger_mcp.y)*0.0002

    # Forward follow routine
    if 0:
        if position.x > 1.0 and position.x < 3:
            pose.position.x = position.x - 1.0 
        if 0.4 < position.x and position.x < 1.0:
            pose.position.x = position.x - 1.0 

    #TODO: Add roll to rotate left and right based on 2 points of the hand. 
    if 1:
        if hand.gesture == "FIVE":
            wrist = hand.landmark[0]
            middle_finger_tip = hand.landmark[12]
            hand_vector = [middle_finger_tip.x - wrist.x, middle_finger_tip.y - wrist.y]
            hand_vector = hand_vector / np.linalg.norm(hand_vector)
            print(f'Hand vector is {hand_vector}')
            vertical_vector = [0, -1] 
            vertical_vector = vertical_vector / np.linalg.norm(vertical_vector)
            dot_product = np.dot(vertical_vector, hand_vector)
            det = np.linalg.det([vertical_vector, hand_vector])
            # angle = np.arccos(dot_product)
            angle = np.arctan2(dot_product, det)
            # roll = np.arccos(dot_product)
        
            print(f"Printing roll of the vectors = {math.degrees(angle)}")

    yaw = yaw + yaw_increment
    #print(yaw_increment)
    pitch = pitch + pitch_increment
    cy =math.cos(yaw*0.5)
    sy =math.sin(yaw*0.5)
    cp =math.cos(pitch*0.5)
    sp =math.sin(pitch*0.5)
    cr =math.cos(roll * 0.5)
    sr =math.sin(roll * 0.5)

    pose.orientation.w = cy * cp * cr + sy * sp * sr
    pose.orientation.x = cy * cp * sr - sy * sp * cr
    pose.orientation.y = sy * cp * sr + cy * sp * cr
    pose.orientation.z = sy * cp * cr - cy * sp * sr

    pub_pose.publish(pose)
    print('Sendng pOse...')
    rate.sleep()

def callback(data):
    handArray = data
    hands = handArray.landmarks
    for hand in hands:
        if hand.label == 'right':
            toward_obj(hand)
    #toward_obj('cup',a)
    #print(a[0])
    
def listener():
    rospy.init_node('hand_detect', anonymous=True)
    rospy.Subscriber("depthai/hand_tracklets", HandLandmarkArray, callback)
    rospy.spin()
 
if __name__ == '__main__':
    pub_pose = rospy.Publisher('/body_pose', Pose, queue_size=10)
    listener()
