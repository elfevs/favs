#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations


class Control():

    def __init__(self):
        
        

        self.pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        self.sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, self.callback_position)
        self.twist= Twist()
        
        self.twist.linear.x = 0.0
        print ("NAMOOO")
            
    def callback_position(self,msg):
        self.odomdata_x= msg.pose.pose.position.x
        self.odomdata_y= msg.pose.pose.position.y
        print("x=",self.odomdata_x,"y=",self.odomdata_y )
        
    # def run(self):
        
    
        if self.odomdata_y < 1.5:
            self.twist.linear.x = 0.5
            rospy.loginfo("ME MOVO")
        else:
            self.twist.linear.x = 0.0
            print("STO BONO E TRANQUILLO")
                
                
        self.pub_mir2.publish(self.twist)
            
# if __name__=="__main__":
#     rospy.init_node ("class_method") 
#     rate=rospy.Rate(100)
#     while not rospy.is_shutdown():
#         Control()
#         rate.sleep()
#         # It doesn't work, the distance increase but with a weird measure unit, 1.5 m it is really far away
            
if __name__=="__main__":
    rospy.init_node ("class_method") 
    
    Control()

    rospy.spin()
            


       