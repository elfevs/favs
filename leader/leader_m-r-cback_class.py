#!/usr/bin/env python3



import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations



class Move_rotate_cback():
    def __init__(self) :

        
        
        self.sub = rospy.Subscriber("/mir1/ground_truth", Odometry, self.move_rot_callback)
        self.pub = rospy.Publisher("/mir1/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        self.command = Twist()
        self.odomdata = Odometry()



    def  move_rot_callback(self,msg): 
        
        self.odomdata_x = msg.pose.pose.position.x
        self.odomdata_y = msg.pose.pose.position.y

       
        self.odomdata_orientation = transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.odomdata_orientation_z= self.odomdata_orientation[2]
        
        print("x=",self.odomdata_x,"y=",self.odomdata_y,"angle=", math.degrees(self.odomdata_orientation_z))

    def go_straight(self, target):
        
       
        distance= (target - self.odomdata_y)
        
        if abs(distance) > 0.1:
            self.command.linear.x = 0.3 * distance
        else:
            self.command.linear.x = 0.0
            
        self.pub.publish(self.command)
        
        rospy.loginfo("distance_x", distance)
            
            
    
    # def rotate(self,target_angle_degrees):
    #     angular_deviation= (target_angle_degrees-(math.degrees(self.odomdata_orientation_z)))
        
    #     if angular_deviation > 1:
    #         self.command.angular.z= angular_deviation * 0.5
            
    #     else:
    #         self.command.angular.z= 0.0
            
    #     self.pub.publish(self.command)

    
    
    
    # def movimentation(self):
    #     for i in range(5):
    #         self.go_straight(1.0)
    #         self.rotate(180)
    #         self.go_straight(0.0)
    #     pass
        
if __name__ == "__main__" :
    
    rospy.init_node ("leader_move")     
    move=Move_rotate_cback()
    move.go_straight(target=1.0)
    rospy.spin()
            
            
            
      
            
            

