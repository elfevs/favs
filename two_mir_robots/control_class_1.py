#!/usr/bin/env python3



import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations



class Control():

    def __init__(self):


        self.sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, self.callback_posion_mir1,"mir1")
        self.sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, self.callback_posion_mir2,"mir2")
        self.sub_mir1_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, self.velocity_callback)
        # self.sub_mir1_vel = rospy.Subscriber("/mir1/ground_truth", Odometry, self.velocity_callback)

        # the velocity is in the global frame, I need it in the local one


        self.pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)


        self.odomdata = Odometry()



        self.command_velocity =Twist()




    def callback_posion_mir1(self, msg, robot_name):
        if robot_name == "mir1":
            self.odomdata_x1 = msg.pose.pose.position.x
            self.odomdata_y1 = msg.pose.pose.position.y
            self.odomdata_z1 = msg.pose.pose.position.z



            self.odomdata_orientation1 = transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            self.odomdata_orientation_z1= self.odomdata_orientation1[2]

         # # TO KNOW THE POSITION OF THE ROBOTS mir1


            # print ("x1:",self.odomdata_x1, "[m]")
            # print ("y1:",self.odomdata_y1, "[m]")
            # print ("orientation1:",math.degrees(self.odomdata_orientation_z1), "°")


    def callback_posion_mir2(self, msg, robot_name):
        if robot_name == "mir2":
            self.odomdata_x2 = msg.pose.pose.position.x
            self.odomdata_y2 = msg.pose.pose.position.y
            self.odomdata_z2 = msg.pose.pose.position.z



            self.odomdata_orientation2 = transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            self.odomdata_orientation_z2= self.odomdata_orientation2[2]

         # # TO KNOW THE POSITION OF THE ROBOTS mir2


            # print ("x2:",self.odomdata_x2, "[m]")
            # print ("y2:",self.odomdata_y2, "[m]")
            # print ("orientation2:",math.degrees(self.odomdata_orientation_z2), "°")




    def velocity_callback(self,msg):


        self.velocity_x = msg.linear.x
        self.velocity_y = msg.linear.y
        self.velocity_angular_z = msg.angular.z



        print("vel_x:",self.velocity_x,  "[m/s]")
        print("vel_y:",self.velocity_y, "[m/s]")
        print("vel_angular:",self.velocity_angular_z, "[rad/s]")

        print ("-----------------------------------------------------------")
        print ("-----------------------------------------------------------")




        if abs(self.velocity_x)> 0.05 or abs(self.velocity_y) > 0.05 or abs(self.velocity_angular_z) > 0.05:
            distance = 1
            absolute_vel= self.velocity_x + self.velocity_angular_z * distance * (1 - 2* (math.cos(self.odomdata_orientation_z1))**2 )

            self.command_velocity.linear.x = absolute_vel

            self.command_velocity.linear.y = 0
            self.command_velocity.angular.z = self.velocity_angular_z

            self.pub_mir2.publish(self.command_velocity)


            print ("*****************I GO******************", absolute_vel)

            print ("angolo", 2* (math.cos(self.odomdata_orientation_z1))**2)
            print ("increase of speed in curves", absolute_vel - self.velocity_x)
          




if (__name__ == "__main__") :

    rospy.init_node ("class_method")
    Control()
    rospy.spin()
















