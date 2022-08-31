#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations

# mir 1 LEADER
# mir 2 FOLLOWER

rospy.init_node ("follower_centralize_formation_control")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    global roll_mir1, pitch_mir1, yaw_mir1, pos_mir1_x, pos_mir1_y
    orientation_q = msg.pose.pose.orientation
    pos_mir1_x = msg.pose.pose.position.x
    pos_mir1_y= msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir1, pitch_mir1, yaw_mir1) = euler_from_quaternion(orientation_list)


def  mir2_pos_callback(msg): 
    global roll_mir2, pitch_mir2, yaw_mir2, pos_mir2_x, pos_mir2_y
    orientation_q = msg.pose.pose.orientation
    pos_mir2_x = msg.pose.pose.position.x
    pos_mir2_y = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir2, pitch_mir2, yaw_mir2) = euler_from_quaternion(orientation_list)

#------------------------------------------------------------------------------------------------------
def  mir1_callback(msg): 
    global lin_vel_mir1_x , lin_vel_mir1_y, ang_vel_mir1_z
    lin_vel_mir1_x= msg.linear.x
    lin_vel_mir1_y = msg.linear.y
    ang_vel_mir1_z = msg.angular.z


pos_mir1_x = 0
pos_mir1_y = 0
roll_mir1 = pitch_mir1 = yaw_mir1 = 0.0

pos_mir2_x = 0
pos_mir2_y = 0
roll_mir2 = pitch_mir2 = yaw_mir2 = 0.0

lin_vel_mir1_x= 0.0
lin_vel_mir1_y = 0.0
ang_vel_mir1_z = 0.0

t0 =0
t1 =0
desired_distance = 1.0 



sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, mir2_pos_callback)
sub_mir2_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, mir1_callback)


pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command_2 = Twist()


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    
    r_1= (pos_mir1_x**2+pos_mir1_y**2)**0.5
    r_2= (pos_mir2_x**2+pos_mir2_y**2)**0.5

    D_x = (pos_mir2_x-pos_mir1_x)  
    D_y = (pos_mir2_y-pos_mir1_y)  
    distance = (D_x**2+ D_y**2)**0.5 
    
    
    if pos_mir1_y!=0 and pos_mir2_y != 0:
        theta = math.atan((pos_mir2_x-pos_mir1_x)/(pos_mir2_y-pos_mir1_y))
        Vel_mir2_x = lin_vel_mir1_x + omega * distance * (1 - 2* (math.cos(theta))**2 )
        Vel_mir2_y = 0
        
        # command_2.linear.x= Vel_mir2_x
        # command_2.angular.z = omega
            
        


        if command_2.angular.z != 0:
            print ("Irotate")
            if omega <= 0:
                print ("clockwise rotation")
            else:
                print ("unter-clockwise rotation")

        else:
            if (pos_mir1_x-pos_mir2_x) >= 0.001:
                print ("leader is going forward")
            else:
                print ("leader is going backward")

        print ("time:",t0,t1)
        print ("distance:", distance)
        print ("desired_distance:", desired_distance)

        print ("V_1:", Vel_mir1,"V_2:",Vel_mir2_x)
        print ("theta1", yaw_mir1,"theta2", yaw_mir2, "vel_ang:", omega)

        print("-------------------------------------------------------------------------------------------")

    
    
    # transformations.
        
    t0 = rospy.Time.now().to_sec() - 1/100  #1/100 is rospy.rate
    t1 = rospy.Time.now().to_sec()
    

    # knowing the orientation of the leader robot

    Vel_mir1_x= lin_vel_mir1_x * math.cos(yaw_mir1)
    Vel_mir1_y = lin_vel_mir1_x * math.sin(yaw_mir1)
    Vel_mir1 = (Vel_mir1_x**2 + Vel_mir1_y**2)**0.5
    
    omega = ang_vel_mir1_z 
  
    

    pub_mir2.publish(command_2)
    rate.sleep()