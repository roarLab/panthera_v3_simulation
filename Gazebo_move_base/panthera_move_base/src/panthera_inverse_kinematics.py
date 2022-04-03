#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Empty, Float64

def inverse_kinematics(data):
    #Display the received data in the terminal
    #Data is data.Received in data
    vx = data.linear.x *5# vx
    wz = data.angular.z *1.57# wz

	# calculate motor speed in m/s
    if wz == 0:
        print("vx", vx, "wz", wz)
        radius = float('inf')
        
        pub_rr_wheel.publish(vx)
        pub_rl_wheel.publish(vx)
        pub_fr_wheel.publish(vx)
        pub_fl_wheel.publish(vx)

        pub_rr_steer.publish(0.0)
        pub_rl_steer.publish(0.0)
        pub_fr_steer.publish(0.0)
        pub_fl_steer.publish(0.0)
    else:
        print("vx", vx, "wz", wz)
        radius = vx/wz
        print("vx", vx, "wz", wz,"radius",radius)
        sign = wz / abs(wz)         
        radius = vx / wz                     #radius of Instantaneous Center of Rotation
        rot_dir = vx / abs(vx)   #rotation direction

        fl_speed = rot_dir*(math.sqrt((radius + addwidth/2 - width/2)**2 + (length/2)**2 )* abs(wz)) # check + or - 
        fr_speed = rot_dir*(math.sqrt((radius + addwidth/2 - width/2)**2 + (length/2)**2 )* abs(wz)) # check + or - 
        rl_speed = rot_dir*(math.sqrt((radius + addwidth/2 - width/2)**2 + (length/2)**2 )* abs(wz)) # check + or - 
        rr_speed = rot_dir*(math.sqrt((radius + addwidth/2 - width/2)**2 + (length/2)**2 )* abs(wz)) # check + or - 
        # if abs(radius) < width/2: 
            # speed  = -sign*speed*rot_dir
            # print(speed)
        # else:
            # pass
        # return speed # check motor direction 


        left = radius - width/2
        right = radius + width/2
        fl = round(math.degrees(math.atan((length*0.5) / left)), 2)/180*math.pi                     #Publish in Radians
        fr = round(math.degrees(math.atan((length*0.5) / right)), 2)/180*math.pi                ##Publish in Radians
        rl = -fl
        rr = -fr
        print(rl,fl)
        pub_rr_wheel.publish(rr_speed)
        pub_rl_wheel.publish(rl_speed)
        pub_fr_wheel.publish(fr_speed)
        pub_fl_wheel.publish(fl_speed)

        pub_rr_steer.publish(rr)
        pub_rl_steer.publish(rl)
        pub_fr_steer.publish(fr)
        pub_fl_steer.publish(fl)


def controller():
    #Declare node name
    rospy.init_node('kinematics_node', anonymous=True)

    sub = rospy.Subscriber('cmd_vel', Twist, inverse_kinematics)




    # rate = rospy.Rate(10)     #Loop period.
    rospy.spin()
    while not rospy.is_shutdown():
        #Fill in the data to publish
        hello_str = "hello world %s" % rospy.get_time()
        #Display the data to publish in the terminal
        rospy.loginfo(hello_str)
        #Publish data

        rate.sleep()



if __name__ == '__main__':
    pub_rr_wheel = rospy.Publisher('/panthera_robot/rr_wheel_link_joint_controller/command', Float64, queue_size=1)
    pub_rl_wheel = rospy.Publisher('/panthera_robot/rl_wheel_link_joint_controller/command', Float64, queue_size=1)
    pub_fr_wheel = rospy.Publisher('/panthera_robot/fr_wheel_link_joint_controller/command', Float64, queue_size=1)
    pub_fl_wheel = rospy.Publisher('/panthera_robot/fl_wheel_link_joint_controller/command', Float64, queue_size=1)
    
    pub_rr_steer = rospy.Publisher('/panthera_robot/rr_steering_link_joint_controller/command', Float64, queue_size=1)
    pub_rl_steer = rospy.Publisher('/panthera_robot/rl_steering_link_joint_controller/command', Float64, queue_size=1)
    pub_fr_steer = rospy.Publisher('/panthera_robot/fr_steering_link_joint_controller/command', Float64, queue_size=1)
    pub_fl_steer = rospy.Publisher('/panthera_robot/fl_steering_link_joint_controller/command', Float64, queue_size=1)
    try:
        width = 0.5 #1.034   0.5 based on Gazebo scaled down model
        length = 1  #0.665    1 based on Gazebo scaled down model
        addwidth = width/2
        controller()
    except rospy.ROSInitException:
        pass
