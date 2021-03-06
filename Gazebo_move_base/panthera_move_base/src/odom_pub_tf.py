#!/usr/bin/env python


import rospy #https://www.youtube.com/watch?v=I_5leJK8vhQ
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

import tf
import geometry_msgs.msg

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/odom',Odometry,queue_size=1)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = 'odom'

model = GetModelStateRequest()
model.model_name = 'panthera'

r = rospy.Rate(40)

while not rospy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    broadcaster = tf.TransformBroadcaster()
        
    time = rospy.Time.now()

    header.stamp = time #rospy.Time.now()


    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z =odom.pose.pose.position.z


    wx = odom.pose.pose.orientation.x
    wy = odom.pose.pose.orientation.y
    wz = odom.pose.pose.orientation.z
    ww = odom.pose.pose.orientation.w

    broadcaster.sendTransform((x,y,z),(wx,wy,wz,ww),time,"base_footprint","odom")
    

    odom.header = header
    print(odom)
    odom_pub.publish(odom)
    r.sleep()