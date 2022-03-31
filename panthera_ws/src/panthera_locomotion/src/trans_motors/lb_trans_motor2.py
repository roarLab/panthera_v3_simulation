#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial.tools.list_ports
from trans_class import TransMotor

if __name__ == "__main__":
	try:
		rospy.init_node('lb_trans_motor')
		lb_motor = TransMotor('lb', 4, 1)
		while not rospy.is_shutdown():
			if lb_motor.wheel_speed == 0: # normal steering locomotion
				lb_motor.adjust_speed(lb_motor.linear_x, lb_motor.angular_z)
			else: # reconfig mode
				lb_motor.motor.writeSpeed(lb_motor.wheel_speed)
			lb_motor.pub_wheel_vel()
	except rospy.ROSInterruptException:
		pass