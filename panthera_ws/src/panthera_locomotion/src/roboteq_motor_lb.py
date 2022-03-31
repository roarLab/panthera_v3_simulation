#!/usr/bin/env python
import rospy
from motor import RoboteqMotor as rm
import serial.tools.list_ports
from std_msgs.msg import Float64, Int64

def run_cmd(msg):
	if mode == 0:
	
		motor.writeSpeed(-20)    #msg.data)    #lf's direction is opposite to rf
		#print("here",msg.data)

			
	elif mode == 1:
		motor.writeTorque(msg.data)
	#motor.read_speed()


def queries(msg):
	if msg.data == 0:
		mode = 0
		
		motor.set_mode(msg.data)
	else:
		mode = 1
		motor.set_mode(msg.data)
		#motor.writeTorque(1000)
	print("mode: {}".format(str(mode)))
	


if __name__=="__main__":

	rospy.init_node("roboteq_motor_lb")
	
	p = list(serial.tools.list_ports.grep("FT4ZF49U")) #
	port = '/dev/' + p[0].name
	lb_motor = rm(port, 'lb',1,-1,1,-0.665) 
	#rospy.Subscriber("/run_motor", Float64, run_cmd)
	
	mode = 0
	try:
		while not rospy.is_shutdown():
			if lb_motor.wheel_speed == 0: # normal steering locomotion
				lb_motor.adjust_speed(lb_motor.linear_x, lb_motor.angular_z)
			else: # reconfig mode
				lb_motor.motor.writeSpeed(lb_motor.wheel_speed)
			
	except rospy.ROSInterruptException:
		pass

