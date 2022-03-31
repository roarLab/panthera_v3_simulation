#!/usr/bin/env python
import rospy
from motor import RoboteqMotor as rm
import serial.tools.list_ports
from std_msgs.msg import Float64, Int64

def run_cmd(msg):
	if mode == 0:
	
		motor.writeSpeed(20)    #msg.data)
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
	
	rospy.init_node("roboteq_motor_rf")

	#initiate
	p = list(serial.tools.list_ports.grep("FT4WIY51")) #
	port = '/dev/' + p[0].name
	rf_motor = rm(port, 'rf',-1,1,-1,0.665)  #20100201396000000       SBL1XXX 
	#rospy.Subscriber("/run_motor", Float64, run_cmd)

	mode = 0
	try:
		while not rospy.is_shutdown():
			if rf_motor.wheel_speed == 0: # normal steering locomotion
				rf_motor.adjust_speed(rf_motor.linear_x, rf_motor.angular_z)
			else: # reconfig mode
				rf_motor.motor.writeSpeed(rf_motor.wheel_speed)
			
	except rospy.ROSInterruptException:
		pass

