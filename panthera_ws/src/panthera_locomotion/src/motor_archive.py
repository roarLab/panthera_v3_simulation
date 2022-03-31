import serial
import time
import keyboard
import math
import serial.tools.list_ports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt32
import time
class RoboteqMotor():
	# 12rpm = 133 units
	def __init__(self, serialnumber, name):
		self.sn = serialnumber
		self.name = name
		self.dir = {'lb': 1, 'rb': 2, 'lf': 3, 'rf': 4}
		p = list(serial.tools.list_ports.grep(self.sn))
		self.port = serialnumber
		self.ser = serial.Serial(self.port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
		self.dt = 0.125
		self.wheel_diamter = 0.36
		self.ratio = 1000
		self.initialize()
		self.once = 0
		self.current_speed = 0
		self.ku = rospy.get_param('/{}_ku'.format(name))            #lf and rf 1 rb 1.5
		self.tu = rospy.get_param('/{}_tu'.format(name))	       #lf and rf 15
		self.kp = 0.6*self.ku  
		self.kd = 1.2*self.ku/self.tu	
		self.ki = 3*self.ku*self.tu/40	
		self.accu_error = 0
		self.prev_error = 0
		self.current_error = 0
		self.previous_read =''

	def initialize(self):
		self.ser.write("# c\r\n")                      # Clear buffer
		self.ser.write("?CB\r\n")                      # select CB for hall sensor or C for encoder
		#self.ser.write("# 10\r\n")                     # read data every 10ms
		#self.ser.write("!CB 1 0_!CB 2 0\r\n")
		#self.ser.write("^BPOL 1 3\r\n")
		self.ser.write("^VAR 1 {}\r\n".format(self.dir[self.name]))
		#self.ser.write("!EES\r\n")
		#self.ser.write("!EELD\r\n")


	def writeSpeed(self, rpm):
		# 16 units = 1 rpm

		speed = rpm # /16*1.06 
		#self.ser.write("!G {}\r\n".format(str(speed)))   #Changes direction when speed = 3277
		self.ser.write("?BS 1\r")
###################################################

		#print("This line has {} bytes".format(int(self.ser.in_waiting)))
		ln = self.ser.read(int(self.ser.in_waiting))
		ln1= self.previous_read + ln                    #Generate the combined raw text
		print("{} Serial output:".format(self.name),ln1)

		try:
			ln1 = str(ln1).split("BS=")
			if len(ln1) == 2:
				ln1 = ln1[1]
			else:
				ln12 = ln1[2]
				if "\r" not in ln12:
					ln1 = ln1[1]
				else:
					ln1 = ln1[2]
		except:
			ln1 = str(ln1).split("=")
			if len(ln1) == 2:
				ln1 = ln1[1]
			else:
				ln1 = ln1[2]
		ln1 = ln1.split("\r")[0] # Extract the actually speed value
		self.previous_read = ln

		try:			
			if (int(ln1)>= 0 and int(ln1)<=1000):
				self.current_speed=int(ln1)
			elif int(ln1)>=-1000 and int(ln1) <= 0:
				self.current_speed=int(ln1)
		except:
			pass

		self.current_error = speed - self.current_speed
		self.accu_error += self.current_error
		p = self.proportional(speed, self.current_speed)
		d = self.derivative(self.current_error, self.prev_error, self.dt)
		i = self.integral(self.accu_error, self.dt)
		speed_pid = self.kp*p + self.ki*i +  self.kd*d
		print("{} motor_speed".format(self.name),self.current_speed,"{} Speed_pid:".format(self.name),str(speed_pid))
		# print(p,i,d,speed_pid)
		speed_pid =  int(speed_pid)

		self.ser.write("!G {}\r".format( speed_pid))

		self.prev_error = self.current_error

###########################################################################

	def speed_to_rps(self, speed):
		return speed/self.wheel_diamter

	def readCurrent(self):
		self.ser.write("?AC 1\r")
		rpm = self.ser.read(1000)
		print(rpm)

	def writeAcc(self, acc):
		# increase motor spped by 0.1* rpm per s
		# acc is in motor units
		self.ser.write("!AC 1 {}\r".format(str(acc)))

	def e_stop(self):
		self.ser.write("!EX\r")

	def set_mode(self, mode):
		if mode == 0:
			print("open loop mode")
		elif mode == 1:
			print("closed-loop speed mode")
		elif mode == 5:
			print("closed-loop torque mode")
		self.ser.write("^MMOD {}\r".format(str(mode)))
		self.ser.write("?TRQ\r")
		t = str(self.ser.readline())
		for i in t:
			print(I)
		print(t)

	def set_torque(self, data):
		self.ser.write("!GIQ 1 {}\r".format(str(data*10)))

	def torque_const(self, data):
		self.ser.write("^TNM 1 1523\r") 

	def read(self):
		self.ser.read(1000)

	def custom_write(self, msg):
		self.ser.write(msg + "\r")

	#### PID controller ######
	def proportional(self, desired, actual): #error - current angle
	    prop =  self.kp * (desired - actual)
	    return prop

	def derivative(self, curr, prev, dt): #d angle-error/ self.dt
	    #self.dt = 0.05 #current_time - prev_time
	    #print("self.dt is: " + str(self.dt))
	    if self.dt == 0:
	        deriv = 0
	    else:
	        deriv = self.kd * (curr - prev) / self.dt
	    #print("Kd: "+ str(deriv))
	    return deriv

	def integral(self, accu, dt):
	    integral =  self.ki * accu * self.dt
	    #print("Ki: " +str(integral))
	    return integral
