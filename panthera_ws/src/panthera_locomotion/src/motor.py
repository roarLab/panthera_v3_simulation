from sqlite3 import Time
from threading import current_thread
import serial
import time
import keyboard
import math
import serial.tools.list_ports
import rospy
from panthera_locomotion.msg import Custom_msg, Motor_health
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt32
class RoboteqMotor():
	# 12rpm = 133 units
	def __init__(self, serialnumber, name, sign,rotate_sign,turning_sign,addwidth):
		rospy.Subscriber('/panthera_cmd', Twist, self.callback) # subscriber for desired angles and cmd_vel
		rospy.Subscriber('/reconfig', Twist, self.reconfig) # individual wheel speeds
		rospy.Subscriber('/can_encoder', Twist, self.encoder_position) # subscriber for current wheel angles
		#self.current_pub = rospy.Publisher('/{}_current'.format(name),Twist) #publish current motor current
		self.wheel_vel_pub = rospy.Publisher('/{}_wheel_vel'.format(name), Custom_msg) # publish current wheel velocity
		self.custom_pub = rospy.Publisher('/{}_readings'.format(name),Custom_msg)
		self.motor_health = rospy.Publisher('/{}_motor_health'.format(name),Motor_health)
		self.sign = sign # specific to each motor, to standardize direction motor turns
		self.turning_sign = turning_sign
		self.addwidth = addwidth
		#self.address = address # address of motor
		self.rotate_sign=rotate_sign
		self.sn = serialnumber
		self.name = name
		self.dir = {'lb': 1, 'rb': 2, 'lf': 3, 'rf': 4}
		p = list(serial.tools.list_ports.grep(rospy.get_param('/{}_serial_number'.format(name))))
		self.port = '/dev/' + p[0].name
		self.ser = serial.Serial(self.port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
		self.dt = 0.125
		self.wheel_diamter = 0.36
		self.ratio = 1000
		self.initialize()
		self.once = 0
		self.current_speed = 0
		self.ku = rospy.get_param('/{}_ku'.format(name))            #lf and rf 1 rb 1.5
		self.tu = rospy.get_param('/{}_tu'.format(name))	       #lf and rf 15
		# self.kp = rospy.get_param("/{}_kp".format(name))
		# self.kd = rospy.get_param("/{}_kd".format(name))
		# self.ki = rospy.get_param("/{}_ki".format(name))

		self.kp = 0.6*self.ku  
		self.kd = 1.2*self.ku/self.tu	
		self.ki = 3*self.ku*self.tu/40	
		self.accu_error = 0
		self.prev_error = 0
		self.current_error = 0
		self.motor_current = 0
		# robot parameters
		self.width = 0.665
		self.length = 1.034
		self.wheel_radius = 0.18
		self.gear_ratio = 1
		self.msg = Twist()
		self.custom = Custom_msg()
		self.fuse_condition = Motor_health()
		self.why = 0 
		self.linear_x = 0
		self.angular_z = 0
		self.wheel_speed = 0
		self.reconfig_speed = 0

		self.wheel_velocity = 0
		self.motor_rpm = 0
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

		speed = -rpm # /16*1.06 
		#self.ser.write("!G {}\r\n".format(str(speed)))   #Changes direction when speed = 3277
		self.ser.write("?BS 1\r")           #Get speed in RPM
		self.ser.write("?A 1\r")           #Get motor current
		#self.ser.write("?BA 1\r")           #Get Battery current

###################################################
		try:
			
			#print("This line has {} bytes".format(int(self.ser.in_waiting)))
			ln = self.ser.read(int(self.ser.in_waiting))
			ln1= self.previous_read + ln                    #Generate the combined raw text
			#print("{} Serial output:".format(self.name),ln1)

			ln2 = str(ln1).split("A=")
			self.why = float(ln2[2].split("\r")[0])


			
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
			try:
				speed_pid = record
			except:
				speed_pid = speed
			command_speed = int(speed*0.22)
			if command_speed >= 0:
				if self.current_speed >= int(0.8*command_speed) and self.current_speed <= int(1.2*command_speed):
				
					speed_pid = int(speed_pid)
				elif self.current_speed >= int(1.2*command_speed) and self.current_speed <= int(1.4*command_speed):
					speed_pid = int(speed_pid*0.7)
				elif self.current_speed >= int(1.4*command_speed):
					speed_pid = int(speed_pid*0.5)
				elif self.current_speed <= int(0.8*command_speed):
					speed_pid = int(speed_pid*1.2)
			if command_speed <= 0:
				if self.current_speed >= int(1.2*command_speed) and self.current_speed <= int(0.8*command_speed):
					speed_pid = int(speed_pid)
				elif self.current_speed >= int(0.8*command_speed):
					speed_pid = int(speed*1.2)
				elif self.current_speed <= int(1.2*command_speed):
					speed_pid = int(speed*0.5)

			record = speed_pid
			#speed_pid = self.kp*p + self.ki*i +  self.kd*d
			print("{} Motor current:".format(self.name),self.why)
			print("{} motor_speed".format(self.name),self.current_speed,"{} Speed_pid:".format(self.name),str(speed_pid))
			# print(p,i,d,speed_pid)
			# speed_pid =  int(speed) #int(speed_pid)
			
			self.msg.linear.x = self.current_speed
			self.msg.linear.y = self.why
			self.msg.linear.z = 1.0
			self.msg.angular.x = 1.0
			self.msg.angular.y = 1.0
			self.msg.angular.z = 1.0
			self.custom.current = self.why #rpm
			self.custom.target_speed = rpm
			self.custom.pid_speed = speed_pid
			self.custom.speed = float(self.current_speed)
			self.fuse_condition = 1

			self.motor_health.publish(self.fuse_condition)
			self.ser.write("!G {}\r".format( speed_pid))
			self.prev_error = self.current_error
			self.wheel_vel_pub.publish(self.custom)
			#self.custom_pub.publish(self.custom)
			counter = time.time()
			while time.time()-counter <= 0.05:
				pass
			#time.sleep(0.05)
		except:
			self.fuse_condition = 0 
			self.motor_health.publish(self.fuse_condition)


############################################################################
	def encoder_position(self, data):
		# positions of wheel and its complement
		if self.name == 'lb':
			self.position = data.linear.x
			self.complement = data.linear.y

		elif self.name == 'rb':
			self.position = data.linear.y
			self.complement = data.linear.x

		elif self.name == 'lf':
			self.position = data.linear.z
			self.complement = data.angular.x

		elif self.name == 'rf':
			self.position = data.angular.x
			self.complement = data.linear.z

		self.width = data.angular.z # 1 wire encoder
		#self.width = (data.angular.z + data.angular.y)/2 # 2 wire encoders

	def reconfig(self, data):
		# individual speed of motor in m/s
		if self.name == 'lb':
			self.reconfig_speed = data.linear.x
		elif self.name == 'rb':
			self.reconfig_speed = data.linear.y
		elif self.name == 'lf':
			self.reconfig_speed = data.linear.z
		elif self.name == 'rf':
			self.reconfig_speed = data.angular.x
		self.wheel_speed = self.rads_to_rpm(self.reconfig_speed / self.wheel_radius)

	def callback(self, data):
		# cmd_vel for robot 
		self.linear_x = data.angular.y # vx
		self.angular_z = data.angular.z # wz

	def motor_lin_vel(self, vx, wz):  #linear velocity
		# calculate motor speed in m/s
		sign = wz / abs(wz)
		r = vx / wz
		rot_dir = vx / abs(vx)
		speed = rot_dir*(math.sqrt((r + self.addwidth/2 - self.sign*self.width/2)**2 + (self.length/2)**2 )* abs(wz)) # check + or - 
		if abs(r) < self.width/2:
			speed  = -sign*self.sign*speed*rot_dir
			print(speed)
		else:
			pass
		return speed # check motor direction 

	def rads_to_rpm(self, x):
		# convert rads-1 to rpm
		rpm = (x / (2*math.pi)) * 60 * self.gear_ratio
		return int(-rpm)

	def rpm_to_rads(self, x):
		# convert rpm to rads-1
		rads = (x/60) * 2 * math.pi / self.gear_ratio
		return (-rads)

	def adjust_speed(self, vx, wz):
		# control speed of wheel
		speed = 0
		#if run_mode == True:
		if vx == 0: # if cmd is to stop robot
			if wz == 0: # if no rotation cmd
				rpm = 0
				speed = 0
				self.motor_rpm = rpm
				self.writeSpeed(rpm)
				#print(" rpm: 0")

			else:
				# static rotation of robot
				speed = self.rotate_sign*self.sign*wz * math.sqrt((self.length/2)**2 + (self.width/2)**2) / self.wheel_radius # speed of the wheel
				rpm = self.rads_to_rpm(speed) # convert to rpm
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm
				self.writeSpeed(self.motor_rpm)

		else:
			if wz == 0: # moving straight
				speed = vx / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm*self.sign
				self.writeSpeed(self.motor_rpm)


			else:
				# robot making a turn
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm*self.sign
				self.writeSpeed(self.motor_rpm)

	
	def pub_wheel_vel(self):
		# publish wheel velocity
		self.wheel_velocity = 10  #self.sign*self.rpm_to_rads(self.readSpeed()) * self.wheel_radius
		self.wheel_vel_pub.publish(self.wheel_velocity)
		#print(self.name, self.wheel_velocity, self.motor_rpm)
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
	    prop =  self.kp * (desired - actual) + desired
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
