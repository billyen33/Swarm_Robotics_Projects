def get_vector_a(pose, target):
	import math
	import numpy as np
	distance_a = math.sqrt((pose[0]-target[0])**2+(pose[1]-target[1])**2)
	return np.array([(target[0]-pose[0])/distance_a, (target[1]-pose[1])/distance_a]), distance_a

def get_vector_r(pose, source):
	import math
	import numpy as np
	distance_r = math.sqrt((pose[0]-source[0])**2+(pose[1]-source[1])**2)
	return np.array([-(source[0]-pose[0])/distance_r, -(source[1]-pose[1])/distance_r]), distance_r

def get_desired_angle(vector): #get desired robot orientation in degrees
	import math
	#set desired angle
	if vector[0] != 0:
		desired_angle = math.degrees(math.atan2(vector[1], vector[0])) #get desired robot orientation in degrees (0-360)
	else: #if we need to go straight down or up
		if vector[1] > 0: #going up
			desired_angle = 270
		else: #going down
			desired_angle = 90
		
	if desired_angle < 0: #make all angles positive for convenience
		desired_angle = desired_angle + 360
	if desired_angle > 360: #make all angles positive for convenience
		desired_angle = desired_angle - 360
	#handling some edge cases for desired angle
	if desired_angle == 0:
		if vector[0] < 0 or vector[1] > 0: #robot lies on x axis on the right of y axis
			desired_angle = desired_angle + 180

	return desired_angle

def get_current_angle(pose):
	import math
	current_angle = math.degrees(pose[2])
	if current_angle < 0: #make all angles positive for convenience
		current_angle = current_angle + 360
	if current_angle > 360: #make all angles positive for convenience
		current_angle = current_angle - 360

	return current_angle

def get_e_ang(current_angle, desired_angle): #error in robot orientation
	e_ang = current_angle - desired_angle
	return abs((e_ang + 180) % 360 - 180)

def usr(robot):
	import struct
	import numpy as np
	import random
	import time
	import math
	import timeit

	vector_r, distance_r = np.array([0,0]), 10000 #initialize repulsion vector
	light_source = [0, 0, 0]
	tol_ang = 1
	tol_d = 0.01
	R1 = 0.15
	R2 = 0.25
	R3 = 0.5

	if robot.assigned_id==0:
		robot.set_led(0,0,100)
		R = R1
	if robot.assigned_id==1:
		robot.set_led(0,100,0)
		R = R2
	if robot.assigned_id==2:
		robot.set_led(100,0,0)
		R = R3
	while True:
		current_pose=robot.get_pose()# gets pose The syscall that returns the robot's global pose (x, t, theta).
		#Note that the onboard sensor has a limited sampling rate of 30HZ. If there is no data from received the sensor since the last time the function get_pose() is called, the syscall will return None. If there is new data received from the sensor, the syscall will return a 3-tuple (x, y, theta), which are robot's x position, y, position, and orientation, respectively.

		#if there is a new postion sensor update, print out and transmit the info
		if current_pose: #check pose is valid before using
			pose=current_pose
			print('The x,y, theta postion of robot ',robot.id,' is ', pose[0],pose[1],pose[2])
			robot.send_msg(struct.pack('ffi', pose[0], pose[1],robot.id))# send pose x,y in message
			
			#calculate attraction vector
			vector_a, distance_a = get_vector_a(current_pose, light_source)

			#if we received a message, calc repulsion vector and print out info in message
			msgs = robot.recv_msg()
			if len(msgs) > 0:
				#create random vector
				vector_rand = get_vector_a(current_pose, [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)])[0]
				
				for i in range(len(msgs)):
					pose_rxed = struct.unpack('ffi', msgs[i][:12])
					if get_vector_r(current_pose, pose_rxed)[1] < R: #within range
						print(get_vector_r(current_pose, pose_rxed)[0])
						vector_r = vector_r + get_vector_r(current_pose, pose_rxed)[0]
						distance_r = get_vector_r(current_pose, pose_rxed)[1]
			else:
				#create random vector
				vector_rand = np.array([0,0]) #no need for randomness when the robot is alone and out of range, just have it head to center where other robots will be
				
				vector_r, distance_r = np.array([0,0]), 10000 #initialize repulsion vector
			print("distance_r: ", distance_r)
			print("distance_a: ", distance_a)
			if np.linalg.norm(vector_rand) != 0:
				vector_rand = vector_rand/np.linalg.norm(vector_rand)
			if np.linalg.norm(vector_r) != 0: #can't divide by 0
				vector_r = vector_r / np.linalg.norm(vector_r)
			vector = 6*vector_a + 10*vector_r + 2*vector_rand #scale all vectors
			vector = vector / np.linalg.norm(vector) #turn this back into a unit vector
			desired_angle = get_desired_angle(vector)
			current_angle = get_current_angle(current_pose)
			desired_angle = get_desired_angle(vector)
			e_ang = get_e_ang(current_angle, desired_angle)
			
			print("current_angle: ", current_angle)
			print("desired_angle: ", desired_angle)
			
			print("vector_rand: ", vector_rand)
			print("vector_a: ", vector_a)
			print("vector_r: ", vector_r)
			print("vector: ", vector)
			# Align robot orientation to vector
			if e_ang > tol_ang:
				#base the wheel speed on angular error
				v = 30*e_ang #proportional control, Kp of 30
				#keep wheel speed between 1 and 100
				if v > 50:
					v = 30
				if v < 1:
					v = 1
				#change wheel direction depending on angle
				if current_angle > desired_angle:
					robot.set_vel(v,-v)
				elif current_angle < desired_angle:
					robot.set_vel(-v,v)
				
			# Once angle is aligned to target, go straight
			elif distance_a > tol_d and e_ang < tol_ang:
				robot.set_vel(90,90)
				time.sleep(1)
				

			# Once close enough to target, stop
			elif distance_a < tol_d and e_ang < tol_ang:
					robot.set_vel(0,0)