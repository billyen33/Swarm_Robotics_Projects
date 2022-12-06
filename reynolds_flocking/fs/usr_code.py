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

def get_alignment(theta_list): # input a list of velocity vectors in format [[theta1,id1], [theta2,id2], [theta3,id3] ...]
	import math
	import numpy as np
	#turn theta_list into a list of velocities
	v_list = []
	for item in theta_list:
		v_list.append([math.cos(item), math.sin(item)])
	#get the average vectors now
	v_list_l_sum = 0
	v_list_r_sum = 0
	for item in v_list:
		v_list_l_sum = v_list_l_sum + item[0]
		v_list_r_sum = v_list_r_sum + item[1]
	alignment = np.array([v_list_l_sum/len(v_list), v_list_r_sum/len(v_list)])
	return alignment / np.linalg.norm(alignment) #normalize vector

def get_cohesion(my_xy, xy_list): 
	# my_xy: 2D numpy array with the current robot's x and y position
	# xy_list: list of xy positions and id in format [[x1,y1,id1], [x2,y2,id2], ...]
	import numpy as np
	x_sum = 0
	y_sum = 0
	for item in xy_list:
		#print(item)
		x_sum = x_sum + item[0]
		y_sum = y_sum + item[1]
	#CoM is the average of all xy positions
	CoM = np.array([x_sum/len(xy_list), y_sum/len(xy_list)])
	#cohesion vector points from the robot's position to the CoM
	cohesion = CoM - np.array(my_xy)
	return cohesion / np.linalg.norm(cohesion) #normalize vector

def distance(a, b):
	#a: [x, y] of one robot
	#b: [x, y] of the other robot
	import math
	return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def get_separation(my_xy, xy_list):
	import numpy as np
	repulsion_vectors = []
	my_xy = np.array(my_xy)
	for item in xy_list:
		
		v = my_xy - item #raw vector pointing from other robot toward you
		v = v/np.linalg.norm(v) #normalize this
		if distance(my_xy, item) > 0:
			repulsion_vectors.append(v/distance(my_xy, item)) #smaller distance -> larger vector

	#now sum all the vectors
	separation = np.array([0,0])
	for item in repulsion_vectors:
		separation = separation + item
	separation = separation/np.linalg.norm(separation)
	return separation*1.2 #give separation vector more weight

def get_migration(my_xy, mig_pt):
	import numpy as np
	my_xy = np.array(my_xy)
	migration = mig_pt - my_xy
	#scale vector by factor of 1/2
	migration = migration/2
	return migration

def usr(robot):
	import struct
	import numpy as np
	import random
	import time
	import math
	import timeit
	tol_ang = 1
	tol_d = 0.01
	#initialize other inputs into vector calc functions
	R = 1000
	center = np.array([0, 0])

	#initialize all vectors that requires other robots' positions
	v_co = np.array([0, 0])
	v_sep = np.array([0, 0])
	v_ali = np.array([0, 0])

	while True:
		current_pose=robot.get_pose()# gets pose The syscall that returns the robot's global pose (x, t, theta).
		#Note that the onboard sensor has a limited sampling rate of 30HZ. If there is no data from received the sensor since the last time the function get_pose() is called, the syscall will return None. If there is new data received from the sensor, the syscall will return a 3-tuple (x, y, theta), which are robot's x position, y, position, and orientation, respectively.

		#if there is a new postion sensor update, print out and transmit the info
		if current_pose: #check pose is valid before using
			pose=current_pose
			robot.send_msg(struct.pack('ffi', pose[0],pose[1],pose[2]))# send pose x,y,theta in message
			my_loc = [pose[0], pose[1]] #xy position of this robot
			
			#get migration vector
			v_mig = get_migration(my_loc, center)
			desired_angle=0 #initialize desired_angle
			#if we received a message, calc repulsion vector and print out info in message
			msgs = robot.recv_msg()
			if len(msgs) > 0:
				#reset these lists and add our robot's coordinate/heading
				xy_list = []
				theta_list = []
				#get our xy_list and theta_list
				for i in range(len(msgs)):
					pose_rxed = struct.unpack('ffi', msgs[i][:12])
					if distance([current_pose[0], current_pose[1]], [pose_rxed[0], pose_rxed[1]]) < R: #within range
						xy_list.append([pose_rxed[0], pose_rxed[1]])
						theta_list.append(pose_rxed[2])
				v_co = get_cohesion(my_loc, xy_list)
				v_sep = get_separation(my_loc, xy_list)
				v_ali = get_alignment(theta_list)
				#get desired vector
				v_desired = v_mig + v_sep + v_co + v_ali

				#normalize desired vector
				v_desired = v_desired / np.linalg.norm(v_desired)
				#get desired heading for this robot
				desired_angle = get_desired_angle(v_desired)
			
			#get actual heading
			current_angle = get_current_angle(pose)
			e_ang = get_e_ang(current_angle, desired_angle)
			if robot.id == 2:
				# print values to debug for 1 robot
				'''print("my_loc: " + str(my_loc) + "\n")
				print("D to mig pt: " + str(distance(my_loc, center)) + "\n")
				print("v_co: " + str(v_co) + "\n")
				print("v_sep: " + str(v_sep) + "\n") 
				print("v_ali: " + str(v_ali) + "\n")
				print("v_mig: " + str(v_mig) + "\n")
				print("v_desired: " + str(v_desired) + "\n")
				print("current_angle: " + str(current_angle))
				print("desired_angle: " + str(desired_angle))
				print("e_ang: " + str(e_ang))'''

			# Align robot orientation to vector
			if e_ang > tol_ang:
				robot.set_led(100,0,0) #red LED means turning
				#base the wheel speed on angular error
				v = 30*e_ang + 25 #proportional control, Kp of 30
				#keep wheel speed between 1 and 100
				if v > 50:
					v = 50
				if v < 0:
					v = 0
				#change wheel direction depending on angle
				if current_angle > desired_angle:
					robot.set_vel(v,25)
				elif current_angle < desired_angle:
					robot.set_vel(25,v)
				
			# Once angle is aligned to target, go straight
			elif e_ang < tol_ang:
				robot.set_led(0,100,0)
				robot.set_vel(90,90)
				time.sleep(1)