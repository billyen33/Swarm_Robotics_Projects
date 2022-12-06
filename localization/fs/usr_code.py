def usr(robot):
	import struct
	import math
	import timeit
	hop1 = float('inf') #init as a big number
	hop2 = float('inf')

	while True:
		## GENERATE HOP COUNTS
		#put 1 and 2 as seed robot
		if robot.assigned_id == 1 or robot.assigned_id == 2:
			#set color of seed robots to white since they are in corners
			robot.set_led(255,255,255)

			#decide message based on which seed it is
			if robot.assigned_id == 1:
				first = 0
				second = hop2
			else: #id is 2
				first = hop1
				second = 0
			
			robot.send_msg(struct.pack('ffi', first, second, 0)) #send init hop counts in message
		
		#not seed robot
		else:
			msgs = robot.recv_msg()
			if len(msgs) > 0: #new message
				hop_rxed = struct.unpack('ffi', msgs[0][:12]) #see what message says

				#updating hop1
				if hop_rxed[0] < hop1: #ignore messages from larger hopcounts
					#increment hop count and send message
					hop1 = hop_rxed[0] + 1
					#print("hop1 changed: " + str(hop1))
					robot.send_msg(struct.pack('ffi', hop1, hop2, 0))
				
				#updating hop2
				if hop_rxed[1] < hop2: #ignore messages from larger hopcounts
					#increment hop count and send message
					hop2 = hop_rxed[1] + 1
					#print("hop2 changed: " + str(hop2))
					robot.send_msg(struct.pack('ffi', hop1, hop2, 0))
				
				## METHOD TO MAKE N FOR REGULAR COORDINATE SYSTEM
				e = float('inf')
				s1_coord = (0, 0)
				s2_coord = (20, 0)
				coord = (0, 0)
				for x in range(20):
					for y in range(40):
						d1 = math.sqrt((s1_coord[0]-x)**2+(s1_coord[1]-y)**2)
						d2 = math.sqrt((s2_coord[0]-x)**2+(s2_coord[1]-y)**2)
						e_temp = (d1-hop1)**2 + (d2-hop2)**2
						if e_temp < e:
							e = e_temp
							coord = (x, y)
				
				## METHOD TO MAKE N FOR SMOOTHED COORDINATE SYSTEM
				""" e = float('inf')
				s1_coord = (0, 0)
				s2_coord = (20, 0)
				coord = (0, 0)
				for x in range(20):
					for y in range(40):
						d1 = math.sqrt((s1_coord[0]-x)**2+(s1_coord[1]-y)**2)
						d2 = math.sqrt((s2_coord[0]-x)**2+(s2_coord[1]-y)**2)
						total_h1 = []
						total_h2 = []
						msgs = robot.recv_msg()
						if len(msgs) > 0: #new message
							hop_rxed = struct.unpack('ffi', msgs[0][:12]) #see what message says
							if hop_rxed[0] < 1000000:
								total_h1.append(hop_rxed[0])
								if len(total_h1) > 200:
									total_h1.pop(0)
							if hop_rxed[1] < 1000000:
								total_h2.append(hop_rxed[1])
								if len(total_h2) > 200:
									total_h2.pop(0)
						s1 = (sum(total_h1)+hop1)/(len(total_h1)+1) - 0.5
						s2 = (sum(total_h2)+hop2)/(len(total_h2)+1) - 0.5
						e_temp = (d1-s1)**2 + (d2-s2)**2
						if e_temp < e:
							e = e_temp
							coord = (x, y) """
				
				## MAKING THE N
				#section 1 (bottom left rectangle)
				if coord[0] < 8 and coord[1] < 10:
					robot.set_led(255, 255, 255)
				#section 2 (bottom left triangle)
				elif coord[0] < 8 and coord[1] < -2*coord[0]+7:
					robot.set_led(255, 255, 255)
				#section 3 (left vertical rectangle)
				elif coord[0] < 7:
					robot.set_led(255, 255, 255)
				#section 4 (diagonal line)
				elif coord[1] < -2.1*coord[0]+32 and coord[1] > -2.1*coord[0]+25:
					robot.set_led(255, 255, 255)
				#section 5 (right vertical rectangle)
				elif coord[0] > 13:
					robot.set_led(255, 255, 255)
				#set all other robots to no light
				else:
					robot.set_led(0,0,0)