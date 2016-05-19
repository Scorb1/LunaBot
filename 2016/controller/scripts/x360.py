#!/usr/bin/env python
import rospy
import time

from controller.msg import controller_i2c
from sensor_msgs.msg import Joy

pub = rospy.Publisher('i2c', controller_i2c, queue_size=10)

#handle joy initializing triggers to 0 instead of 1.0 (the release value)
l_trig_ready = False
r_trig_ready = False
UL = 0x09
UR = 0x04
BL = 0x05
BR = 0x07
LB = 0x01 #TODO mapped to B button
BM = 0x02 #TODO mapped to A button
	
bin_pos = 0
ex_pos = 0
bm_pos = 0
desiredPWM_L = 0.0
desiredPWM_R = 0.0
axis_x = 0.0
axis_y = 0.0
button_a = 0
button_b = 0
button_x = 0
button_y = 0
buttons = [0] * 11
l_trigger = 0.0		
r_trigger = 0.0
maxPWMval = 255.0	#pwm range is (0 - 255) forward or reverse
PWM_dx = maxPWMval/10.0
counter = [0] * 3	#button counters for B, X, Y

def callback_joy(data):
	global axis_x
	global axis_y
	global l_trigger
	global r_trigger
	global buttons
	global l_trig_ready
	global r_trig_ready
	buttons = data.buttons
	axis_x = data.axes[0]		 	#from -1 to 1
	axis_y = data.axes[1]		 	#from -1 to 1
	l_trigger = -1.0 * (data.axes[2]/2 - 0.5)	#from 0 to 1
	r_trigger = -1.0 * (data.axes[5]/2 - 0.5)	#from 0 to 1
	#triggers will incorrectly equal 0.5 until pressed and released due to terrible joy initialization
	if l_trigger != 0.5 and l_trig_ready == False:
		l_trig_ready = True
	if r_trigger != 0.5 and r_trig_ready == False:
		r_trig_ready = True

#handles slowing down to stop
def stop():
	global desiredPWM_L
	global desiredPWM_R
	msg = controller_i2c()
	msg.direction = 127
	
	#slow down left wheels to stop (0.0 is stationary value)
	if desiredPWM_L - PWM_dx > 10:		#if robot was going forward 
		#decrease
		desiredPWM_L = desiredPWM_L - PWM_dx
		msg.direction = 102
		msg.pwn = desiredPWM_L
		msg.address = BL
		pub.publish(msg)
		msg.address = UL
		pub.publish(msg)
	elif desiredPWM_L + PWM_dx < -10:		#if robot was reversing
		#increase
		desiredPWM_L = desiredPWM_L + PWM_dx
		msg.direction = 114
		msg.pwn = abs(desiredPWM_L)
		msg.address = BL
		pub.publish(msg)
		msg.address = UL
		pub.publish(msg)
	else:
		#settle to mid
		desiredPWM_L = 0
		msg.direction = 32
		msg.pwn = abs(desiredPWM_L)
		msg.address = BL
		pub.publish(msg)
		msg.address = UL
		pub.publish(msg)
	#slow down right wheel to stop
	if desiredPWM_R - PWM_dx > 10:		#if robot was going forward 
		#decrease
		desiredPWM_R = desiredPWM_R - PWM_dx
		msg.direction = 102
		msg.pwn = desiredPWM_R
		msg.address = BR
		pub.publish(msg)
		msg.address = UR
		pub.publish(msg)
	elif desiredPWM_R + PWM_dx < -10:		#if robot was reversing
		#increase
		desiredPWM_R = desiredPWM_R + PWM_dx
		msg.direction = 114
		msg.pwn = abs(desiredPWM_R)
		msg.address = BR
		pub.publish(msg)
		msg.address = UR
		pub.publish(msg)
	else:
		#settle to mid
		msg.direction = 32
		desiredPWM_R = 0
		msg.pwn = desiredPWM_R
		msg.address = BR
		pub.publish(msg)
		msg.address = UR
		pub.publish(msg)

def move():
	global desiredPWM_L
	global desiredPWM_R
	button_a = buttons[0]
	button_b = buttons[1]
	button_x = buttons[2]
	button_y = buttons[3]
	msg = controller_i2c()

	#wheel movement
	if l_trigger > 0.05 and l_trig_ready == True:	
		#zero turn left
		print 'zero left: ',desiredPWM_L," ",desiredPWM_R," ",l_trigger
		if desiredPWM_L - PWM_dx > -maxPWMval * l_trigger:
			#decrease
			desiredPWM_L = desiredPWM_L - PWM_dx
			msg.direction = 114
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)
		else:
			#settle to min
			desiredPWM_L = -maxPWMval * l_trigger
			msg.direction = 114
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)
		if desiredPWM_R + PWM_dx < maxPWMval * l_trigger:
		#increase
			desiredPWM_R = desiredPWM_R + PWM_dx
			msg.direction = 102
			msg.pwn = desiredPWM_R
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)  
			  
		else:
			#settle to max
			desiredPWM_R = maxPWMval * l_trigger
			msg.direction = 102
			msg.pwn = desiredPWM_R
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)  
			
	elif r_trigger > 0.05 and r_trig_ready == True:	
		#zero turn right
		print 'zero right: ',desiredPWM_L," ",desiredPWM_R," ",r_trigger#
		if desiredPWM_R - PWM_dx > -maxPWMval * r_trigger:
			#decrease
			desiredPWM_R = desiredPWM_R - PWM_dx
			msg.direction = 114
			msg.pwn = desiredPWM_R
			msg.address = UR
			pub.publish(msg)
			msg.address = BR
			pub.publish(msg)  
		else:
			#settle to min
			desiredPWM_R = -maxPWMval * r_trigger
			msg.direction = 114
			msg.pwn = desiredPWM_R
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)  
		if desiredPWM_L + PWM_dx < maxPWMval * r_trigger:
			#increase
			desiredPWM_L = desiredPWM_L + PWM_dx
			msg.direction = 102
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)
		else:
			#settle to max
			desiredPWM_L = maxPWMval * r_trigger
			msg.direction = 102
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)


	elif axis_y > 0.1 :
		#if pressing forward
		print 'going forward: ',desiredPWM_L," ",desiredPWM_R," ",axis_y, " ", axis_x
		msg.direction = 102
#forward left
		#if axis_x< -0.1:		
			#if desiredPWM_L + PWM_dx < maxPWMval * axis_y:
			##increase left wheels
				#desiredPWM_L = desiredPWM_L + PWM_dx - (abs(axis_x*maxPWMval))/2
				#if desiredPWM_L <10:
					#desiredPWM_L=10
				#msg.pwn = desiredPWM_L
				#msg.address = BL
				#pub.publish(msg)
				#msg.address = UL
				#pub.publish(msg)
				
			#else:
			##settle left wheels
				#desiredPWM_L = maxPWMval * axis_y - (abs(axis_x*maxPWMval))/2
				#if desiredPWM_L <10:
					#desiredPWM_L=10
				#msg.pwn = desiredPWM_L
				#msg.address = BL
				#pub.publish(msg)
				#msg.address = UL
				#pub.publish(msg)
			#if desiredPWM_R + PWM_dx < maxPWMval * axis_y:
			##increase right wheels
				#desiredPWM_R = desiredPWM_R + PWM_dx
				#msg.pwn = desiredPWM_R
				#msg.address = BR
				#pub.publish(msg)
				#msg.address = UR
				#pub.publish(msg)
			#else:
				##settle right wheels
				#desiredPWM_R = maxPWMval * axis_y
				#msg.pwn = desiredPWM_R
				#msg.address = BR
				#pub.publish(msg)
				#msg.address = UR
				#pub.publish(msg)

##forward right
		#if axis_x > 0.1:		
			#if desiredPWM_L + PWM_dx < maxPWMval * axis_y:
			##increase left wheels
				#desiredPWM_L = desiredPWM_L + PWM_dx 
				#msg.pwn = desiredPWM_L
				#msg.address = BL
				#pub.publish(msg)
				#msg.address = UL
				#pub.publish(msg)
			#else:
			##settle left wheels
				#desiredPWM_L = maxPWMval * axis_y 
				#msg.pwn = desiredPWM_L
				#msg.address = BL
				#pub.publish(msg)
				#msg.address = UL
				#pub.publish(msg)

			#if desiredPWM_R + PWM_dx < maxPWMval * axis_y:
			##increase right wheels
				#desiredPWM_R = desiredPWM_R + PWM_dx - axis_x*maxPWMval/2
				#if desiredPWM_R <10:
					#desiredPWM_R=10
				#msg.address = BR
				#pub.publish(msg)
				#msg.address = UR
				#pub.publish(msg)
			#else:
				##settle right wheels
				#desiredPWM_R = maxPWMval * axis_y - axis_x*maxPWMval/2
				#if desiredPWM_R <10:
					#desiredPWM_R=10
				#msg.address = BR
				#pub.publish(msg)
				#msg.address = UR
				#pub.publish(msg)
#forward
		#else:
		if desiredPWM_L + PWM_dx < maxPWMval * axis_y:
		#increase left wheels
			desiredPWM_L = desiredPWM_L + PWM_dx
			msg.pwn = desiredPWM_L
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)
		else:
			#settle left wheels
			desiredPWM_L = maxPWMval * axis_y 
			msg.pwn = desiredPWM_L
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)

		if desiredPWM_R + PWM_dx < maxPWMval * axis_y:
		#increase right wheels
			desiredPWM_R = desiredPWM_R + PWM_dx
			msg.pwn = desiredPWM_R
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)
		else:
			#settle right wheels
			desiredPWM_R = maxPWMval * axis_y
			msg.pwn = desiredPWM_R
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)


	elif axis_y < -0.1:
		#if pressing back
		print 'going backward: ',desiredPWM_L," ",desiredPWM_R," ",axis_y, " ", axis_x
		msg.direction = 114
		if desiredPWM_L - PWM_dx > maxPWMval * axis_y:
			#decrease left wheels
			desiredPWM_L = desiredPWM_L - PWM_dx
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)
			
		else:
			#settle left wheels
			desiredPWM_L = maxPWMval * axis_y
			msg.pwn = abs(desiredPWM_L)
			msg.address = BL
			pub.publish(msg)
			msg.address = UL
			pub.publish(msg)


		if desiredPWM_R - PWM_dx > maxPWMval * axis_y:
			#decrease right wheels
			desiredPWM_R = desiredPWM_R - PWM_dx
			msg.pwn = abs(desiredPWM_R)
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)

		else:
			#settle right wheels
			desiredPWM_R = maxPWMval * axis_y
			msg.pwn = abs(desiredPWM_R)
			msg.address = BR
			pub.publish(msg)
			msg.address = UR
			pub.publish(msg)
	else:
		#print 'no movement input.. ',desiredPWM_L," ",desiredPWM_R," ",axis_y, " ", axis_x
		stop()


	#bin movement
	global bin_pos
	#pressing B alternates bin linear actuator position
	if button_b == 1:
		print "pressed button B: ",bin_pos
		
		# change direction of bin movement
		msg.address = LB
		msg.pwn = 255
		
		if bin_pos == 0:
			bin_pos = 1
			msg.direction = 'q' # move bin actuator up
		elif bin_pos == 1:
			bin_pos = 0
			msg.direction = 'a' # move bin actuator down
		pub.publish(msg)

		# wait for bin movement to complete
		time.sleep(5)
		
		# tell the motor to stahp
		msg.direction = ' '
		pub.publish(msg)

	#excavator linear actuator movement
	global ex_pos
	#pressing X alternates excavator linear position
	if button_x == 1:
		print "pressed button X: ",ex_pos
		
		# change direction of excavator linear actuator
		msg.address = LB
		msg.pwn = 255
		
		if ex_pos == 0:
			ex_pos = 1
			msg.direction = 'w' # move linear actuator up 
		elif ex_pos == 1:
			ex_pos = 0
			msg.direction = 's' # move linear actuator down
		pub.publish(msg)

		# wait for bin movement to complete
		time.sleep(5)
		
		# tell the motor to stahp
		msg.direction = ' '
		pub.publish(msg)
	
	# excavator motor movement
	global bm_pos
	msg.address = BM
	
	#pressing A alternates linear actuator position
	if button_a == 1:
		print "excavation motor spinning: ",button_a,bm_pos
		# change direction of excavator linear actuator
		bm_pos = 1
		msg.direction = 'f'
		msg.pwn = 255
		pub.publish(msg)
	elif button_a == 0 and bm_pos == 1:
		print "linear actuator stopping: ",button_a,bm_pos
		bm_pos = 0
		msg.direction = ' '
		msg.pwn = 0
		pub.publish(msg)

def x360():
  
	rospy.Subscriber("joy", Joy, callback_joy)
	rospy.init_node('mancontrol', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	msg = controller_i2c()
	while not rospy.is_shutdown():
	  move()
	  rate.sleep()

if __name__ == '__main__':
  try:
	x360()
  except rospy.ROSInterruptException:
	pass
