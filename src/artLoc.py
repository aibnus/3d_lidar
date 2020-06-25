#!/usr/bin/env python
import rospy
import tf
import sys
from std_msgs.msg import String
from dronekit import connect
from pymavlink import mavutil

first = True
firstl = True
toChange = False
lines = []
count = 0
coord = [0,0,0]
rpy = [0,0,0]
loc = [0,0,0]
init= [0,0,0]
initl=[0,0,0]
z = 0

def tel(msg):
	global toChange, first, lines, count, coord
	print first or toChange
	if toChange or first:
		coord = lines[count].split()
		print coord
		for i in range(len(coord)):
			coord[i] = float(coord[i])
		count += 1
	first = False
	toChange = not toChange

def att(self, attr, value):
	global rpy, first, init
	rpy[0] = v.attitude.roll
	rpy[1] = v.attitude.pitch
	rpy[2] = v.attitude.yaw

	if first:
		init[2] = rpy[2]
		rospy.loginfo(init)
		first= False

def locf(self, attr, value):
	global loc, firstl, initl
	loc[0] = v.location.global_relative_frame.lat
	loc[1] = v.location.global_relative_frame.lon
	loc[2] = v.location.global_relative_frame.alt

	if firstl:
		initl = loc[:]
		rospy.loginfo(initl)
		firstl = False

def rangef(self, attr, value):
	global z
	z = v.rangefinder.distance

if __name__ == '__main__':
	if len(sys.argv) != 3:
		print len(sys.argv)
		print 'Please provide a single input file'
		sys.exit(1)

	rospy.init_node('trans_telem', anonymous=True)
	v = connect('/dev/ttyACM0', baud=115200, wait_ready=True, rate=40)
	rospy.loginfo('Connected')
	v.add_attribute_listener('attitude', att)
	# v.add_attribute_listener('rangefinder', rangef)
	v.location.add_attribute_listener('global_relative_frame', locf)

	f = open(sys.argv[0], 'r')
	lines = f.readlines()
	rospy.loginfo('Got coordinates')
	f.close()
	
	# print lines
#	sys.exit(0)
	
	rospy.loginfo('Added attitude listener')	
	rospy.Subscriber('power', String, tel, queue_size=10)
	r = rospy.Rate(40)
	
	while not rospy.is_shutdown():
		br = tf.TransformBroadcaster()
		tpp = rpy[:]
		cpp = loc[:]
		# dist = z
		rospy.loginfo(cpp[:])
		br.sendTransform((float(cpp[0]-initl[0]), float(cpp[1]-initl[1]), float(cpp[2]-initl[2])), tf.transformations.quaternion_from_euler(tpp[0]-init[0], tpp[1]-init[1], tpp[2]-init[2]), rospy.Time.now(), '/base_link', '/map')

	v.remove_attribute_listener('attitude', att)
	v.location.remove_attribute_listener('local_frame', locf)
	v.close()
