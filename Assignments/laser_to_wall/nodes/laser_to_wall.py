#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_to_wall')
import rospy
import math
from sensor_msgs.msg import LaserScan
from laser_to_wall.msg import WallScan

THRESHOLD = 1.0

class wallscan():
	def __init__(self):
		self.pub = rospy.Publisher('/wall_scan', WallScan)
		self.sub = rospy.Subscriber('/base_scan', LaserScan, self.callback_laser)

	def callback_laser(self, msg):
		#print msg.ranges
		#print msg.angle_max		
		scanmessage = WallScan()
		right_index = math.ceil(((-90.0/180.0*math.pi) - msg.angle_min)/msg.angle_increment)
		left_index = math.ceil(((90.0/180.0*math.pi) - msg.angle_min)/msg.angle_increment)
		#print left_index;
		#print right_index;
		if (msg.ranges[int(left_index)] < THRESHOLD):
			scanmessage.wall_left = True
			#print 'left wall'
		
		if (msg.ranges[int(right_index)] < THRESHOLD):
			scanmessage.wall_right = True
			#print 'right wall'

		if (msg.ranges[240] < THRESHOLD):
			scanmessage.wall_front = True
			#print 'front wall'
	
		self.pub.publish(scanmessage)


if __name__ == '__main__':
	rospy.init_node('wallscan')
	walscanner = wallscan()
	rospy.spin()
