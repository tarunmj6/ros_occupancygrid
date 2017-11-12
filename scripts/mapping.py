#!/usr/bin/env python
import roslib; roslib.load_manifest('assign6')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header

ranges=[]
roll=0.0
pitch=0.0
yaw=0.0
positionx=0.0
positiony=0.0
global occupancy
pub=rospy.Publisher('/robot/cmd_vel',Twist,queue_size=1)

occupancy=[]

def talker1(data1):
	#print data1.pose.pose.orientation
	global roll,pitch,yaw,positionx,positiony
	quaternion =(data1.pose.pose.orientation.x, data1.pose.pose.orientation.y, data1.pose.pose.orientation.z, data1.pose.pose.orientation.w)
	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	positionx=data1.pose.pose.position.x
	positiony=data1.pose.pose.position.y
		
def talker(data):
	msg = Twist()
	global ranges
	for a in range(0,5):
		ranges.append(data.ranges[a])	
	right = data.ranges[0]
	left = data.ranges[4]
	right1 = data.ranges[1]
	left1 =data.ranges[3] 
	if left < right:
		msg.linear.x= left1-0.5
		msg.angular.z=-.28
	elif left > right:
		msg.linear.x= right1-0.5
		msg.angular.z=.28
	elif left == right:
		if left1 < right1:
			msg.linear.x=.15
			msg.angular.z=-.30
		elif right1 < left1:
			msg.linear.x = .15
			msg.angular.z=.30
	pub.publish(msg)
	l=positionx/0.05
	boundary = 600+l	#creating a 6m boundary around the robot
	k=6.0	#maximum value for Y
	for i in range(0,240):
		m=-30.0 #maximum value for X
		for j in range(0,1200):
			occupancy[i][j]=logodds(inverse_range_sensor_model(m,k,positionx,positiony,yaw,ranges))#m and k are the position of cells in terms of meters 
			m=m+.05
		k=k-0.05
	publisher=rospy.Publisher('/map',OccupancyGrid,queue_size=1)
	rate = rospy.Rate(10)
	try:
		rospy.delete_param('/use_sim_time') #Rate.sleep wont work unless you remove this param, the simulator does not appear to publish to /clock
	except:
		print("No parameter to delete")

	mymap = OccupancyGrid( #create my message 2400 pixels wide by 600 pixels long, at 0.05 meters per pixel, AKA 120 meters x 30 meters
		header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), 
		info = MapMetaData(width=1200, height=240, resolution=0.05,map_load_time=rospy.Time.now()))

	#As per http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
	#The data is in row-major order, starting witht he 0,0 origin.
	#Probabilities are in a range from [0,100], with -1 being 0
	for i in range(0,240):
		for j in range(0,1200): 
			mymap.data.append(occupancy[i][j]*100) 

	while not rospy.is_shutdown():
		publisher.publish(mymap)
		rate.sleep()

def inverse_range_sensor_model(xi,yi,x,y,theta,z):
	r=math.sqrt(((xi-x)**2)+((yi-y)**2))
	phi=math.atan2((yi-y),(xi-x))-theta
	thetasens=[-1.5708,-0.785398,0,0.785398,1.5708]
	global angledifference	
	angledifference = []
	for i in range (0,5):
		angledifference.append(abs(phi-thetasens[i]))
	k=np.argmin(angledifference)
	if r > min(5,z[k]+.1) or abs(phi-thetasens[k]) > 0.3:
		return 0
	if z[k]<5 and abs(r-z[k])<.1:
		return 3
	if r<=z[k]:
		return -3

def logodds(l):
	odds=math.exp(l)
	p=odds/(1+odds)
	return p

def listener():
	rospy.init_node('ListenerTalker',anonymous=True)
	rospy.Subscriber('/stage/base_pose_ground_truth',Odometry,talker1)
	rospy.Subscriber('/robot/base_scan', LaserScan,talker)
	rospy.spin()

if __name__=='__main__':
	for i in range(0,240):
		occupancy.append([])	#creatig rows
	for i in range(0,240):
		for j in range(0,1200):
			occupancy[i].append(0.5) #creating columns and initializing all values to 0.5
	listener()
