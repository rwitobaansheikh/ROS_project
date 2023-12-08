#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x=0.0
y=0.0
theta=0.0

range_vals=[0.0,0.0,0.0,0.0]

def newOdom(msg):
    global x
    global y
    global theta
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
def Rangefl(msg):
    rospy.loginfo(msg.range)
    range_vals[0]=msg.range
    
def Rangefr(msg):
    rospy.loginfo(msg.range)
    range_vals[1]=msg.range

def Rangerl(msg):
    rospy.loginfo(msg.range)
    range_vals[2]=msg.range
    
def Rangerr(msg):
    rospy.loginfo(msg.range)
    range_vals[3]=msg.range
    #print(msg)
    #print(msg[range])
    
rospy.init_node("reading_range_msg")

subfl=rospy.Subscriber("/range/fl",Range,Rangefl)
subfr=rospy.Subscriber("/range/fr",Range,Rangefr)
subrl=rospy.Subscriber("/range/rl",Range,Rangerl)
subrr=rospy.Subscriber("/range/rr",Range,Rangerr)
sub=rospy.Subscriber("/odom",Odometry,newOdom)

pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)

#rospy.spin()

speed = Twist()
r=rospy.Rate(4)

#goal = Point()
#goal.x = 5
#goal.y = 5

while not rospy.is_shutdown():
    #inc_x = goal.x - x
    #inc_y = goal.y - y
    
    #angle_to_goal = atan2(inc_y, inc_x)
    #if abs(angle_to_goal-theta) > 0.1:
    if range_vals[0]<0.500 or range_vals[1]<0.500:
        speed.linear.x = 0.0
        speed.angular.z = 0.3    
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        
    pub.publish(speed)
    r.sleep()
