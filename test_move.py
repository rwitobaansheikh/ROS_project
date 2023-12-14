#! /usr/bin/env python3
import time
import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

# Global variables
# States
current_state = "Forward"
last_state = None

last_print_time = None
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom = None
total_distance = 0.0
prev_distance = 0
target_yaw = None
current_yaw = None

# Global variable to track the last turn time
last_turn_time = 0
turn_delay = 1.0  # Delay 


# State definitions
FORWARD = "Forward"
AVOID_OBSTACLE_LEFT = "AvoidLeft"
AVOID_OBSTACLE_RIGHT = "AvoidRight"

range_vals=[0.0,0.0,0.0,0.0]

def Rangefl(msg):
    #rospy.loginfo(msg.range)
    range_vals[0]=msg.range
    
def Rangefr(msg):
    #rospy.loginfo(msg.range)
    range_vals[1]=msg.range

def Rangerl(msg):
    #rospy.loginfo(msg.range)
    range_vals[2]=msg.range
    
def Rangerr(msg):
    #rospy.loginfo(msg.range)
    range_vals[3]=msg.range
    #print(msg)
    #print(msg[range])
    

def clbk_laser(msg):
    global current_state, last_print_time, prev_distance

    if last_print_time is None:
        last_print_time = rospy.Time.now()
    
    step = 1
    
    # TODO FINISH THIS********************
    '''lfront = slice(0*step,18*step)
    rfront = slice(0*step,360*step)
    right = slice(252*step, 288*step)'''
# ****************************************

    regions = {
        'front': min(min(msg.ranges[:36] + msg.ranges[684:]), 10),
        'right': min(min(msg.ranges[504:576]), 10),
        'left': min(min(msg.ranges[144:216]), 10),
    }

    # State transition logic based on LiDAR data
    safe_distance = 0.75  # Example safe distance threshold
    if regions['front'] < safe_distance:
        if regions['left'] < regions['right']:
            current_state = AVOID_OBSTACLE_RIGHT
        else:
            current_state = AVOID_OBSTACLE_LEFT
    else:
        current_state = FORWARD

    # Call state action 
    fsm_action()

def odom_clbk(msg):
    global odom, total_distance
    if odom is not None:
        dx = msg.pose.pose.position.x - odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - odom.pose.pose.position.y

        distance = np.sqrt(dx**2+dy**2)

        total_distance += distance

    odom = msg

    rospy.loginfo("Total Traveled Distance: {:.2f} meters".format(total_distance))


def imu_clbk(msg):
    global target_yaw

    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
        )

    # convert to Euler 
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    if target_yaw is None:
        target_yaw = yaw
    #rospy.loginfo(f"Target direction: {target_yaw}\nCurrent direction: {yaw}\nError:{abs(target_yaw-yaw)}")
    '''
    TODO Implement these ideas:

    yaw_difference = target_yaw - yaw

    # Normalize the angle difference to the range [-pi, pi] or [-180, 180]
    yaw_difference = normalize_angle(yaw_difference) pseudo

    if yaw_difference > angle_tolerance:
        # Turn right
        send_motor_command(right_turn_speed)
    elif yaw_difference < -angle_tolerance:
        # Turn left
        send_motor_command(left_turn_speed)
    else:
        # On the right track, proceed forward
        send_motor_command(forward_speed)
        
    ........................................
        while True:
    yaw = get_current_yaw()  # Obtain current yaw from IMU
    yaw_error = target_yaw - current_yaw

    if abs(yaw_error) <= angle_tolerance:
        # If error is within the acceptable range, break the loop
        break

    if yaw_error > 0:
        # If error is positive, turn right
        send_motor_command(right_turn_speed)
    else:
        # If error is negative, turn left
        send_motor_command(left_turn_speed)

'''


def fsm_action():
    global current_state, last_turn_time, last_state, prev_distance
    current_time = time.time()

    if current_state == FORWARD:
        if last_state != FORWARD:
            rospy.loginfo("New FORWARD state detected. Resetting distance.")
            prev_distance = total_distance
        if range_vals[0]<0.500 or range_vals[1]<0.500:
            rospy.loginfo("New FORWARD state detected. Resetting distance.")
            prev_distance = total_distance
        move_forward()
        distance = total_distance-prev_distance

        rospy.loginfo(f"Distance travelled: {distance}")

    elif current_state == AVOID_OBSTACLE_LEFT and (current_time - last_turn_time) > turn_delay:
        turn_left()
        last_turn_time = current_time
    elif current_state == AVOID_OBSTACLE_RIGHT and (current_time - last_turn_time) > turn_delay:
        turn_right()
        last_turn_time = current_time
    last_state = current_state


def turn_left():
    msg = Twist()
    msg.linear.x = -0.1  # Slight backward movement
    msg.angular.z = 1.0  # Left turn
    pub.publish(msg)
    rospy.sleep(0.5)  # Back up for a short duration before turning

def turn_right():
    msg = Twist()
    msg.linear.x = -0.1  # Slight backward movement
    msg.angular.z = -1.0  # Right turn
    pub.publish(msg)
    rospy.sleep(0.5) 


def move_forward():
    msg = Twist()
    msg.linear.x = 0.5  # Forward speed
    msg.angular.z = 0   # No rotation
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('obstacle_avoidance_fsm')
    
    rospy.Subscriber("/range/fl",Range,Rangefl)
    rospy.Subscriber("/range/fr",Range,Rangefr)
    rospy.Subscriber("/range/rl",Range,Rangerl)
    rospy.Subscriber("/range/rr",Range,Rangerr)

    
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/odom', Odometry, odom_clbk)
    rospy.Subscriber('/imu', Imu, imu_clbk)


    rospy.spin()

if __name__ == '__main__':
    main()