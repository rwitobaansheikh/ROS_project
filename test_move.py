#! /usr/bin/env python3
import time
import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


# States
current_state = "Forward"
last_state = None

# Global variables
last_print_time = None
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom = None
total_distance = 0.0
prev_distance = 0
target_yaw = 0
current_yaw = None
distance_in_fwd = 0
yaw_error = 0

# Range sensors
range_fl = 0.7 
range_fr = 0.7
# Global variable to track the last turn time
last_turn_time = 0
turn_delay = 1.0  # Delay 


# State definitions
FORWARD = "Forward"
AVOID_OBSTACLE_LEFT = "AvoidLeft"
AVOID_OBSTACLE_RIGHT = "AvoidRight"
CORRECTION = "Correction"

# Range sensors clbks
def Rangefl(msg):
    global range_fl
    range_fl = msg.range

def Rangefr(msg):
    global range_fr
    range_fr = msg.range


def clbk_laser(msg):
    global current_state, last_print_time, distance_in_fwd, yaw_error, range_fl, range_fr

    if last_print_time is None:
        last_print_time = rospy.Time.now()
    
    step = 2
  
    lfront = slice(0*step,18*step)
    rfront = slice(342*step,360*step)
    right = slice(252*step, 288*step)
    left = slice(72*step, 108*step)

    regions = {
        'front': min(min(msg.ranges[lfront] + msg.ranges[rfront]), 10),
        'right': min(min(msg.ranges[right]), 10),
        'left': min(min(msg.ranges[left]), 10),
    }


    # State transition logic based on LiDAR data
    safe_distance = 0.6  # Safe distance threshold for laser
    safe_dist_rng = 0.5
    if regions['front'] < safe_distance or range_fl < safe_dist_rng or range_fr < safe_dist_rng :
        if regions['left'] < regions['right']:
            current_state = AVOID_OBSTACLE_RIGHT
        else:
            current_state = AVOID_OBSTACLE_LEFT
    
    elif distance_in_fwd >= 1:
            yaw_error = normalize_angle(0 - current_yaw)
            
            if abs(yaw_error) > 0.1:
                current_state = CORRECTION
            else:
                current_state = FORWARD
    
    else:
        current_state = FORWARD
    
    rospy.loginfo(f"Distance in clbk: {distance_in_fwd}")
 
    fsm_action()                       

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

    

def odom_clbk(msg):
    global odom, total_distance
    if odom is not None:
        dx = msg.pose.pose.position.x - odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - odom.pose.pose.position.y

        distance = np.sqrt(dx**2+dy**2)

        total_distance += distance

    odom = msg

    #rospy.loginfo("Total Traveled Distance: {:.2f} meters".format(total_distance))


def imu_clbk(msg):
    global current_yaw

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
    current_yaw = euler[2]

    rospy.loginfo(f"Target direction: {target_yaw}\nCurrent direction: {current_yaw}\n")


def fsm_action():
    global current_state, last_turn_time, last_state, distance_in_fwd, prev_distance
    current_time = time.time()

    if current_state == FORWARD:
        if last_state != FORWARD:
            #rospy.loginfo("New FORWARD state detected. Resetting distance.")
            prev_distance = total_distance
        move_forward()
        distance_in_fwd = total_distance-prev_distance

        rospy.loginfo(f"FWD Distance travelled in FSM: {distance_in_fwd}")
        rospy.loginfo(f"TOTAL Distance travelled in FSM: {total_distance}")

    elif current_state == CORRECTION:
        yaw_error = normalize_angle(target_yaw - current_yaw)

        # Check if the robot is close enough to the target orientation
        if abs(yaw_error) <= 0.1:  # Threshold to exit correction state
            current_state = FORWARD
            
        else:
            # Proportional control for smoother turning
            Kp = 1.5  # Gain factor, adjust as needed
            angular_velocity = Kp * yaw_error

            # Limit the angular velocity to avoid overcorrection
            max_angular_velocity = 1
            angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

            # Send turn command
            msg = Twist()
            msg.angular.z = angular_velocity
            pub.publish(msg)

    elif current_state == AVOID_OBSTACLE_LEFT and (current_time - last_turn_time) > turn_delay:
        turn_left()
        last_turn_time = current_time
    elif current_state == AVOID_OBSTACLE_RIGHT and (current_time - last_turn_time) > turn_delay:
        turn_right()
        last_turn_time = current_time
    
    rospy.loginfo(f"State: {current_state}\n")
    last_state = current_state


def turn_left():
    msg = Twist()
    msg.linear.x = -0.15  # Slight backward movement
    msg.angular.z = 1.0  # Left turn
    pub.publish(msg)
    rospy.sleep(0.5)  # Back up for a short duration before turning

def turn_right():
    msg = Twist()
    msg.linear.x = -0.15  # Slight backward movement
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

    # Subscribers
    rospy.Subscriber('/range_fl', Range, Rangefl)
    rospy.Subscriber('/range_fr', Range, Rangefr)
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/odom', Odometry, odom_clbk)
    rospy.Subscriber('/imu', Imu, imu_clbk)
    


    rospy.spin()

if __name__ == '__main__':
    main()