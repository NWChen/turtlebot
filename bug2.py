#!/usr/bin/env python

import numpy as np
import rospy
import sys
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from math import isnan, pi, sqrt

class Rotation(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class Bug(object):
    def __init__(self):
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'

        # reset odometry
        self.r = rospy.Rate(20)
        for a in range(10):
            self.reset_odom.publish(Empty())
            self.r.sleep()
        self.tf_listener = tf.TransformListener()

        # constants
        self.linear_speed = 1.0 #0.5
        self.angular_speed = 1.0 #0.5
        self.fwd_range, self.r_range, self.l_range = np.inf, np.inf, np.inf
        self.min_range = np.inf
        self.pos, self.quat, self.rot = Point(), Quaternion(), Rotation()

        # wait for tf buffer to fill. this MUST be in init.
        rospy.sleep(2)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def set_goal(self, goal):
        self.goal = goal 

    def scan_callback(self, msg):
        midpoint = len(msg.ranges)/2
        bandwidth = len(msg.ranges)/3
        self.fwd_range = min(msg.ranges[midpoint-10:midpoint+10])
        self.l_range = min(msg.ranges[-bandwidth:-1])
        self.r_range = min(msg.ranges[0:bandwidth])
        if isnan(self.fwd_range):
            self.fwd_range = np.inf
        if isnan(self.r_range):
            self.r_range = np.inf
        if isnan(self.l_range):
            self.l_range = np.inf
        self.min_range = min(msg.ranges) if not isnan(min(msg.ranges)) else np.inf

    # Update position (x,y,z) and quaternion orientation (x,y,z,w).
    def odom_callback(self, msg):
        (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        self.pos = Point(*trans)
        self.quat = Quaternion(*rot)
        euler = euler_from_quaternion((self.quat.x, self.quat.y, self.quat.z, self.quat.w))
        self.rot.x, self.rot.y, self.rot.z = euler[0], euler[1], euler[2]

    # Stop abruptly.
    def stop(self):
        vel_msg = Twist()
        self.cmd_vel.publish(vel_msg)

    # Move forward in current heading (fwd relative to robot base frame).
    def move_forward(self, goal_dist=0.1):
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        x0, y0 = self.pos.x, self.pos.y
        dist = 0

        while dist < goal_dist:
            dist = sqrt(pow((self.pos.x - x0), 2) + pow((self.pos.y - y0), 2))
            self.cmd_vel.publish(move_cmd)
        self.stop()

    # Turn wrt inertial frame (radians).
    def inertial_turn(self, theta):
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed if theta > 0 else -self.angular_speed
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < theta/move_cmd.angular.z:
            self.cmd_vel.publish(move_cmd)
        self.stop()

    # Turn until theta radians wrt operating frame (not inertial frame). +theta = left, -theta=right.
    def turn(self, theta):
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed if theta >= self.rot.z else -self.angular_speed

        while not approxeq([self.rot.z], [theta], epsilon=0.05):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        self.stop()

    def sensor_extremity(self, direction): # 'left', 'right'
        INCREMENT = pi/6
        out = np.inf
        reset_rot = self.rot.z
        if direction=='left':
            self.inertial_turn(theta=INCREMENT)
            out = min(self.fwd_range, self.l_range)
            self.inertial_turn(theta=-INCREMENT)
        elif direction=='right':
            self.inertial_turn(theta=-INCREMENT)
            out = min(self.fwd_range, self.r_range)
            self.inertial_turn(theta=INCREMENT)
        return out

# Tests for approximate equality in two iterables A, B
def approxeq(A, B, epsilon=0.1):
    for i, a in enumerate(A):
        if abs(a - B[i]) > epsilon:
            return False
    return True

# Main loop for Bug2 algorithm
def run(bug, goal):
    # constants
    r = rospy.Rate(20)
    K_RANGE = 0.95
    K_THETA = pi/8
    K_LEFT, K_FWD, K_RIGHT = pi/2, 0, -pi/2
    i = 0

    # states
    HEAD_TO_GOAL = 0
    HIT_OBSTACLE = 1
    SEARCH_FOR_EXIT = 8
    EXIT_MLINE = 2
    FOLLOW_OBSTACLE = 3
    ORIENT_LEFT = 4
    ORIENT_RIGHT = 5
    ORIENT_MLINE = 6
    AT_GOAL = 7
    FAILURE = -1

    state = HEAD_TO_GOAL 
    hit_point = [np.inf, np.inf]
    while not rospy.is_shutdown():
        #import pdb; pdb.set_trace()

        if state==HEAD_TO_GOAL:
            rospy.loginfo('HEAD_TO_GOAL')
            bug.turn(K_FWD)
            bug.move_forward()
            if approxeq([bug.pos.x, bug.pos.y], [goal.x, goal.y], epsilon=0.5) or (approxeq([bug.pos.y], [goal.y], epsilon=0.5) and bug.pos.x > goal.x):
                state = AT_GOAL
            elif bug.min_range > K_RANGE:
                state = HEAD_TO_GOAL
            else:
                state = HIT_OBSTACLE

        elif state==HIT_OBSTACLE:
            rospy.loginfo('HIT_OBSTACLE')
            i = 0
            if i>15 and approxeq([bug.pos.x, bug.pos.y], hit_point, epsilon=epsilon):
                state = FAILURE
            else: 
                hit_point = [bug.pos.x, bug.pos.y]
                state = SEARCH_FOR_EXIT

        elif state==SEARCH_FOR_EXIT:
            rospy.loginfo('SEARCH_FOR_EXIT')
            bug.inertial_turn(theta=K_THETA)
            epsilon = 0.5 if i<15 else K_RANGE+1.5
            if (bug.r_range < K_RANGE) or (bug.fwd_range < K_RANGE):
                state = SEARCH_FOR_EXIT
            else:
                state = EXIT_MLINE

        elif state==EXIT_MLINE: # 2
            rospy.loginfo('EXIT_MLINE')
            bug.move_forward()
            state = FOLLOW_OBSTACLE

        elif state==FOLLOW_OBSTACLE: # 3
            i += 1
            rospy.loginfo('FOLLOW_OBSTACLE (steps=%d)' % i)
            bug.move_forward(goal_dist=0.1) #!!!!!

            if bug.sensor_extremity('right') < K_RANGE:
                state = ORIENT_LEFT
            elif bug.sensor_extremity('left') < K_RANGE:
                state = ORIENT_RIGHT
            elif i>15 and approxeq([bug.pos.y], [0], epsilon=0.25):
                state = ORIENT_MLINE
            else:
                state = ORIENT_RIGHT

        elif state==ORIENT_LEFT: # 4
            rospy.loginfo('ORIENT_LEFT')
            bug.inertial_turn(theta=K_THETA)
            state = FOLLOW_OBSTACLE

        elif state==ORIENT_RIGHT:
            rospy.loginfo('ORIENT_RIGHT')
            bug.inertial_turn(theta=-K_THETA)
            state = FOLLOW_OBSTACLE

        elif state==ORIENT_MLINE:
            rospy.loginfo('ORIENT_MLINE')
            bug.turn(theta=K_FWD)
            state = HEAD_TO_GOAL

        elif state==AT_GOAL:
            rospy.loginfo('Arrived at goal.')
            bug.stop()

        else:
            rospy.loginfo('Failed.')
            bug.stop()

        r.sleep() # 20Hz

if __name__ == '__main__':
    rospy.init_node('bug_2')
    rospy.loginfo('Starting bug_2')

    # Make sure simulation time works
    while not rospy.Time.now():
        pass

    # Run bug2 planning algorithm
    rospy.loginfo('Starting planning')
    bug = Bug()
    goal = Point()
    goal.x = 10
    run(bug, goal)
    rospy.loginfo('Shutdown')
