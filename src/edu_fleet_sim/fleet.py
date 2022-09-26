# ------------------------------------------------------------------------
# Author:      Stefan May
# Date:        18.9.2022
# Description: Pygame-based robot fleet representation for the mecanum simulator
# ------------------------------------------------------------------------

import os
from re import T
import pygame
import rospy
import time, threading
import operator
import numpy as np
from math import cos, sin, pi, sqrt, acos, atan2
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from edu_fleet_sim.msg import WheelSpeed
from edu_fleet_sim.srv import Spawn, Kill, SpawnRequest, SpawnResponse, KillRequest, KillResponse
from .robot import Robot
from copy import copy, deepcopy

class Fleet:

    def __init__(self, T_w2f, name):
        self._T_pose  = deepcopy(T_w2f)
        self._T_r2f_ref = np.matrix([[1, 0, 0],
                                     [0, 1, 0],
                                     [0, 0, 1]])
        
        self._name = name

        self._robots = []
        self._T_f2r  = []

        rospy.Service(str(self._name)+"/spawn", Spawn, self.service_callback_spawn)
        rospy.Service(str(self._name)+"/kill" , Kill , self.service_callback_kill)

        self._lock      = threading.Lock()
        self._sub_joy   = rospy.Subscriber(str(self._name) + "/joy"    , Joy  , self.callback_joy)
        self._sub_twist = rospy.Subscriber(str(self._name) + "/cmd_vel", Twist, self.callback_twist)

    def get_coords(self):
        return [self._T_pose[0, 2], self._T_pose[1, 2]]

    def add_robot(self, T, name):
        T_robot = np.matrix([[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]])

        # only apply rotation to rotation part of pose
        R = T[[0,1], :][:, [0,1]] * self._T_pose[[0,1], :][:, [0,1]]
        T_robot[0, 0] = R[0, 0]
        T_robot[0, 1] = R[0, 1]
        T_robot[1, 0] = R[1, 0]
        T_robot[1, 1] = R[1, 1]

        # only apply translation part of pose
        T_robot[0, 2] = T[0, 2] + self._T_pose[0, 2]
        T_robot[1, 2] = T[1, 2] + self._T_pose[1, 2]

        # print('add robot at location:\n' + str(T_robot))
        self._robots.append(Robot(T_robot, name))

        T_f2r_rot = np.matrix([[T[0, 0], T[0, 1], 0.0],
                               [T[1, 0], T[1, 1], 0.0],
                               [    0.0,     0.0, 1.0]])
        T_f2r_trans = np.matrix([[1.0, 0.0, -T[1, 2]],
                                 [0.0, 1.0,  T[0, 2]],
                                 [0.0, 0.0,      1.0]]) 
        T_f2r = np.linalg.pinv(T_f2r_rot) * np.linalg.pinv(T_f2r_trans)
        # print('add T_f2r:\n' + str(T_f2r))
        # print('T_f2r_trans:\n' + str(T_f2r_trans))         
        self._T_f2r.append(T_f2r)

    def get_robots(self):
        return self._robots

    def service_callback_spawn(self, req):
        self.spawn_robot(req.x, req.y, req.theta, req.name)
        response = SpawnResponse(req.x, req.y, req.theta, req.name)
        return response

    def service_callback_kill(self, req):
        self.kill_robot(req.name)
        response = KillResponse(True)
        return response

    def callback_twist(self, data):
        vx = data.linear.x
        vy = data.linear.y
        omega = data.angular.z
        velocity = np.matrix([[vx], [vy], [omega]])

        # This is the inverse of the matrix transforming
        # the first robot to the common fleet coordinate system
        # T_r2f_ref_inv = np.linalg.pinv(self._T_f2r[0])

        # The fleet's reference pose is coupled with
        # the pose of the first robot
        # T_pose_fleet = self._robots[0]._T_pose * T_r2f_ref_inv
        # T_pose_fleet_inv = np.linalg.pinv(T_pose_fleet)

        for robot in range(len(self._robots)):
            # Transformation matrix 
            # T_pose_r2f = T_pose_fleet_inv * robot._T_pose
            # T_pose_r2f_inv = np.linalg.pinv(T_pose_r2f)

            # calculate translational part in coordinate system of robot
            # nx = -T_pose_r2f[1, 2] * omega
            # ny =  T_pose_r2f[0, 2] * omega
            # vtx_robot = T_pose_r2f_inv[0, 0] * (vx+nx) + T_pose_r2f_inv[0, 1] * (vy+ny)
            # vty_robot = T_pose_r2f_inv[1, 0] * (vx+nx) + T_pose_r2f_inv[1, 1] * (vy+ny)

            velocity_robot = self._T_f2r[robot] * velocity
            # print('velocity_robot:\n' + str(velocity_robot))
            # print('T_f2r:\n' + str(self._T_f2r[robot]))
            # and add it to the translational part
            # print('set velocity for robot' + str(robot) + ':\n' + str(velocity_robot))
            self._robots[robot].set_velocity(velocity_robot[0, 0], velocity_robot[1, 0], omega)

        self._last_command = rospy.Time.now()

    def callback_joy(self, data):
        
        if (len(self._robots)==0):
            return

        twist_msg = Twist()
        twist_msg.linear.x = data.axes[1]
        twist_msg.linear.y = data.axes[0]
        twist_msg.angular.z = data.axes[2]

        self.callback_twist(twist_msg)
