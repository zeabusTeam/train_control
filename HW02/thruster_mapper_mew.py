#!/usr/bin/env python2
'''
File        : thruster_mapper_mew.py
Author      : music
'''
from __future__ import print_function

import math
import rospy
import numpy
from train_control.msg import ForceThruster, ForcePlane
class ThrusterMapper:
    def __init__( self ):

        rospy.init_node('control_thruster')
        self.force_message = ForcePlane()
        self.thruster_messagee = ForceThruster()

        self.pub_thruster = rospy.Publisher('thruster', ForceThruster, queue_size = 1)
        self.sub_force = rospy.Subscriber('/target', ForcePlane, self.callback_force)

        cos_45 = math.cos(math.radians(45))
        sin_45 = math.sin(math.radians(45))
        self.direction_linear = numpy.array([
            [cos_45, -sin_45],   # thruster id 1
            [cos_45, sin_45],  # thruster id 2
            [-cos_45, -sin_45],    # thruster id 3
            [-cos_45, sin_45]    # thruster id 4
        ])
        self.distance = numpy.array([
            [0.25, 0.2], # thruster id 1
            [0.25, -0.2], # thruster id 2
            [-0.4, 0.2], # thruster id 3
            [-0.4, -0.2], # thruster id 4
        ])
        self.direction_angular = numpy.array([
            [numpy.cross(self.distance[0], self.direction_linear[0])], # thruster id 1
            [numpy.cross(self.distance[1], self.direction_linear[1])], # thruster id 2
            [numpy.cross(self.distance[2], self.direction_linear[2])], # thruster id 3
            [numpy.cross(self.distance[3], self.direction_linear[3])] # thruster id 4
        ])
        self.direction = numpy.concatenate((self.direction_linear, self.direction_angular), axis=1)
        self.direction_inverse = numpy.linalg.pinv(self.direction)
    
    def thruster_mapper(self, force_x, force_y, moment_z):
        command = numpy.array([
            [force_x],
            [force_y],
            [moment_z],
            ])
        print('Input [force x,force y, moment z][shape = (3x1)]:')
        print(command)
        force_thrust = numpy.matmul(command.T , self.direction_inverse)
        print('Force Thruster (1-4):')
        print(force_thrust.T)

    def force(self):
        temp_force = ForcePlane()
        thrust = self.thruster_mapper(temp_force.force_x, temp_force.force_y, temp_force.moment_z)     
    
    def callback_force(self, message):
        self.force_message = message

if __name__ == '__main__':
    thruster_mapper_mew = ThrusterMapper()
    thruster_mapper_mew.force()
