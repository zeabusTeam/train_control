#!/usr/bin/env python2
import math
import numpy
import rospy
from train_control.msg import ForcePlane, ForceThruster

input_fx = 0
input_fy = 0
input_momentz = 0

rospy.Subscriber("/target", ForcePlane, callback)
def callback(data):
    global input_fx,input_fy,input_momentz
    input_fx = data.force_x
    input_fy = data.force_y
    input_momentz = data.moment_z

cos45 = math.cos(math.radians(45))
sin45 = math.cos(math.radians(45))

direction_linear = numpy.array([
          [cos45,-sin45,0], # 1
          [cos45,sin45,0], # 2
          [-cos45,-sin45,0], # 3
          [-cos45,sin45,0] # 4
         ])

distance = numpy.array([
            [0.25,0.2,0], # 1
            [0.25,-0.2,0], # 2
            [-0.4,0.2,0], # 3
            [-0.4,-0.2,0] # 4
])

direction_angular = numpy.array([numpy.cross(distance[0], direction_linear[0]),
                                numpy.cross(distance[1], direction_linear[1]),
                                numpy.cross(distance[2], direction_linear[2]),
                                numpy.cross(distance[3], direction_linear[3])
])

direction = numpy.concatenate((direction_linear, direction_angular), axis = 1)

direction_inverse = numpy.linalg.pinv(direction)
inputs = numpy.array([input_fx,input_fy,0,0,0,input_momentz])
# inputs = numpy.array([1,1,0,0,0,0])
answer = numpy.matmul(direction_inverse.T, inputs.T)
print(answer)
pub = rospy.Publisher("/thruster", ForceThruster, queue_size = 10)
pub.publish(answer)
