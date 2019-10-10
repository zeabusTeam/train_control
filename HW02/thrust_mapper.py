#!/usr/bin/env python

import rospy
import math
import numpy as np 
from train_control.msg import ForcePlane, ForceThruster

class ThrustMapper:

    def __init__(self):
        
        rospy.init_node('thrust_mapper', anonymous=True)

        self.force_message = ForcePlane()
        self.out_thrust = ForceThruster()

        self.pub_thrust = rospy.Publisher( '/thruster', ForceThruster , queue_size = 1)
        self.sub_force = rospy.Subscriber('/target', ForcePlane, self.callback_force)
        
        cos_45 = math.cos(math.radians(45))
        sin_45 = math.sin(math.radians(45))

        self.thruster_direction = np.array([[cos_45, sin_45],
                                            [-cos_45, sin_45],  
                                            [cos_45, -sin_45],   
                                            [-cos_45, -sin_45]
                                            ]) 

        self.thruster_position = np.array([ [0.25,0.2],
                                            [0.25,-0.2],
                                            [-0.4,0.2],
                                            [-0.4,-0.2]
                                            ])

        self.thruster_moment = np.array([[
                            np.cross(self.thruster_position[0], self.thruster_direction[0]),
                            np.cross(self.thruster_position[1], self.thruster_direction[1]),
                            np.cross(self.thruster_position[2], self.thruster_direction[2]),
                            np.cross(self.thruster_position[3], self.thruster_direction[3])
                            ]])

        self.thruster_moment = self.thruster_moment.T
        
        self.direction = np.concatenate([self.thruster_direction,self.thruster_moment], axis=1)
        
        self.direction_inverse = np.linalg.pinv(self.direction)
    
    
    def thrust(self, force_x, force_y, moment_z):
        nforce = np.array([[force_x, force_y, moment_z]])
        print('Force input (X, Y, moment of Z):', force_x, force_y, moment_z)
        thrust_map = np.matmul(nforce, self.direction_inverse) #array.shape = (1,4)
        print('Four thrusters have been mapped(1-4):', thrust_map)
        return thrust_map 
    
    def loop_control(self):
        temp_force = ForcePlane()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            temp_force = self.force_message
            thrust = self.thrust(temp_force.force_x, temp_force.force_y, temp_force.moment_z)
            self.out_thrust.force = list(thrust[0])
            self.pub_thrust.publish(self.out_thrust)
            rate.sleep()

    def callback_force(self, message):
        self.force_message = message
    
 

if __name__ == "__main__":
    thrustmapper = ThrustMapper()
    thrustmapper.loop_control()
