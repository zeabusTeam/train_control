#!/usr/bin/env python2
# FILE			: first_control.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, September 24 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

import rospy

import thread

from std_msgs.msg import Float64

from train_control.msg import SingleState

class Control:

    def __init__( self ):
    
        rospy.init_node( "control" )

        self.state_message = SingleState()
        self.target_message = Float64()
        self.out_force = Float64()

        self.state_lock = thread.allocate_lock()
        self.target_lock = thread.allocate_lock()

        self.pub_force = rospy.Publisher( "/force" , Float64 , queue_size = 1 )

        self.sub_state = rospy.Subscriber( "/state" , SingleState , self.callback_state )

        self.sub_target = rospy.Subscriber( "/target" , Float64 , self.callback_target )

    def force( self , current_point , current_velocity , 
                current_acceleration , target_point , diff_time ):
        answer = 0.0
        print( "(point , velocity , acceleration , target ) : ( {:6.3f} , {:6.3f} , {:6.3f} , {:6.3f}".format( current_point , current_velocity , current_acceleration , target_point ) ) 
#==================================== EDIT at HERE ============================================
        #target module 
        if target_point < 0:
            target = target_point + abs(current_point)
        else:
            target = target_point - current_point
        print('target =',target)
        
        #velocity module    
        if (-0.1<target<0.1):
            target_velocity = 0
            answer = 0
        elif (-0.5<target<0.5):
            if target > 0:
                target_velocity = 0.1
            else:
                target_velocity = -0.1
        else:
            target_velocity = target*0.2
        print('target velocity =', target_velocity)
        
        #acceleration module  
        if target_velocity > 0: 
            if target_velocity - current_velocity > 0:
                target_acceleration = target_velocity*0.5
            else:
                target_acceleration = target_velocity*(-0.5)
        elif target_velocity < 0:
            if target_velocity - current_velocity > 0:
                target_acceleration = target_velocity*(-0.5)
            else:
                target_acceleration = target_velocity*(0.5)
        else:
            target_acceleration = 0
        print('target acceleration =', target_acceleration)

        
        #force module
        answer = 1.15*(target_acceleration*20)        
#==============================================================================================
        print( "command force is " + str( answer ) + "\n")
        return answer

    def loop_control( self ):
        previous_time = rospy.get_rostime()
        current_time = rospy.get_rostime()
        diff_time = 0.1
        rate = rospy.Rate( 10 )
        temp_state = SingleState()
        self.target_message.data = 0.0
        while( not rospy.is_shutdown() ):
            with self.target_lock:
                temp_target = self.target_message.data
            with self.state_lock:
                temp_state = self.state_message

            self.out_force.data = self.force( temp_state.point , temp_state.velocity , 
                    temp_state.acceleration , temp_target , diff_time )

            self.pub_force.publish( self.out_force )
            rate.sleep()
            current_time = rospy.get_rostime()
            diff_time = (current_time - previous_time).to_sec()
            previous_time = current_time

    def callback_target( self , message ):
        with self.target_lock:
            self.target_message = message

    def callback_state( self , message ):
        with self.state_lock:
            self.state_message = message

if __name__=="__main__":
    control = Control()
    control.loop_control()
