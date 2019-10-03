#!/usr/bin/env python2
"""
FILE			: thruster_mapper.py
AUTHOR		    : K.Supasan
CREATE ON	    : 2019, May 26 (UTC+0)
MAINTAINER	    : K.Supasan
ORIGINAL FILE   : K.Supakit 2018/11/14

README
  This code have response about command to use matrix distribute for from world frame to
      robot frame.
  In the future, I think this node should have process to detect about frame id to checkout
      about piority to use so that is the future time.
  If above line have I think we must to use thread manage about that process
  About tf library. In different between python and cpp style coding.
      Python language have purpose about to interesting process and how to
      CPP language have stlye to interesting about theory and ristric
      All that make python don't have quaternion struct please ref02
      You will know quaternion have been representing in mode numpy.dyarray for parameter
          you must to pass by use tuple to pass parameter
  In noted of ref02 you will know this tf library not optimized for speed!
  Axes 4-str : will have affect about output and input. Only quaternion don't affect from this
      first str will be 's'tatic or 'r'otation
          - 's' will return euler from that quaternion to origin
          - 'r' will return euler form origin to that quaternion
  Axes 4-tuple :
      Array or tuple will representing to ix+jy+kz+w to [ x , y , z , w ]

  In new version we remove tf and move process to control_fuzzy or part beforce send data

REFERENCE
  ref01 : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
  ref02 : http://docs.ros.org/melodic/api/tf/html/python/transformations.html#module-tf.transformations
  ref03 : https://pyformat.info/
  ref04 : https://www.w3schools.com/python/ref_func_print.asp
  ref05 : https://docs.python.org/3/whatsnew/3.0.html
  ref06 : https://docs.python.org/2/library/thread.html
  ref07 : wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
  ref08 : http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
  ref09 : https://www.geeksforgeeks.org/sleep-in-python/
"""

from __future__ import print_function

import math
import time
import rospy
import numpy
import thread
import constant
from lookup_pwm_force import lookup_pwm_force
from zeabus_utility.srv import SendThrottle,  SendControlCommand, GetAUVState
from zeabus_utility.srv import SendControlCommandResponse
from zeabus_utility.msg import ControlCommand, Int16Array
from std_msgs.msg import Header

class ThrusterMapper:
    """
    function init that is function to constructor class
    I will use to assign about const data that mean that data will not change
    """

    def __init__(self):

        rospy.init_node('control_thruster')

        # Becuase python always open signal to receive signal in rosnode that make us
        #   Have to declare variable in critcal before setup variable in ros system   
        self.control_message = ControlCommand()
        self.mission_message = ControlCommand()
        self.thruster_message = Int16Array()
        # Two line above will have critical section for read and write that make us have to
        #   use lock to protec about critical senstion
        self.control_lock = thread.allocate_lock()
        self.mission_lock = thread.allocate_lock() 

        # In above 4 variable we have data and lock for manage about critical section next
        #   we have to manage about time out of data
        self.control_stamp = rospy.get_rostime()
        self.mission_stamp = rospy.get_rostime()

        # Setup about variable in ROS System to use for connect with other node
        self.client_throttle = rospy.ServiceProxy(
            '/hardware/thruster_throttle', SendThrottle)

        self.pub_throttle = rospy.Publisher( "/control/thruster_throttle"
            , Int16Array 
            , queue_size = 1)

        self.client_state = rospy.ServiceProxy(
            '/fusion/auv_state', GetAUVState)

        self.server_service = rospy.Service(
#        self.server_service = rospy.Subscriber(
            '/control/thruster', SendControlCommand(), self.callback_service )

        self.server_subscriber = rospy.Subscriber(
            '/control/thruster' , ControlCommand , self.callback_subscriber )

        self.server_force_velocity = rospy.Subscriber(
            '/control/thrusters' , ControlCommand , self.callback_subscribers )
        self.lookup_handle = lookup_pwm_force(
            "zeabus_control", "scripts", "throttle_force_table.txt")

        cos_45 = math.cos(math.radians(45))
        sin_45 = math.sin(math.radians(45))

        # In the below function we use to declare matrix for using about calculating 
        # force and torque of robot

        # this variable will show about position to calculate force
        self.direction_linear = numpy.array([
            [0, 0, 1],              # thruster id 0
            [0, 0, 1],              # thruster id 1
            [0, 0, 1],              # thruster id 2
            [0, 0, 1],              # thruster id 3
            [cos_45, -sin_45, 0],   # thruster id 4
#            [0, 0, 0],   # thruster id 4
            [cos_45, sin_45, 0],  # thruster id 5
            [-cos_45, -sin_45, 0],    # thruster id 6
            [-cos_45, sin_45, 0]    # thruster id 7
        ])

        self.distance = numpy.array([
            [0.44, 0.32, -0.023],    # thruster id 0
            [0.44, -0.32, -0.023],   # thruster id 1
            [-0.44, 0.32, -0.023],   # thruster id 2
            [-0.44, -0.32, -0.023],  # thruster id 3
            [0.46, 0.34, -0.023],   # thruster id 4
            [0.46, -0.34, -0.023],  # thruster id 5
            [-0.46, 0.34, -0.023],  # thruster id 6
            [-0.46, -0.34, -0.023]  # thruster id 7
        ])

        # this variable will show about momentum to calculate about rotatio by euler
        self.direction_angular = numpy.array( [
            # thruster id 0
            numpy.cross(self.distance[0], self.direction_linear[0]),
            # thruster id 1
            numpy.cross(self.distance[1], self.direction_linear[1]),
            # thruster id 2
            numpy.cross(self.distance[2], self.direction_linear[2]),
            # thruster id 3
            numpy.cross(self.distance[3], self.direction_linear[3]),
            # thruster id 4
            numpy.cross(self.distance[4], self.direction_linear[4]),
            # thruster id 5
            numpy.cross(self.distance[5], self.direction_linear[5]),
            # thruster id 6
            numpy.cross(self.distance[6], self.direction_linear[6]),
            # thruster id 7
            numpy.cross(self.distance[7], self.direction_linear[7])
        ] )

        self.direction = numpy.concatenate(
            (self.direction_linear, self.direction_angular), axis=1)

        self.direction_inverse = numpy.linalg.pinv(self.direction)

        self.current_quaternion = (0, 0, 0, 1)
        # At ref02 you can use tuple instead numpy.array
        # we have to use current quaternion because input from control will always world frame
        #   that make us have convert world frame to robot frame

        self.header = Header()
        self.header.frame_id = "base_link"

        self.spin_thread = thread.start_new_thread( self.spin , () )

        rate = rospy.Rate( 30 )

        if( constant.THRUSTER_MAPPER_RESULT ):
            self.count_print = constant.THRUSTER_MAPPER_COUNT
        while( not rospy.is_shutdown() ):
            rate.sleep()
            self.client_loop()

        rospy.signal_shutdown( "End of program of thruster_mapper_enu.py")

    def client_loop( self ):

        # Zero process we call prepare process because we have to check time out of data 
        current_time = rospy.get_rostime()
        mission_data = ControlCommand()
        control_data = ControlCommand()

        # Below condition use to decision about command from mission
        temp_time = ( current_time - self.mission_stamp ).to_sec()

        self.mission_lock.acquire( ) # start crtical section of mission data
        if( temp_time > constant.THRUSTER_MAPPER_TIME_OUT ):
            mission_data.mask = constant.FALSE_MASK 
        else:
            mission_data = self.mission_message
        self.mission_lock.release() # end critical section of mission data

        # Below condition use to decision about command from control
        temp_time = ( current_time - self.control_stamp ).to_sec()

        self.control_lock.acquire( ) #start critical section of control data
        use_control = False
        if( temp_time > constant.THRUSTER_MAPPER_TIME_OUT ):
            control_data.mask = constant.FALSE_MASK
        else:
            control_data = self.control_message 
            use_control = True
        self.control_lock.release() # end critical section of control data

        # First process is about get value from command of mission and control
        #   We will make piority in order mission, control and 0 data
        temp_force = []
        for run in range( 0 , 6 ):
            if( mission_data.mask[ run ] ):
                temp_force.append( mission_data.target[ run ] )
                if( constant.THRUSTER_MAPPER_CHOOSE_PROCESS ):
                    print("id {:2d} choose mission data is {:6.2f}".format( 
                        run
                        , mission_data.target[run] ) )
                if( ( run < 3 ) and ( not control_data.mask[ run ] ) ):
                    temp_force[ run ] += control_data.target[ run ]
            elif( use_control ):
                temp_force.append( control_data.target[ run ] )
                if( constant.THRUSTER_MAPPER_CHOOSE_PROCESS ):
                    print( "id {:2d} choose control data is {:6.2f}".format( 
                        run
                        , control_data.target[run] ) )
            else:
                temp_force.append( 0 ) 
                if( constant.THRUSTER_MAPPER_CHOOSE_PROCESS ):
                    print("id {:2d} choose zero data is 0".format( run ) )

        if( abs( temp_force[5] ) > 3 ):
            print("Yaw over")
            temp_force[5] = 0

        force = numpy.array( [
            (temp_force)[0]     * 1
            , (temp_force)[1]   * 1
            , (temp_force)[2]   * 1
            , (temp_force)[3]   * 1
            , (temp_force)[4]   * 1
            , (temp_force)[5]   * 1])

        # PRINT CONDITION
        if(constant.THRUSTER_MAPPER_CALCULATE_PROCESS):
            print("Force result robot frame : ".format( force ) )

        torque = numpy.matmul(self.direction_inverse.T, force.T)

        cmd = []
        for run in range(0, 8):
            temp = int(self.lookup_handle.find_pwm(torque[run]) )
            cmd.append( int( temp ) )

        self.header.stamp = rospy.get_rostime()
        pwm = tuple(cmd)

        # PRINT CONDITION
        if(constant.THRUSTER_MAPPER_RESULT ):
            if( self.count_print == constant.THRUSTER_MAPPER_COUNT  ):
                print("===================================================")
                print("{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(
                    force[0], force[1], force[2], force[3], force[4], force[5] ) )
                print("{:6d} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d}".format(
                    pwm[0], pwm[1], pwm[2], pwm[3], pwm[4], pwm[5], pwm[6], pwm[7]))
                self.count_print = 0
            self.count_print += 1

        try:
            self.client_throttle( self.header , pwm )
            self.thruster_message.header = self.header
            self.thruster_message.data = pwm
            self.pub_throttle.publish( self.thruster_message )
        except rospy.ServiceException , e :
            rospy.logfatal( "Failure to write pwm response from haredware")

    def callback_subscriber( self ,  message ):

        # PRINT_CONDTION
        if( constant.THRUSTER_MAPPER_CALLBACK_CALLED ):
            print("callback of subscriber have been called!")

        self.mission_lock.acquire( )

        self.mission_message = message
        if( constant.THRUSTER_MAPPER_AUTO_TIME ):
            self.mission_stamp = rospy.get_rostime()
        else:
            self.mission_stamp = message.header.stamp

        self.mission_lock.release( )

    def callback_subscribers( self ,  message ):

        # PRINT_CONDTION
        if( constant.THRUSTER_MAPPER_CALLBACK_CALLED ):
            print("callback of subscriber have been called!")

        self.control_lock.acquire( )

        self.control_message = message
        if( constant.THRUSTER_MAPPER_AUTO_TIME ):
            self.control_stamp = rospy.get_rostime()
        else:
            self.control_stamp = message.header.stamp

        self.control_lock.release( )

    def callback_service( self , request ):
        
        # PRINT_CONDTION
        if( constant.THRUSTER_MAPPER_CALLBACK_CALLED ):
            print("callback of service server have been called!")

        self.control_lock.acquire( )

        self.control_message = request.command
        if( constant.THRUSTER_MAPPER_AUTO_TIME ):
            self.control_stamp = rospy.get_rostime()
        else:
            self.control_stamp = request.command.header.stamp

        self.control_lock.release( )

        return SendControlCommandResponse()

    def spin( self ):
        print( "Open spin of thread id {:d}".format( thread.get_ident() ) )
        rospy.spin()
        print( "End spin of thread id is {:d}".format( thread.get_ident() ) )

if __name__ == '__main__':
    thruster_mapper = ThrusterMapper()
