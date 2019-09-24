// FILE			: plan_1_dimension.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _CALCULATE_      : This will show about information and calculate

// README
//  This file for node plan model in 1 dimension use to test control
//  This will calculate of equation of Newton Law is F - F(friction) = ma and it ideal!

// REFERENCE
//  ref01   : http://www.cplusplus.com/reference/cstdlib/abs/

// MACRO SET
#define _CALCULATE_

// MACRO CONDITION

#include    <cmath>

#include    <iostream>

#include    <ros/ros.h>

#include    <std_msgs/Float64.h>

#include    <train_control/SingleState.h>

const static double Epsilon = 1e-7;

template< class message_type >
class subscribe_object
{
    public:

        subscribe_object( message_type* message_pointer )
        {
            this->message_pointer = message_pointer;
        }

        void callback( const message_type& message )
        {
            *(this->message_pointer) = message;
        }

    protected:
        message_type* message_pointer;

}; // declare class

void print_state( train_control::SingleState state , std::string message )
{
    std::cout   << message << " :\nS\t" << state.point
                << "\nV\t" << state.velocity
                << "\nA\t" << state.acceleration << "\n";
}

int main( int argv , char** argc )
{

    ros::init( argv , argc , "model_plan");
    
    ros::NodeHandle node_handle("");
    ros::Publisher pub_state = node_handle.advertise< train_control::SingleState >("/state" ,1);

    std_msgs::Float64 force;
    force.data = 0;
    subscribe_object< std_msgs::Float64 > sub_force( &force );
    ros::Subscriber ros_sub = node_handle.subscribe( "/force" , 1 , 
            &subscribe_object< std_msgs::Float64 >::callback , &sub_force );

    ros::Rate rate( 10 );

    train_control::SingleState model_state;
    model_state.header.stamp = ros::Time::now();
    model_state.point = 0;
    model_state.velocity = 0;
    model_state.acceleration = 0;
    train_control::SingleState previous_model_state = model_state;

    double sum_force = 0;

    int type = 0; // 0 is static , 1 is kinetic

    double friction = 0;
    static const double coefficient_static = 0.1;
    static const double coefficient_kinetic = 0.065;
    static const double mass = 10;
    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = previous_time;
    
    while( ros::ok() )
    {

        double diff_time = (current_time - previous_time).toSec();

#ifdef _CALCULATE_
        std::cout   << "\n\n============ " << diff_time << " ======================\n";
        print_state( previous_model_state , "Input calculate" );
        std::cout   << "Force input " << force.data << "\n";
#endif // _CALCULATE_
        // Next step calculate acceleration
        if( fabs( previous_model_state.velocity ) > Epsilon )
        { 
            friction = coefficient_kinetic * mass;
#ifdef _CALCULATE_
            std::cout   << "Case kinetic friction : " << friction << '\n';
#endif
            if( previous_model_state.velocity > 0 )
            {
                friction *= -1;
            }
            sum_force = force.data + friction;
            model_state.acceleration = sum_force/mass;

            model_state.velocity = ( previous_model_state.velocity +
                    (previous_model_state.acceleration*diff_time ) );

            if( fabs( force.data ) < Epsilon )
            {
                if( model_state.velocity * previous_model_state.velocity < 0 )
                {
                    model_state.velocity = 0;
                    model_state.acceleration = 0;
                }
            }
        } // end type object have velocity <kinetic>
        else
        {
            friction = coefficient_static * mass;
#ifdef _CALCULATE_
            std::cout   << "Case static friction : " << friction << '\n';
#endif
            if( force.data > 0 )
            {
                sum_force = force.data - friction;
#ifdef _CALCULATE_
                std::cout   << "sum_force is " << sum_force << "\n";
#endif
                if( sum_force < 0 )
                {
                    model_state.acceleration = 0;
                }
                else
                {
                    model_state.acceleration = sum_force/mass;
                }
            }
            else
            {
                sum_force = force.data + friction;
#ifdef _CALCULATE_
                std::cout   << "sum_force is " << sum_force << "\n";
#endif
                if( sum_force > 0 )
                {
                    model_state.acceleration = 0;
                }
                else
                {
                    model_state.acceleration = sum_force/mass;
                }
            }
            model_state.velocity = ( previous_model_state.velocity +
                    (previous_model_state.acceleration*diff_time ) );
        } // end type object don't have velocity <static>

        model_state.point += ( previous_model_state.velocity + model_state.velocity ) / 2 *
                diff_time;

#ifdef _CALCULATE_
        print_state( model_state , "Output calculate" );
#endif // _CALCULATE_

        pub_state.publish( model_state );

        force.data = 0;
        previous_time = current_time;
        rate.sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
        previous_model_state = model_state;
    }

}
