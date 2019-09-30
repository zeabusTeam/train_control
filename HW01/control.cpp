// FILE			: control.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 23 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <std_msgs/Float64.h>

#include    <train_control/SingleState.h>

const static double Epsilon = 1e-7;

template< class type_object >
class template_object
{
    public:
        template_object( type_object* pointer )
        {
            this->pointer = pointer;
        }
    
        void callback( const type_object& message )
        {
            *( this->pointer ) = message;
        }

    protected:
        type_object* pointer;
}; // template_object

double s; int range_out_cal = 0, range_pre_acceleration = 0, range_s = 0;

double calculate(double current_point , double current_velocity , double current_acceleration , double target_point) {
            double v,out;
            // v^2 = u^2 + 2as
            s = abs(target_point - current_point);
            v = sqrt(pow(current_velocity,2) + 2*current_acceleration*s);
            out = v - current_velocity;
        return out;
        }

void condition_out_cal(double value) {
    if(value < 1) {
        range_out_cal = 0;
    }
    else if(value < 10) {
        range_out_cal = 1;
    }
    else if(value < 15) {
        range_out_cal = 2;
    }
    else if(value < 20) {
        range_out_cal = 3;
    }
}

void condition_pre_acceleration(double value) {
    if(value < 1) {
        range_pre_acceleration = 0;
    }
    else if(value < 10) {
        range_pre_acceleration = 1;
    }
    else if(value < 15) {
        range_pre_acceleration = 2;
    }
    else if(value < 20) {
        range_pre_acceleration = 3;
    }
}

void condition_s(double value) {
    if(value < 1) {
        range_s = 0;
    }
    else if(value < 10) {
        range_s = 1;
    }
    else if(value < 15) {
        range_s = 2;
    }
    else if(value < 20) {
        range_s = 3;
    }
}

double value_force(double out_cal,double pre_acceleration,int y,int z,int x) {
    // axis x = s, axis y = out_cal
    double value_force;
    int value_range;
    double range_value[5] = {0,3,5,10,20};
    int range_force[3][3][3] = {
                            {{-1,0,1},{0,1,2},{1,2,2}},
                            {{-2,-1,0},{-1,0,1},{0,1,1}},
                            {{-3,-2,-1},{-2,-1,0},{-1,0,0}}
                            };
    value_range = range_force[z][y][x];
    value_force = copysign(range_value[abs(value_range)],value_range);
    return value_force;
}



double pre_acceleration = 0;


double force( double current_point , 
        double current_velocity , 
        double current_acceleration , 
        double target_point ,
        double diff_time )
{
    double answer = 0;
    std::cout   << "\n\n====================== Calculate ====================================\n";
    std::cout.precision(5);
    std::cout.width(10);
    target_point = 10;
    std::cout   << "(Point , Velocity , Acceleration , Target ) : " << current_point << " , "
                << current_velocity << " , " << current_acceleration << " , "
                << target_point << "\n";
//====================================== EDIT HERE ==========================================
    double out_cal, force;
    out_cal = calculate(current_point,current_velocity,current_acceleration,target_point);
    condition_out_cal(out_cal);
    condition_pre_acceleration(pre_acceleration);
    condition_s(s);
    answer = value_force(out_cal,current_acceleration,range_out_cal,range_pre_acceleration,range_s);
//===========================================================================================
    std::cout   << "Command force " << answer << "\n";
    return answer;
} // force

int main( int argv , char** argc )
{
    ros::init( argv , argc , "control" );
    
    ros::NodeHandle node_handle("");

    std_msgs::Float64 force_output;
    force_output.data = 0;
    ros::Publisher pub_state = node_handle.advertise< std_msgs::Float64 >("/force" , 1 );

    train_control::SingleState current_state;
    template_object<train_control::SingleState> current_object( &current_state );
    ros::Subscriber sub_current = node_handle.subscribe( "/state" , 1 ,
            &template_object< train_control::SingleState >::callback , &current_object );

    std_msgs::Float64 target_point;
    target_point.data = 0 ;
    template_object<std_msgs::Float64> target_object( &target_point );
    ros::Subscriber sub_target = node_handle.subscribe( "/target" , 1 ,
            &template_object< std_msgs::Float64 >::callback , &target_object );

    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Rate rate( 10 );
    while( ros::ok() )
    {
        rate.sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
        force_output.data = force( current_state.point , current_state.velocity , 
                current_state.acceleration , target_point.data , 
                (current_time - previous_time).toSec() );
        pub_state.publish( force_output );
        previous_time = current_time;     
    }
    

} // main
