#ifndef HEXAPOD_CRANE_PUMP_H_
#define HEXAPOD_CRANE_PUMP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class HexapodCranePump
{
    public:
        HexapodCranePump( void );
        std_msgs::Bool crane_;
        std_msgs::Bool pump_;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber joy_sub_;
        void joyCallback( const sensor_msgs::Joy::ConstPtr &joy );
};

#endif // HEXAPOD_CRANE_PUMP_H_
