#!/usr/bin/env

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

//==============================================================================
// Constructor
//==============================================================================

HexapodCranePump::HexapodCranePump( void )
{
    crane_.data = false;
    pump_.data = false;
    #joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 5, &HexapodCranePump::joyCallback, this);
    rospy.Subscriber("joy", Joy, joyCallback)
}

def joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    if( joy->buttons[9] == 1 )
    {
        pump_.data = true;
    }
    else if ( joy->buttons[9] == 0 )
    {
        pump_.data = false;
    }

    if( joy->buttons[10] == 1 )
    {
        crane_.data = true;
    }
    else if(joy->buttons[10] == 0)
    {
        crane_.data = false;
    }
}

if __name__ == '__main__':

    int craneOpen = 0;
    int pumpOn = 0;
    
    #ros::init(argc, argv, "hexapod_crane_pump");
    rospy.init_node("hexapod_crane_pump", anonymous = True)
    HexapodCranePump hexapodCranePump;

    ros::AsyncSpinner spinner( 1 ); // Using 1 threads
    spinner.start();
    ros::Rate loop_rate( 10 ); // 10 hz
    while( ros::ok() )
    {
        if( (hexapodCranePump.crane_.data == true) && (craneOpen == 0))
        {
            #ROS_INFO("Hexapod crane is now open.");
            rospy.loginfo("Hexapod crane is now open.")
            craneOpen = 1;
        }
        else if( (hexapodCranePump.crane_.data == false) && (craneOpen == 1) )
        {
            #ROS_INFO("Hexapod crane is now close.");
            rospy.loginfo("Hexapod crane is now close.")
            craneOpen = 0;
        } 

        if( (hexapodCranePump.pump_.data == true) && (pumpOn == 0) )
        {
            #ROS_INFO("Hexapod pump is now on.");
            rospy.loginfo("Hexapod pump is now on.")
            pumpOn = 1;
        } else if ( (hexapodCranePump.pump_.data == false) && (pumpOn == 1) )
        {
            #ROS_INFO("Hexapod pump is now off.");
            rospy.loginfo("Hexapod pump is now off.")
            pumpOn = 0;
        }

        loop_rate.sleep();
    }
}
