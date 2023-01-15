#ifndef KEY_BOARD_TO_JOY_H_
#define KEY_BOARD_TO_JOY_H_

#include "keyboard/Key.h"
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>


class keyboard_to_joy {
        public: keyboard_to_joy();
                ros::NodeHandle node;
                sensor_msgs::Joy joy;
                ros::Subscriber keyboard_sub;
                ros::Publisher joy_pub;
                ros::Publisher locomde_pub;
	private:

                int STANDUP_BUTTON, SITDOWN_BUTTON, BODY_ROTATION_BUTTON, FORWARD_BACKWARD_AXES, LEFT_RIGHT_AXES, YAW_ROTATION_AXES, PITCH_ROTATION_AXES;
		void KeyCallback(const keyboard::Key::ConstPtr& key);
                std_msgs::Int32 locomode;
};


#endif // KEY_BOARD_TO_JOY_H_
