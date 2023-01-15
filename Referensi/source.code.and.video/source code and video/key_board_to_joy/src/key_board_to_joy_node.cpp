
#include "key_board_to_joy.hpp"



keyboard_to_joy::keyboard_to_joy()
{
    ros::param::get( "STANDUP_BUTTON", STANDUP_BUTTON );
    ros::param::get( "SITDOWN_BUTTON", SITDOWN_BUTTON );
    ros::param::get( "BODY_ROTATION_BUTTON", BODY_ROTATION_BUTTON );
    ros::param::get( "FORWARD_BACKWARD_AXES", FORWARD_BACKWARD_AXES );
    ros::param::get( "LEFT_RIGHT_AXES", LEFT_RIGHT_AXES );
    ros::param::get( "YAW_ROTATION_AXES", YAW_ROTATION_AXES );
    ros::param::get( "PITCH_ROTATION_AXES", PITCH_ROTATION_AXES );

    keyboard_sub = node.subscribe<keyboard::Key>("/keyboard/keydown",10,&keyboard_to_joy::KeyCallback, this);
    joy_pub = node.advertise<sensor_msgs::Joy>("/joy",100);
    locomde_pub = node.advertise<std_msgs::Int32>("/locomode",1);
    joy.buttons.assign(12,0);
    joy.axes.assign(4,0);
    locomode.data =2;

}

void keyboard_to_joy::KeyCallback(const keyboard::Key::ConstPtr& key)
{
    //std::vector<int> button;
   // button.assign(12,0);

    if (key->code ==49) // key 1 stand up
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.buttons[STANDUP_BUTTON] = 1;
        joy_pub.publish(joy);
        joy.buttons[STANDUP_BUTTON] = 0;
    }

    if (key->code ==50) // key 2 sit down
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.buttons[SITDOWN_BUTTON] = 1;

        joy_pub.publish(joy);
        joy.buttons[SITDOWN_BUTTON] = 0;
    }

    if (key->code ==51) // key 2 is_rotation
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.buttons[BODY_ROTATION_BUTTON] = !joy.buttons[BODY_ROTATION_BUTTON];

        joy_pub.publish(joy);
    }

    if (key->code ==119) // w, left up
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.axes[FORWARD_BACKWARD_AXES] = joy.axes[FORWARD_BACKWARD_AXES] + 0.01;
        locomde_pub.publish(locomode);
        joy_pub.publish(joy);
    }

    if (key->code ==120) // x, left down
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.axes[FORWARD_BACKWARD_AXES] = joy.axes[FORWARD_BACKWARD_AXES] - 0.01;

        joy_pub.publish(joy);
    }

    if (key->code == 97) // a, left left
    {
        joy.axes[LEFT_RIGHT_AXES] = joy.axes[LEFT_RIGHT_AXES] - 0.01;
        joy_pub.publish(joy);
    }

    if (key->code == 100) // d, left right
    {
        joy.axes[LEFT_RIGHT_AXES] = joy.axes[LEFT_RIGHT_AXES] + 0.01;
        joy_pub.publish(joy);
    }

    if (key->code == 115) // s, left center
    {
        joy.axes[LEFT_RIGHT_AXES] = 0;
        joy.axes[FORWARD_BACKWARD_AXES] = 0;
        joy_pub.publish(joy);
    }


    if (key->code ==105) // i, right up
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
        joy.axes[YAW_ROTATION_AXES] = joy.axes[YAW_ROTATION_AXES] + 0.01;

        joy_pub.publish(joy);
    }

    if (key->code == 44) // , , right down
    {
       // body_command.cmd = body_command.STAND_UP_CMD;
       // body_cmd_pub.publish(body_command);
         joy.axes[YAW_ROTATION_AXES] = joy.axes[YAW_ROTATION_AXES] - 0.01;

        joy_pub.publish(joy);
    }

    if (key->code == 106) // j, r left
    {
        joy.axes[PITCH_ROTATION_AXES] = joy.axes[PITCH_ROTATION_AXES] - 0.01;
        joy_pub.publish(joy);
    }

    if (key->code == 108) // l, r right
    {
        joy.axes[PITCH_ROTATION_AXES] = joy.axes[PITCH_ROTATION_AXES] + 0.01;
        joy_pub.publish(joy);
    }

    if (key->code == 107) // k, r center
    {
        joy.axes[YAW_ROTATION_AXES] = 0;
        joy.axes[PITCH_ROTATION_AXES] = 0;
        joy_pub.publish(joy);
    }


    //ros::Duration(25).sleep();

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_board_to_joy_node");
    keyboard_to_joy keyboard_to_joy;
    ros::spin();
    return 0;
}
