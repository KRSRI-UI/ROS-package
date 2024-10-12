#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class MyClass {
  public:
    int NUMBER_OF_LEGS = 6;        // Number of legs
    int NUMBER_OF_LEG_JOINTS = 3;  // Number of leg segments

    // Topics we are publishing
    std::array<ros::Publisher, 18> joint_command_pub_;  // constant size allocation 18

    // Topics we are subscribing
    ros::Subscriber joint_state_sub_;
    void joint_stateCallback(const sensor_msgs::JointStateConstPtr &joint_state_msg );

  private:
    std_msgs::Float64 msg;
};

void MyClass::joint_stateCallback(const sensor_msgs::JointStateConstPtr &joint_state_msg ) {
  int i = 0;
  for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
  {
      msg.data = joint_state_msg->position[i];
      joint_command_pub_[i].publish(msg);
      i++;
      msg.data = joint_state_msg->position[i];
      joint_command_pub_[i].publish(msg);
      i++;
      msg.data = joint_state_msg->position[i];
      joint_command_pub_[i].publish(msg);
      i++;
      if ( NUMBER_OF_LEG_JOINTS == 4 )
      {
          msg.data = joint_state_msg->position[i];
          joint_command_pub_[i].publish(msg);
          i++;
      }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "joint_command_publisher");
  
  // Create class objects
  MyClass myObj;

  ros::param::get( "NUMBER_OF_LEGS", myObj.NUMBER_OF_LEGS );
  ros::param::get( "NUMBER_OF_LEG_SEGMENTS", myObj.NUMBER_OF_LEG_JOINTS );

  XmlRpc::XmlRpcValue SERVOS;
  std::vector<std::string> servo_map_key_;
  std::vector<std::string> servo_names_;

  // Find out how many servos/joints we have
  for( XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++ )
  {
    servo_map_key_.push_back( it->first );
  }

  servo_names_.resize( servo_map_key_.size() );

  for( std::size_t i = 0; i < servo_map_key_.size(); i++ )
  {
    ros::param::get( ("/SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/name"), servo_names_[i] );
  }

  // Node Handle
  ros::NodeHandle nh_;

  // Topics we are publishing
  if (ros::param::has("/reignblaze/coxa_joint_LF_position_controller/joint"))
  {
    myObj.joint_command_pub_[0] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[1] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[2] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[3] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[4] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[5] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[6] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[7] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[8] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[9] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[10] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[11] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[12] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[13] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[14] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[15] = nh_.advertise<std_msgs::Float64>( "/reignblaze/coxa_joint_LF_position_controller/command" , 10 );
    myObj.joint_command_pub_[16] = nh_.advertise<std_msgs::Float64>( "/reignblaze/femur_joint_LF_position_controller/command" , 10 );
    myObj.joint_command_pub_[17] = nh_.advertise<std_msgs::Float64>( "/reignblaze/tibia_joint_LF_position_controller/command" , 10 );
  } else {
    myObj.joint_command_pub_[0] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[1] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[2] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_RR_position_controller/command" , 10 );
    myObj.joint_command_pub_[3] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[4] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[5] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_RM_position_controller/command" , 10 );
    myObj.joint_command_pub_[6] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[7] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[8] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_RF_position_controller/command" , 10 );
    myObj.joint_command_pub_[9] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[10] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[11] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_LR_position_controller/command" , 10 );
    myObj.joint_command_pub_[12] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[13] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[14] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_LM_position_controller/command" , 10 );
    myObj.joint_command_pub_[15] = nh_.advertise<std_msgs::Float64>( "/phantomX/coxa_joint_LF_position_controller/command" , 10 );
    myObj.joint_command_pub_[16] = nh_.advertise<std_msgs::Float64>( "/phantomX/femur_joint_LF_position_controller/command" , 10 );
    myObj.joint_command_pub_[17] = nh_.advertise<std_msgs::Float64>( "/phantomX/tibia_joint_LF_position_controller/command" , 10 );
  }

//  for( int j = 0; j < servo_map_key_.size() ; j++ )
//  { 
//    myObj.joint_command_pub_[j] = nh_.advertise<std_msgs::Float64>( ("/phantomX/" + servo_names_[j] + "_position_controller/command") , 10 );
//  }

  // Topics we are subscribing
  myObj.joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>( "/joint_states", 10, &MyClass::joint_stateCallback, &myObj);

  ros::spin();
  return 0;
}
