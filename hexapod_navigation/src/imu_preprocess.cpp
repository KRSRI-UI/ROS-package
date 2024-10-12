#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include <exception>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::string src_topic;
std::string dst_topic;
std::vector<double> o_cov;
std::vector<double> av_cov;
std::vector<double> la_cov;

ros::Subscriber sub_from;
ros::Publisher pub_to;


std::vector<double> GetListParam( const ros::NodeHandle& n, 
                                  const std::string& param_path )
{
    std::vector<double> ret;
    if (n.getParam(param_path, ret))
    {
        ROS_INFO_STREAM("Got param from " << param_path);
    } 
    else 
    {
        ROS_ERROR_STREAM("Failed to get param " << param_path );
        throw std::logic_error("failed to get param from " + param_path);
    }
    return ret;
}

std::string GetStringParam( const ros::NodeHandle& n, 
                            const std::string& param_path )
{
    std::string ret;
    if (n.getParam(param_path, ret))
    {
        ROS_INFO_STREAM("Got param from " << param_path);
    } 
    else 
    {
        ROS_ERROR_STREAM("Failed to get param " << param_path );
        throw std::logic_error("failed to get param from " + param_path);
    }
    return ret;
}

void ProcessIMUCallback( const sensor_msgs::Imu::ConstPtr& msg )
{
    sensor_msgs::Imu msg_with_cov = *msg;
    
    msg_with_cov.header.frame_id = "base_link";

    // not using orientation for now, but axis need to be switched.
    tf::Quaternion q(
                        msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w
                    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // transfrom quaternion axis, quaternion still in NED format
    tf2::Quaternion new_quat;
    new_quat.setRPY( (0 - pitch ), roll, yaw );
    msg_with_cov.orientation = tf2::toMsg(new_quat);

    // switch x and y axis for linear accel
    msg_with_cov.angular_velocity.y = msg->angular_velocity.x;
    msg_with_cov.angular_velocity.x = (0.0 - msg->angular_velocity.y);

    // switch x and y axis for linear accel
    msg_with_cov.linear_acceleration.y = msg->linear_acceleration.x;
    msg_with_cov.linear_acceleration.x = (0.0 - msg->linear_acceleration.y);

    // adding orientation covariance
    memcpy( &msg_with_cov.orientation_covariance[0],
            &o_cov[0], 
            o_cov.size() * sizeof(double));

    // adding angular_velocity covariance
    memcpy( &msg_with_cov.angular_velocity_covariance[0],
            &av_cov[0], 
            av_cov.size() * sizeof(double));

    // adding linear_acceleration covariance
    memcpy( &msg_with_cov.linear_acceleration_covariance[0],
            &la_cov[0], 
            la_cov.size() * sizeof(double));

    // republishing imu data with covariance
    pub_to.publish(msg_with_cov);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smi_cov_adder");

    ros::NodeHandle n;
    std::string ns = ros::this_node::getName();
    std::string src_topic_path = "/from";
    std::string dst_topic_path = "/to";
    std::string o_cov_path = "/covariance/orientation";
    std::string av_cov_path = "/covariance/angular_velocity";
    std::string la_cov_path = "/covariance/linear_acceleration";

    std::string src_topic_param =  ns + src_topic_path;
    std::string dst_topic_param =  ns + dst_topic_path;
    std::string o_cov_param =  ns + o_cov_path;
    std::string av_cov_param = ns + av_cov_path;
    std::string la_cov_param = ns + la_cov_path;

    try
    {
        src_topic = GetStringParam( n, src_topic_param);
        dst_topic = GetStringParam( n, dst_topic_param);
        o_cov = GetListParam( n, o_cov_param);
        av_cov = GetListParam( n, av_cov_param);
        la_cov = GetListParam( n, la_cov_param);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to get necessary param");
        return -1;
    }

    ROS_INFO_STREAM("Subscribing to " << src_topic);
    ROS_INFO_STREAM("Publishing to " << dst_topic);
    pub_to   = n.advertise<sensor_msgs::Imu>(dst_topic, 1000);
    sub_from = n.subscribe(src_topic, 1000, ProcessIMUCallback);
    
    ros::spin();

    return 0;
}