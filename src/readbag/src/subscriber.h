#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/QuadWord.h"
#include "tf2_ros/transform_listener.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"


class Listener
{
public:
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

private:
    ros::NodeHandle nh;

    ros::Subscriber imu_data;

};




#endif