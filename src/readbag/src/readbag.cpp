#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/QuadWord.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc,char* argv[])
{
    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;
    ros::Publisher joint_pub =nh.advertise<sensor_msgs::JointState>("/bag_joint_states",1);
    sensor_msgs::JointState joint_state;
    geometry_msgs::PoseStamped dogpose;
    //sensor_msgs::JointState joint_state=boost::make_shared<sensor_msgs::JointState>();
    joint_state.name.resize(12);
    joint_state.name={
        "FR_hip_joint","FR_thigh_joint","FR_calf_joint",
        "FL_hip_joint","FL_thigh_joint","FL_calf_joint",
        "RR_hip_joint","RR_thigh_joint","RR_calf_joint",
        "RL_hip_joint","RL_thigh_joint","RL_calf_joint"};
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    rosbag::Bag bag;
    bag.open("/home/xwl/LegOdemetry/aliengo_vrpn_joint_3.bag",rosbag::bagmode::Read);

    std::vector<std::string> topics;
    //topics.push_back(std::string("/aliengo/imu"));
    //topics.push_back(std::string("/aliengo/joint_states"));
    //topics.push_back(std::string("/tf"));
    topics.push_back(std::string("/vrpn_client_node/aliendog/pose"));
    ros::Rate r(1000);
    for(auto &&m:rosbag::View(bag,rosbag::TopicQuery(topics)))
    {
        std::string topic =m.getTopic();
        //sensor_msgs::Imu::ConstPtr imu_data=m.instantiate<sensor_msgs::Imu>();
        //sensor_msgs::JointState::ConstPtr joint_data =m.instantiate<sensor_msgs::JointState>();
        //tf2_msgs::TFMessage::ConstPtr tf_data=m.instantiate<tf2_msgs::TFMessage>();
        geometry_msgs::PoseStamped::ConstPtr pose_data=m.instantiate<geometry_msgs::PoseStamped>();



        // if(topic == "/aliengo/joint_states")
        // {
        //     joint_state.header.stamp=ros::Time::now();
        //     joint_state.position=joint_data->position;
        //     joint_state.velocity=joint_data->velocity;
        //     ROS_INFO("%f\n%f\n%f\n%f\n",joint_data->effort[12],joint_data->effort[13],joint_data->effort[14],joint_data->effort[15]);
        //     joint_pub.publish(joint_state);
        // }

        if(topic == "/vrpn_client_node/aliendog/pose")
        {
            dogpose.pose.position=pose_data->pose.position;
            ROS_INFO("%f\n%f\n%f\n",pose_data->pose.position.x,pose_data->pose.position.y,pose_data->pose.position.z);
        }


        r.sleep();

    }

    bag.close();
    return 0;
}