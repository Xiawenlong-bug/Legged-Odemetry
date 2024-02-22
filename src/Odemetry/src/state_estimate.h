#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H


#include <eigen3/Eigen/Dense>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include "tf2_ros/static_transform_broadcaster.h"

#include "Param.h"
#include "CtrlState.h"
#include "Ekf.h"
#include "Kinematics.h"
#include "filter.hpp"


class Estimate
{
public:
    Estimate(ros::NodeHandle &_nh);
    
    bool main_update(double t, double dt);

   // callback functions
    void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);


    void joint_callback(const sensor_msgs::JointState::ConstPtr & msg_p);

    void rotation_callback(const geometry_msgs::PoseStamped::ConstPtr & tf_data);
private:

    ros::NodeHandle nh;
    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf

    ros::Subscriber sub_joint_msg;    

    // 0, 1, 2, 3: FL, FR, RL, RR

    ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
//    ros::Subscriber sub_joy_msg;
    
    ros::Subscriber sub_tf_msg;
    // debug estimation 
    ros::Publisher pub_estimated_pose;
    ros::Publisher joint_pub;

    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    Kinematics kin;
    CtrlState ctrl_state;
    Ekf estimate;

};



#endif