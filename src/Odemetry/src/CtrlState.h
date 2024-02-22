#ifndef CTRLSTATE_H
#define CTRLSTATE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "Param.h"
#include "sensor_msgs/JointState.h"
class CtrlState
{
public:
    CtrlState()
    {
        reset();
    }
    void reset()
    {
        movement_mode=0;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler_orin.setZero();
        root_euler.setZero();
        root_rot_mat.setZero();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();

        foot_force.setZero();

        joint_pos.setZero();
        joint_vel.setZero();


        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();
        foot_pos_abs_mpc.setZero();
        foot_pos_rel_last_time.setZero();
        foot_pos_target_last_time.setZero();
        foot_pos_cur.setZero();
        foot_pos_recent_contact.setZero();
        foot_vel_world.setZero();
        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        j_foot.setIdentity();

        for(int i=0;i<NUM_LEG;i++)
        {
            contacts[i]=false;
        }

        joint_state.name.resize(12);
        joint_state.name={
        "FR_hip_joint","FR_thigh_joint","FR_calf_joint",
        "FL_hip_joint","FL_thigh_joint","FL_calf_joint",
        "RR_hip_joint","RR_thigh_joint","RR_calf_joint",
        "RL_hip_joint","RL_thigh_joint","RL_calf_joint"};
        joint_state.position.resize(12);
        joint_state.velocity.resize(12);
    }

    void resetFromROSParam(ros::NodeHandle &_nh)
    {
        
    }
    //variables
    //运动状态
    int movement_mode;  // 0: standstill, 1: start to locomote


    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Quaterniond root_quat_test;
    Eigen::Vector3d root_euler_orin;
    Eigen::Vector3d root_test;
    Eigen::Vector3d root_euler;
    Eigen::Vector3d root_euler_test;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

    bool contacts[NUM_LEG];         // flag to decide leg in the stance/swing mode

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;

    // state estimation
    bool estimated_contacts[NUM_LEG];  // true if the estimator thinks the foot has contact
    Eigen::Vector3d estimated_root_pos;
    Eigen::Vector3d estimated_root_vel;
    Eigen::Quaterniond estimate_root_pose;

    sensor_msgs::JointState joint_state;
};



#endif