#ifndef EKF_H
#define EKF_H

#include "Kinematics.h"
#include "CtrlState.h"
#include "Param.h"
#include <iostream>
#include <vector>

// state estimator parameters
#define STATE_SIZE 18   // 状态变量维度
#define MEAS_SIZE 28    // 观测变量维度
#define PROCESS_NOISE_PIMU 0.35    // 位置预测协方差
#define PROCESS_NOISE_VIMU 0.06     // 速度预测协方差
#define PROCESS_NOISE_PFOOT 0.01    // foot位置预测协方差
#define SENSOR_NOISE_PIMU_REL_FOOT 0.01    // 足端位置测量协方差
#define SENSOR_NOISE_VIMU_REL_FOOT 0.005      // 足端速度测量协方差
#define SENSOR_NOISE_ZFOOT 0.1  // 足端高度测量协方差

class Ekf
{

public:
    Ekf();
    
    void init_state(CtrlState & state);

    void update_estimation(CtrlState & state,double dt);

    bool is_inited(){return filter_initialized;};

    static Eigen::Matrix3d skew(Eigen::Vector3d vec) ;//3阶向量，叉乘变矩阵
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);

    

private:
    bool filter_initialized = false;
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR 
    Eigen::Matrix<double, STATE_SIZE, 1> x; // 先验估计 estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // 后验估计 estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // 先验估计协方差 estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // 后验估计协方差 estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // 状态转移矩阵 estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // 输入矩阵
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // 预测协防差 estimation state transition noise
    // observation
    // 0 1 2   FL pos residual  世界坐标系下，质心位置和足端位置之差
    // 3 4 5   FR pos residual
    // 6 7 8   RL pos residual
    // 9 10 11 RR pos residual
    // 12 13 14 vel residual from FL    
    // 15 16 17 vel residual from FR
    // 18 19 20 vel residual from RL
    // 21 22 23 vel residual from RR
    // 24 25 26 27 foot height  足端的高度 
    Eigen::Matrix<double, MEAS_SIZE, 1> y; // 实际测量 observation
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // 测量矩阵×状态先验估计 estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // y-yhat estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; //
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // 测量矩阵 estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; 
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // 测量协方差矩阵 estimation state observation noise    
    // helper matrices 
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; 
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // 卡尔曼增益 kalman gain

    bool assume_flat_ground = true;

    // variables to process foot force
    double estimated_contacts[4];
    
};


#endif