#include "state_estimate.h"

Estimate::Estimate(ros::NodeHandle &_nh)
{
    nh=_nh;

   // debug estimation
    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/estimation_body_pose", 100);
    joint_pub=nh.advertise<sensor_msgs::JointState>("/bag_joint_states",1);

    sub_joint_msg=nh.subscribe("/aliengo/joint_states",100,&Estimate::joint_callback,this);
    sub_imu_msg=nh.subscribe("/aliengo/imu",100,&Estimate::imu_callback,this);
    sub_gt_pose_msg=nh.subscribe("/estimation_body_pose",100,&Estimate::gt_pose_callback,this);
    sub_tf_msg=nh.subscribe("/vrpn_client_node/aliendog/pose",100,&Estimate::rotation_callback,this);


    ctrl_state.reset();
    // init leg kinematics
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.2407;
    leg_offset_x[1] = 0.2407;
    leg_offset_x[2] = -0.2407;
    leg_offset_x[3] = -0.2407;
    leg_offset_y[0] = 0.051;
    leg_offset_y[1] = -0.051;
    leg_offset_y[2] = 0.051;
    leg_offset_y[3] = -0.051;
    motor_offset[0] = 0.0868;
    motor_offset[1] = -0.0868;
    motor_offset[2] = 0.0868;
    motor_offset[3] = -0.0868;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = UPPER_LEG_LENGTH;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++)
    {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }    

}


bool Estimate::main_update(double t,double dt)
{
    //movement_mode 0:四足站立;1:走动
    //如果四条全在20N以上，则认为在站立
    bool standflag=(ctrl_state.foot_force[0]>20)&&(ctrl_state.foot_force[1]>20)&&(ctrl_state.foot_force[2]>20)&&(ctrl_state.foot_force[3]>20);
    
    if(standflag) ctrl_state.movement_mode=0;
    else ctrl_state.movement_mode=1;

    if (!estimate.is_inited()) {
        estimate.init_state(ctrl_state);
    } else {
        estimate.update_estimation(ctrl_state, dt);
    }    



    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = ctrl_state.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = ctrl_state.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = ctrl_state.estimated_root_pos(2);

    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = ctrl_state.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = ctrl_state.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = ctrl_state.estimated_root_vel(2);

    estimate_odom.pose.pose.orientation.x=ctrl_state.estimate_root_pose.x();
    estimate_odom.pose.pose.orientation.y=ctrl_state.estimate_root_pose.y();
    estimate_odom.pose.pose.orientation.z=ctrl_state.estimate_root_pose.z();
    estimate_odom.pose.pose.orientation.w=ctrl_state.estimate_root_pose.w();


    pub_estimated_pose.publish(estimate_odom);
    joint_pub.publish(ctrl_state.joint_state);

    return true;
}


// callback functions
void Estimate::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {

    // calculate several useful variables
    // euler should be roll pitch yaw
    //ctrl_state.root_rot_mat = ctrl_state.root_quat.toRotationMatrix();
    ctrl_state.root_euler_orin = Ekf::quat_to_euler(ctrl_state.root_quat);



/***********************************************************/
    //Z-Y-X,即RPY
    //手动补偿
    //bag1
    // ctrl_state.root_euler[0]=ctrl_state.root_euler_orin[0]-2.0/57.3;
    // ctrl_state.root_euler[1]=ctrl_state.root_euler_orin[1]+0.6/57.3;
    // ctrl_state.root_euler[2]=ctrl_state.root_euler_orin[2]+206.4/57.3;
    //bag2
    // ctrl_state.root_euler[0]=ctrl_state.root_euler_orin[0]+(0.2-2.0)/57.3;
    // ctrl_state.root_euler[1]=ctrl_state.root_euler_orin[1]+(0.6+0.16)/57.3;
    // ctrl_state.root_euler[2]=ctrl_state.root_euler_orin[2]+(206.4+10.6)/57.3;
    //bag3
    ctrl_state.root_euler[0]=ctrl_state.root_euler_orin[0]-1.5/57.3;
    ctrl_state.root_euler[1]=ctrl_state.root_euler_orin[1]+0.6/57.3;
    ctrl_state.root_euler[2]=ctrl_state.root_euler_orin[2]+226/57.3;
/*******************************************************************/



    for (int i = 0; i < 3; i++)
    {
        if (ctrl_state.root_euler[i]>3.141)
        {
            ctrl_state.root_euler[i]=ctrl_state.root_euler[i]-2*3.141;
        }
        if(ctrl_state.root_euler[i]<-3.141)
        {
            ctrl_state.root_euler[i]=ctrl_state.root_euler[i]+2*3.141;
        }
    }

    //这里转换不太对
    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(ctrl_state.root_euler[0], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(ctrl_state.root_euler[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(ctrl_state.root_euler[2], Eigen::Vector3d::UnitX());
    //调整一下
    double qz=quaternion3.x();
    double qy=quaternion3.y();
    double qx=quaternion3.z();
    double qw=quaternion3.w();

    Eigen::Quaterniond quaternion4(qw,qx,qy,qz);
    //直接将imu作为姿态估计值
    ctrl_state.estimate_root_pose=quaternion4;

    ctrl_state.root_rot_mat=quaternion4.toRotationMatrix();

     double yaw_angle = ctrl_state.root_euler[2];

    ctrl_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
    for (int i = 0; i < NUM_LEG; ++i) {
        ctrl_state.foot_pos_rel.block<3, 1>(0, i) = kin.fk(
                ctrl_state.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        ctrl_state.j_foot.block<3, 3>(3 * i, 3 * i) = kin.jac(
                ctrl_state.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Eigen::Matrix3d tmp_mtx = ctrl_state.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = ctrl_state.joint_vel.segment<3>(3 * i);
        ctrl_state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        ctrl_state.foot_pos_abs.block<3, 1>(0, i) = ctrl_state.root_rot_mat * ctrl_state.foot_pos_rel.block<3, 1>(0, i);
        ctrl_state.foot_vel_abs.block<3, 1>(0, i) = ctrl_state.root_rot_mat * ctrl_state.foot_vel_rel.block<3, 1>(0, i);

        ctrl_state.foot_pos_world.block<3, 1>(0, i) = ctrl_state.foot_pos_abs.block<3, 1>(0, i) + ctrl_state.root_pos;
        ctrl_state.foot_vel_world.block<3, 1>(0, i) = ctrl_state.foot_vel_abs.block<3, 1>(0, i) + ctrl_state.root_lin_vel;
    }


}

void Estimate::imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    ctrl_state.root_quat = Eigen::Quaterniond(imu->orientation.w,
                                                  imu->orientation.x,
                                                  imu->orientation.y,
                                                  imu->orientation.z);


    ctrl_state.imu_acc = Eigen::Vector3d(
            imu->linear_acceleration.x,
            imu->linear_acceleration.y,
            imu->linear_acceleration.z
    );
    ctrl_state.imu_ang_vel = Eigen::Vector3d(
            imu->angular_velocity.x,
            imu->angular_velocity.y,
            imu->angular_velocity.z
    );

    ctrl_state.root_ang_vel = ctrl_state.root_rot_mat * ctrl_state.imu_ang_vel;

}



void Estimate::joint_callback(const sensor_msgs::JointState::ConstPtr & msg_p)
{


        //FL_hip
        ctrl_state.joint_pos[0]=msg_p->position[3];
        ctrl_state.joint_vel[0]=msg_p->velocity[3];        
        //FL_thigh
        ctrl_state.joint_pos[1]=msg_p->position[4];
        ctrl_state.joint_vel[1]=msg_p->velocity[4];        
        //FL_calf
        ctrl_state.joint_pos[2]=msg_p->position[5];
        ctrl_state.joint_vel[2]=msg_p->velocity[5];
        //FR_hip
        ctrl_state.joint_pos[3]=msg_p->position[0];
        ctrl_state.joint_vel[3]=msg_p->velocity[0];
        //FR_thigh
        ctrl_state.joint_pos[4]=msg_p->position[1];
        ctrl_state.joint_vel[4]=msg_p->velocity[1];
        //FR_calf
        ctrl_state.joint_pos[5]=msg_p->position[2];
        ctrl_state.joint_vel[5]=msg_p->velocity[2];
        //RL_hip
        ctrl_state.joint_pos[6]=msg_p->position[9];
        ctrl_state.joint_vel[6]=msg_p->velocity[9];
        //RL_thigh
        ctrl_state.joint_pos[7]=msg_p->position[10];
        ctrl_state.joint_vel[7]=msg_p->velocity[10];
        //RL_calf
        ctrl_state.joint_pos[8]=msg_p->position[11];
        ctrl_state.joint_vel[8]=msg_p->velocity[11];
        //RR_hip
        ctrl_state.joint_pos[9]=msg_p->position[6];
        ctrl_state.joint_vel[9]=msg_p->velocity[6];
        //RR_thigh
        ctrl_state.joint_pos[10]=msg_p->position[7];
        ctrl_state.joint_vel[10]=msg_p->velocity[7];
        //RR_calf
        ctrl_state.joint_pos[11]=msg_p->position[8];
        ctrl_state.joint_vel[11]=msg_p->velocity[8];

    

        //FL_foot
        ctrl_state.foot_force[0]=msg_p->effort[13];
        //FR_foot
        ctrl_state.foot_force[1]=msg_p->effort[12];
        //RL_foot
        ctrl_state.foot_force[2]=msg_p->effort[15];
        //RR_foot
        ctrl_state.foot_force[3]=msg_p->effort[14];
        

        ctrl_state.joint_state.position=msg_p->position;
        ctrl_state.joint_state.velocity=msg_p->velocity;
}


void Estimate::rotation_callback(const geometry_msgs::PoseStamped::ConstPtr & tf_data)
{
   
    ctrl_state.root_quat_test = Eigen::Quaterniond(tf_data->pose.orientation.w,
                                                tf_data->pose.orientation.x,
                                                tf_data->pose.orientation.y,
                                                tf_data->pose.orientation.z);
    
}