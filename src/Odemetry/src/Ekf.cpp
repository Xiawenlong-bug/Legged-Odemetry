#include "Ekf.h"

Ekf::Ekf()
{
    eye3.setIdentity();
    C.setZero();
    for (int i=0; i<NUM_LEG; ++i) {
        C.block<3,3>(i*3,0) = -eye3;  //-pos
        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
        C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
    }    

    // Q R are fixed
    //! 预测协方差矩阵Q 固定
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;               // position transition
    //y方向不太准
    Q(1,1)=4*PROCESS_NOISE_PIMU;
    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;               // velocity transition
    for (int i=0; i<NUM_LEG; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
    }
    R.setIdentity();
    for (int i=0; i<NUM_LEG; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                        // fk estimation
        R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;      // vel estimation
        R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    assume_flat_ground = true;  

}

void Ekf::init_state(CtrlState &state)
{
    filter_initialized = true;

    //先验估计协方差初始化
    P.setIdentity();
    P = P * 3;

    //状态先验估计 set initial value of x
    x.setZero();
/********************************************************************************/
    //质心位置初始化
    //x.segment<3>(0) = Eigen::Vector3d(6.705873, 1.531889, 0.417853);  //! 质心位置初始化，BAG1
    //x.segment<3>(0) = Eigen::Vector3d(5.716564, 2.709800, 0.441364);  //! 质心位置初始化，BAG2
    x.segment<3>(0) = Eigen::Vector3d(5.186118, 1.476573, 0.422339);  //! 质心位置初始化，BAG3  
/********************************************************************************/  

    for (int i = 0; i < NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);
        x.segment<3>(6 + i * 3) = state.root_rot_mat * fk_pos + x.segment<3>(0);    //! 阻断在世界坐标系下
    }

}

void Ekf::update_estimation(CtrlState &state,double dt)
{
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;
    //u = Ra + ag
    Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -9.81);
    if (state.movement_mode == 0) 
    {  //stand状态下，四个足端都是触地的
        for (int i = 0; i < NUM_LEG; ++i) 
            estimated_contacts[i] = 1.0;
    } 
    else 
    {  
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            estimated_contacts[i] = std::min(std::max( (state.foot_force(i)) / (100.0 - 0.0), 0.0), 1.0);
        }
    }

    //update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        // foot position transition
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3; 
        //摆动相加大噪声
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;            
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;    
        if (assume_flat_ground) 
        {
            //摆动项加大噪声
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;   
        }
    }
    xbar = A*x + B*u ;
    Pbar = A * P * A.transpose() + Q;
    //测量
    yhat = C*xbar;
    //! 实际测量过程 actual measurement
    for (int i=0; i<NUM_LEG; ++i) 
    {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3,1>(0,i);
        // 世界坐标系下足端的位置fk estimation
        y.block<3,1>(i*3,0) = state.root_rot_mat*fk_pos;  
        //state.root_rot_matrix是将机体坐标系转换到世界坐标系的矩阵
        Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3,1>(0,i) - skew(state.imu_ang_vel)*fk_pos;   
        y.block<3,1>(NUM_LEG*3+i*3,0) = (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*state.root_rot_mat*leg_v;    
        // height z estimation
        y(NUM_LEG*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;     
    }
    S = C * Pbar *C.transpose() + R;
    S = 0.5*(S+S.transpose());
    error_y = y - yhat; //! 测量误差 = 实际测量值 - 测量矩阵×先验状态
    //这一项是:（z-Hx）*（H×P×HT + R）卡尔曼增益的分母
    Serror_y = S.fullPivHouseholderQr().solve(error_y);
    //后验状态估计  
    x = xbar + Pbar * C.transpose() * Serror_y; 
    SC = S.fullPivHouseholderQr().solve(C);
    P = Pbar - Pbar * C.transpose() * SC * Pbar;
    P = 0.5 * (P + P.transpose());  
    //减少位置漂移 reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }
    //大于50N认为接触
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        if (estimated_contacts[i] < 0.5) 
        {
            state.estimated_contacts[i] = false;
        } 
        else 
        {
            state.estimated_contacts[i] = true;
        }
    }
    //质心位置与速度
    state.estimated_root_pos = x.segment<3>(0);     
    state.estimated_root_vel = x.segment<3>(3);                                                       
    state.root_pos = x.segment<3>(0);
    state.root_lin_vel = x.segment<3>(3);
}

//叉乘变矩阵
Eigen::Matrix3d Ekf::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}



Eigen::Vector3d Ekf::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}