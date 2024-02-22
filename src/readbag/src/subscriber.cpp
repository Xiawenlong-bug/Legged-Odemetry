#include "subscriber.h"


Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat) {
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

void imu_callback(const sensor_msgs::Imu::ConstPtr & msg_p)
{
    Eigen::Quaterniond  root_quat = Eigen::Quaterniond(msg_p->orientation.w,
                                    msg_p->orientation.x,
                                    msg_p->orientation.y,
                                    msg_p->orientation.z);
    Eigen::Vector3d root_euler=quat_to_euler(root_quat);
    ROS_INFO("IMU...\nX:%f\nY:%f\nZ:%f\n",root_euler[0]*57.3,root_euler[1]*57.3,root_euler[2]*57.3);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg_p)
{
    Eigen::Quaterniond  root_quat = Eigen::Quaterniond(msg_p->pose.orientation.w,
                                    msg_p->pose.orientation.x,
                                    msg_p->pose.orientation.y,
                                    msg_p->pose.orientation.z);
    Eigen::Vector3d root_euler=quat_to_euler(root_quat);
    ROS_INFO("TRUE...\nX:%f\nY:%f\nZ:%f\n",root_euler[0]*57.3,root_euler[1]*57.3,root_euler[2]*57.3);
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"bag_subscribe");
    ros::NodeHandle nh;

    //实例化订阅者对象
    ros::Subscriber imu_data=nh.subscribe("/aliengo/imu",100,imu_callback);

    ros::Subscriber joint_data=nh.subscribe("/vrpn_client_node/aliendog/pose",100,pose_callback);

    



    ros::spin();//循环读取接收的数据，并调用回调函数处理



    return 0;
}


