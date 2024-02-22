# 介绍
此工程完成了四足机器人的足式里程计，通过IMU、编码器、足部气压计对四足机器人的位置、速度和姿态进行估计。该工程使用ROS进行开发，模型基于宇树的aliengo机器人。
里程计的输出在话题`/estimation_body_pose`  中
# 使用
在工程目录下，配置source环境
`soure ./devel/setup.bash`
通过roslaunch运行程序和rviz仿真环境
`roslaunch Odemetry legodemetry.launch`
 查看话题
`rostopic list`
查看话题数据，以估计的输出为例，它以nav_msgs/Odometry格式发布估计的位置、速度和姿态
`rostopic echo /estimation_body_pose `
录制rosbag，便于后续数据分析
`rosbag record -a -O bagname.bag` 
运行plotjuggler软件打开rosbag查看、分析各个数据值
`rosrun plotjuggler plotjuggler`
# 文件说明
文件夹LegOdemetry中包含的rosbag分别为
`aliengo_vrpn_joint_1.bag` `aliengo_vrpn_joint_2.bag` `aliengo_vrpn_joint_3.bag` 为给出的原始数据包
`bag1_final.bag` `bag2_final.bag` `bag3_final.bag` 包含了里程计最终的输出数据

