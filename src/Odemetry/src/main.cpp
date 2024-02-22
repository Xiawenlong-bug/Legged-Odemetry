// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#include "state_estimate.h"
#include "Param.h"




int main(int argc,char*argv[])
{
    ros::init(argc,argv,"LegOdemetry");
    ros::NodeHandle nh;
    std::unique_ptr<Estimate> aliengo =std::make_unique<Estimate>(nh);
    std::thread main_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  
        ros::Duration dt(0);    

        while (ros::ok())
        {
            ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;    
            bool main_update_running = aliengo->main_update(elapsed.toSec(), dt.toSec());
            if(!main_update_running)
            {
                std::cout<<"error"<<std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    ros::AsyncSpinner spinner(6);
    spinner.start();
    main_thread.join();
    return 0;
}

