#include "utility.h"
#include "imuPreintegration.hpp"
#include "featureExtraction.hpp"
#include "imageProjection.hpp"
#include "mapOptmization.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1); Done


// subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay()); Done
// subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000); Done


// subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1); Done


// subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay()); Done
// pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1); Done


// subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay()); Done
// pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1); Done

// subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay()); Done

// subscribe<sensor_msgs::Imu>(imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay()); Done
// subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay()); Done

// subscribe<nav_msgs::Odometry>(gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());

// subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam_offline");

    TransformFusion TF;
    IMUPreintegration ImuP;
    ImageProjection IP;
    FeatureExtraction FE;
    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Offline LIO_SAM started...\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::Rate rate(200);
    rosbag::Bag read_bag;
    read_bag.open(MO.readBag, rosbag::bagmode::Read);
    
    ROS_INFO("listening to:");
    ROS_INFO(MO.imuTopic.c_str());
    ROS_INFO(MO.pointCloudTopic.c_str());
    ROS_INFO(MO.gpsTopic.c_str());

    std::vector<std::string> topics;
    topics.push_back(MO.imuTopic);
    topics.push_back(MO.pointCloudTopic);
    // topics.push_back(MO.gpsTopic);
 
    rosbag::View view(read_bag, rosbag::TopicQuery(topics)); //note:TopicQuery;TypeQuery
	
    boost::shared_ptr<sensor_msgs::Imu> IMUptr;
    boost::shared_ptr<sensor_msgs::PointCloud2> RawCloudptr;
 
    if (ros::ok())
    {
        ROS_INFO("OK!!!!");

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            ros::spinOnce();
            ROS_INFO("Looping...");

            if(MO.pubLaserOdometryGlobalFlag)
            {
                TF.lidarOdometryHandler(MO.pubLaserOdometryGlobalPtr);
                MO.pubLaserOdometryGlobalFlag = false;
            }

            if(MO.pubLaserOdometryIncrementalFlag)
            {
                ImuP.odometryHandler(MO.pubLaserOdometryIncrementalPtr);
                MO.pubLaserOdometryIncrementalFlag = false;
            }

            if(ImuP.pubImuOdometryFlag)
            {
                TF.imuOdometryHandler(ImuP.pubImuOdometryPtr);
                IP.odometryHandler(ImuP.pubImuOdometryPtr);
                ImuP.pubImuOdometryFlag = false;
            }

            if(IP.pubLaserCloudInfoFlag)
            {
                FE.laserCloudInfoHandler(IP.pubLaserCloudInfoPtr);
                IP.pubLaserCloudInfoFlag = false;
            }

            if(FE.pubLaserCloudInfoFlag)
            {
                MO.laserCloudInfoHandler(FE.pubLaserCloudInfoPtr);
                FE.pubLaserCloudInfoFlag = false;
            }

            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg)
            {
                ROS_INFO("Pub an IMU msg");
                ImuP.imuHandler(imu_msg);
                IP.imuHandler(imu_msg);
            }

            sensor_msgs::PointCloud2::ConstPtr pointcloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pointcloud_msg)
            {
                ROS_INFO("Pub a PointCloud");
                IP.cloudHandler(pointcloud_msg);
            }

            // nav_msgs::Odometry::ConstPtr gps_msg = m.instantiate<nav_msgs::Odometry>();
            // if (gps_msg)
            // {
            //     ROS_INFO("Pub a GPS msg");
            //     MO.gpsHandler(gps_msg);
            // }
            rate.sleep();
        }

        ROS_INFO("Reached the end of the bag.");
    }
    
    read_bag.close();

    rosbag::Bag write_bag;
    write_bag.open(MO.writeBag, rosbag::bagmode::Write);

    ROS_INFO("Saving global map and path to %s", MO.writeBag.c_str());

    write_bag.write("/lio_sam/mapping/map_global", ros::Time::now(), MO.globalMapToSave);
    write_bag.write("/lio_sam/mapping/path", ros::Time::now(), MO.globalPath);
    
    ROS_INFO("Saved!");

    write_bag.close();

    loopthread.join();
    visualizeMapThread.join();

    ros::shutdown();

    return 0;
}