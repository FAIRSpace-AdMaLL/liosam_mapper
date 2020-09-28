#include "utility.h"
#include "imuPreintegration.hpp"
#include "featureExtraction.hpp"
#include "imageProjection.hpp"
#include "mapOptimization.hpp"

// subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1); Done


// subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay()); Done
// subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// advertise<nav_msgs::Odometry>(odomTopic+"_incremental", 2000); Done


// subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); Done
// advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1); Done


// subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay()); Done
// advertise<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1); Done


// subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay()); Done
// advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1); Done

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
    rosbag::Bag bag;
    bag.open("/home/jaguar/longshaw3_2020-09-16-16-21-07.bag", rosbag::bagmode::Read);
 
    std::vector<std::string> topics;
    topics.push_back(MO.imuTopic);
    topics.push_backMO(MO.pointCloudTopic);
    topics.push_back(MO.gpsTopic);
 
    rosbag::View view(bag, rosbag::TopicQuery(topics)); //note:TopicQuery;TypeQuery
	
    boost::shared_ptr<sensor_msgs::Imu> IMUptr;
    boost::shared_ptr<sensor_msgs::PointCloud2> RawCloudptr;
 
    if (ros::ok())
    {
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            ros::spinOnce();
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
                TF.imuOdometryHandler(pubImuOdometryPtr);
                IP.odometryHandler(pubImuOdometryPtr);
                ImuP.pubImuOdometryFlag = false;
            }

            if(IP.pubLaserCloudInfoFlag)
            {
                FE.laserCloudInfoHandler(IP.pubLaserCloudInfoPtr);
                IP.pubLaserCloudInfoFlag = false;
            }
            
            if(FE.pubLaserCloudInfoFlag)
            {
                MO.laserCloudInfoHandler(IP.pubLaserCloudInfoPtr);
                IP.pubLaserCloudInfoFlag = false;
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
            
            nav_msgs::Odometry::ConstPtr gps_msg = m.instantiate<nav_msgs::Odometry>();
            if (gps_msg)
            {
                ROS_INFO("Pub a GPS msg");
                MO.gpsHandler(gps_msg);
            }
            
            rate.sleep();
        }
    }
    
    bag.close();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}