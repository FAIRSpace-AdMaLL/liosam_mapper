#include "featureExtraction.hpp"
#include "imageProjection.hpp"
#include "imuPreintegration.hpp"
#include "mapOptmization.hpp"
#include "utility.h"

#include <ros/service.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

#include <algorithm>
#include <chrono>
#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lio_sam_offline");
  ros::NodeHandle nh;

  TransformFusion TF;
  IMUPreintegration ImuP;
  ImageProjection IP;
  FeatureExtraction FE;
  mapOptimization MO;

  ROS_INFO("\033[1;32m----> Offline LIO_SAM started...\033[0m");

  std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
  std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

  ros::Rate rate(MO.loopRate);
  rosbag::Bag read_bag;
  read_bag.open(MO.readBag, rosbag::bagmode::Read);

  // Get sensor metadata
  ouster_ros::OSConfigSrv cfg{};
  auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("/os_node/os_config");
  client.waitForExistence();
  if (!client.call(cfg))
  {
    ROS_ERROR("Calling config service failed");
    return EXIT_FAILURE;
  }

  auto info = ouster::sensor::parse_metadata(cfg.response.metadata);
  uint32_t H = info.format.pixels_per_column;
  uint32_t W = info.format.columns_per_frame;

  auto pf = ouster::sensor::get_format(info);

  auto xyz_lut = ouster::make_xyz_lut(info);

  ouster_ros::Cloud cloud{ W, H };
  ouster::LidarScan ls{ W, H };

  ouster::ScanBatcher batch(W, pf);

  ROS_INFO("listening to:");
  ROS_INFO(MO.imuTopic.c_str());
  ROS_INFO(MO.pointCloudTopic.c_str());
  ROS_INFO(MO.gpsTopic.c_str());

  std::vector<std::string> topics;
  topics.push_back(MO.imuTopic);
  topics.push_back(MO.pointCloudTopic);
  // topics.push_back(MO.gpsTopic);

  rosbag::View view(read_bag, rosbag::TopicQuery(topics));  // note:TopicQuery;TypeQuery

  boost::shared_ptr<sensor_msgs::Imu> IMUptr;
  boost::shared_ptr<sensor_msgs::PointCloud2> RawCloudptr;

  if (ros::ok())
  {
    ROS_INFO("OK!!!!");

    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
      ros::spinOnce();
      // ROS_INFO("Looping...");

      if (MO.pubLaserOdometryGlobalFlag)
      {
        TF.lidarOdometryHandler(MO.pubLaserOdometryGlobalPtr);
        MO.pubLaserOdometryGlobalFlag = false;
      }

      if (MO.pubLaserOdometryIncrementalFlag)
      {
        ImuP.odometryHandler(MO.pubLaserOdometryIncrementalPtr);
        MO.pubLaserOdometryIncrementalFlag = false;
      }

      if (ImuP.pubImuOdometryFlag)
      {
        TF.imuOdometryHandler(ImuP.pubImuOdometryPtr);
        IP.odometryHandler(ImuP.pubImuOdometryPtr);
        ImuP.pubImuOdometryFlag = false;
      }

      if (IP.pubLaserCloudInfoFlag)
      {
        FE.laserCloudInfoHandler(IP.pubLaserCloudInfoPtr);
        IP.pubLaserCloudInfoFlag = false;
      }

      if (FE.pubLaserCloudInfoFlag)
      {
        MO.laserCloudInfoHandler(FE.pubLaserCloudInfoPtr);
        FE.pubLaserCloudInfoFlag = false;
      }

      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg)
      {
        // ROS_INFO("Pub an IMU msg");
        ImuP.imuHandler(imu_msg);
        IP.imuHandler(imu_msg);
      }

      ouster_ros::PacketMsg::ConstPtr packet_msg = m.instantiate<ouster_ros::PacketMsg>();
      sensor_msgs::PointCloud2 pointcloud_msg;
      if (packet_msg)
      {
        // ROS_INFO("Pub a PointCloud");
        if (batch(packet_msg->buf.data(), ls))
        {
          auto h = std::find_if(ls.headers.begin(), ls.headers.end(),
                                [](const auto& h) { return h.timestamp != std::chrono::nanoseconds{ 0 }; });
          if (h != ls.headers.end())
          {
            scan_to_cloud(xyz_lut, h->timestamp, ls, cloud);

            pointcloud_msg = ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp, "/os_sensor");

            IP.cloudHandler(boost::make_shared<sensor_msgs::PointCloud2 const>(pointcloud_msg));
          }
        }
      }

      rate.sleep();
    }

    ROS_INFO("Reached the end of the bag.");
  }

  read_bag.close();

  // Save data
  MO.saveMapAndTrajectory();

  loopthread.detach();
  visualizeMapThread.detach();

  ros::shutdown();

  return 0;
}