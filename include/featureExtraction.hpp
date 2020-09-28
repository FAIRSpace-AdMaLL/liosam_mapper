#include "utility.h"
#include "lio_sam/cloud_info.h"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;
                
    boost::shared_ptr<lio_sam::cloud_info> pubLaserCloudInfoPtr;
    bool pubLaserCloudInfoFlag = false;

    FeatureExtraction();
 
    void initializationValue();

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn);

    void calculateSmoothness();

    void markOccludedPoints();

    void extractFeatures();
  
    void freeCloudInfoMemory();

    void publishFeatureCloud();
};
