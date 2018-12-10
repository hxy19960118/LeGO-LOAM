// JD test , XinYou Huo TianJin University
// add ukf prior information used to lidar point cloud register

#include "utility.h"

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subUKFprior; // sub UKF prior information (p,q)
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud; // seg + ground (sparase) pub
    ros::Publisher pubSegmentedCloudPure; // seg points pub
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn; // raw lidar data

    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud; // seg + ground (sparase) points
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure; // seg points
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint;

    cv::Mat rangeMat; // range image
    cv::Mat labelMat;
    cv::Mat groundMat;
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg;
    std_msgs::Header cloudHeader; // lidar header

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;
    float remove_range_points;
    int groundScanInd;
    float segmentTheta;
    string UKFPrior_topic;
    int N_SCAN;
    int Horizon_SCAN;


public:

    ImageProjection():
        nh("~"){

        nh.param<float>("remove_range",remove_range_points,1.0);
        nh.param<int>("groundscanind",groundScanInd,5);
        nh.param<float>("segtheta",segmentTheta,1.0742);
        nh.param<string>("ukfprior",UKFPrior_topic,"/aaaa");
        nh.param<int>("n_scan",N_SCAN,16);
        nh.param<int>("h_scan",Horizon_SCAN,2016);

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ImageProjection::cloudHandler, this);
        subUKFprior = nh.subscribe<sensor_msgs::PointCloud2>(UKFPrior_topic, 1, &ImageProjection::UKFpriorHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

}
