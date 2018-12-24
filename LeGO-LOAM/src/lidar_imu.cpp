#include "utility.h"

ros::Publisher pub_imu; 
ros::Publisher pub_lidar; 

void callbackHandle(const sensor_msgs::ImuConstPtr& imuIn, const sensor_msgs::PointCloud2ConstPtr& lidarIn){
    std::cout<<"test"<<std::endl;
    pub_imu.publish(*imuIn);
    pub_lidar.publish(*lidarIn);
}

void imuHandler(const sensor_msgs::PointCloud2ConstPtr& imuIn){

    pub_lidar.publish(*imuIn);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"lidar_imu_sub");
    ros::NodeHandle nh;

    pub_lidar= nh.advertise<sensor_msgs::PointCloud2> ("/lidar_points", 50);
    pub_imu= nh.advertise<sensor_msgs::Imu> ("/imu_raw_data", 5);

    // ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuHandler);
    // ros::Subscriber subLidar= nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 50, imuHandler);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh,"/imu/data",50);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh,"/velodyne_points",50);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::PointCloud2> MySyncPolicy;   

    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Imu, sensor_msgs::PointCloud2> MySyncPolicy;   
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callbackHandle, _1, _2));

    ros::spin();

    return 0;
    
}