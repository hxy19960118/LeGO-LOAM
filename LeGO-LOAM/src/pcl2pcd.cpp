#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  
#include "utility.h"
  
void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  
  pcl::PointCloud<pcl::PointXYZI> cloud;  
  // std::cout<< "success" << std::endl;
  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息  
  pcl::io::savePCDFileASCII ("/home/nuc/pcd/test.pcd", cloud);//保存pcd  
}  
int main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcl_write");  
  ros::NodeHandle nh;  
  ros::Subscriber bat_sub = nh.subscribe("/velodyne_ns/velodyne_points", 10, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
} 