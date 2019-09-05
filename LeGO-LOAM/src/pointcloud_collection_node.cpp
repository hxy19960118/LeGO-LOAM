#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <deque>
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>

tf::TransformListener * tf_listener_;
std::deque<pcl::PointXYZ> _PointBuf;
ros::Publisher _cloud_out_pub;

tf::StampedTransform last_tf;

void vlp_cb(const sensor_msgs::PointCloud2::ConstPtr& _msg) 
{
    ros::Time time1 = ros::Time::now();
    if (!tf_listener_-> waitForTransform(
        "world",
        "vio_test/velodyne",
        _msg->header.stamp,
        ros::Duration(0.5f) ) ) {
            ROS_INFO_THROTTLE(1, "[pcl collection]: cannot get tf");
            return;
        }
    sensor_msgs::PointCloud2 _scan_cloud_in_world;
    tf::StampedTransform temp_tf;
    tf_listener_->lookupTransform("world", "vio_test/velodyne", _msg->header.stamp, temp_tf);
    pcl_ros::transformPointCloud("world", *_msg ,_scan_cloud_in_world, *tf_listener_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(_scan_cloud_in_world, *_pcl_cloud_xyz);

    double dwx = std::abs(temp_tf.getRotation().x() - last_tf.getRotation().x());
    double dwy = std::abs(temp_tf.getRotation().y() - last_tf.getRotation().y());
    double dwz = std::abs(temp_tf.getRotation().z() - last_tf.getRotation().z());

    // std::cout << "dq : " << dwx <<" " <<  dwy << " " << dwz << std::endl;

    last_tf = temp_tf;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_xyz2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_xyz3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter1;
    filter1.setLeafSize(0.3f, 0.3f, 0.3f);
    filter1.setInputCloud(_pcl_cloud_xyz);
    filter1.filter(*_pcl_cloud_xyz2);


    double _min_x, _max_x, _min_y, _max_y;
    _min_x = temp_tf.getOrigin().x() - 45.0f;
    _max_x = temp_tf.getOrigin().x() + 45.0f;
    _min_y = temp_tf.getOrigin().y() - 45.0f;
    _max_y = temp_tf.getOrigin().y() + 45.0f;


    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(_pcl_cloud_xyz2);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(-1.0f, 4.0f);
    pass1.filter(*_pcl_cloud_xyz2);

    pass1.setInputCloud(_pcl_cloud_xyz2);
    pass1.setFilterFieldName("x");
    pass1.setFilterLimits(_min_x, _max_x);
    pass1.filter(*_pcl_cloud_xyz2);

    pass1.setInputCloud(_pcl_cloud_xyz2);
    pass1.setFilterFieldName("y");
    pass1.setFilterLimits(_min_y, _max_y);
    pass1.filter(*_pcl_cloud_xyz2);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
    sor.setInputCloud (_pcl_cloud_xyz2);                           //设置待滤波的点云
    sor.setMeanK (25);                               //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1);                      //设置判断是否为离群点的阀值
    sor.filter (*_pcl_cloud_xyz3);     


    // if (dwx < 0.015 && dwy < 0.015 && dwz < 0.015 ) {
        // pcl::PointCloud<pcl::PointXYZHSV>::Ptr _pcl_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
        for (int _i = 0; _i < _pcl_cloud_xyz3->points.size(); _i++ ) {
            // _pcl_cloud->points[_i].h = _msg->header.stamp.toSec();
            pcl::PointXYZ _tmp_p;
            _tmp_p.x = _pcl_cloud_xyz3->points[_i].x;
            _tmp_p.y = _pcl_cloud_xyz3->points[_i].y;
            _tmp_p.z = _pcl_cloud_xyz3->points[_i].z;
            // _tmp_p.h = (float)_msg->header.stamp.toSec();
            _PointBuf.push_back(_tmp_p);
            while (_PointBuf.size() > 50000) {
                _PointBuf.pop_front();
            }
        }
    // } else {
        // ROS_WARN("[pointcloud collection]: high rotation drop LiDar data");
    // }



    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_all(new pcl::PointCloud<pcl::PointXYZ>);

    for (int _i = 0; _i < _PointBuf.size(); _i++) {
        _pcl_cloud_all->points.push_back(_PointBuf[_i]);
    }


    // std::cout << "dt0 : " << (ros::Time::now() - time1).toSec() <<std::endl;

    // *_pcl_cloud_all += *_pcl_cloud;


    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(_pcl_cloud_all);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-1.0f, 4.0f);
    // pass.filter(*_pcl_cloud_all);

    // pass.setInputCloud(_pcl_cloud_all);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(_min_x, _max_x);
    // pass.filter(*_pcl_cloud_all);

    // pass.setInputCloud(_pcl_cloud_all);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(_min_y, _max_y);
    // pass.filter(*_pcl_cloud_all);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_all_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    // std::cout << "total pcl all size: " << _pcl_cloud_all->points.size() << std::endl;

    // for (int _i = 0; _i < _pcl_cloud_all->points.size(); _i++ ) {
    //     pcl::PointXYZ _tmp_p;
    //     _tmp_p.x = _pcl_cloud_all->points[_i].x;
    //     _tmp_p.y = _pcl_cloud_all->points[_i].y;
    //     _tmp_p.z = _pcl_cloud_all->points[_i].z;
    //     _pcl_cloud_all_xyz->points.push_back(_tmp_p);
    // }

    // std::cout << "dt1 : " << (ros::Time::now() - time1).toSec() <<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.4f, 0.4f, 0.4f);
    filter.setInputCloud(_pcl_cloud_all);
    filter.filter(*_pcl_cloud_filtered);
    
    // _pcl_cloud_all_xyz = _pcl_cloud_filtered;



    // std::cout << "dt2 : " << (ros::Time::now() - time1).toSec() <<std::endl;
    sensor_msgs::PointCloud2 _cloud_in_world;
    pcl::toROSMsg(*_pcl_cloud_filtered, _cloud_in_world);

    _cloud_in_world.header = _msg->header;
    _cloud_in_world.header.frame_id = "world";

    sensor_msgs::PointCloud2 _cloud_in_scan;
    pcl_ros::transformPointCloud("vio_test/velodyne", _cloud_in_world, _cloud_in_scan, *tf_listener_);
    _cloud_in_scan.header.frame_id = "vio_test/velodyne";
    _cloud_out_pub.publish(_cloud_in_scan);
    // ros::Time time2 = ros::Time::now();
    // std::cout << "process dt : " << (ros::Time::now() - time1).toSec() <<std::endl;


    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_collection_node");
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener();
    // _pcl_cloud_all.reset(new pcl::PointCloud<pcl::PointXYZHSV>());

    ros::Subscriber vlp_sub;
    vlp_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_ns/velodyne_points", 2, vlp_cb);
    _cloud_out_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_ns/velodyne_points_filtered", 2);

    ros::spin();
    return 0;
}