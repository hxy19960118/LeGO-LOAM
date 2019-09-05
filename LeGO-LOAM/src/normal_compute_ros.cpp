
#include "utility.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


class NormalComputeRos{

private:
    ros::NodeHandle nh;

    ros::Subscriber subPointCloud;

    ros::Publisher pubClusterCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn; // raw lidar data
    pcl::PointCloud<PointType>::Ptr fullCloud; // raw lidar data
    pcl::PointCloud<PointType>::Ptr fullCloudDS; // raw lidar data
    pcl::PointCloud<PointType>::Ptr tempCloud; // raw lidar data
    pcl::PointCloud<PointType>::Ptr groundCloud;

    pcl::PointCloud<PointType>::Ptr ungroundCloud;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;

    pcl::VoxelGrid<PointType> downSizeFilterfull;

    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::Normal> normalEstimation;

    pcl::IndicesPtr indices;

    std_msgs::Header cloudHeader;

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZI, pcl::Normal> ne;

    cv::Mat rangeMat; // range image
    cv::Mat labelMat;
    cv::Mat groundMat;
    int labelcount;

    PointType nanPoint;

    float startOrientation;
    float endOrientation;

    std::vector<int> Curvature;
    std::vector<std::pair<int8_t, int8_t> > neighborIterator;
    int N_SCAN;
    int Horizon_SCAN;
    int groundScanInd;
    float remove_range_points;

public:
    NormalComputeRos():
        nh("~"){
        nh.param<int>("groundscanind",groundScanInd,8);
        nh.param<int>("n_scan",N_SCAN,16);
        nh.param<int>("h_scan",Horizon_SCAN,1800);
        nh.param<float>("remove_range",remove_range_points,0.0);

        // subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_ns/velodyne_points", 1, &NormalComputeRos::ProcessCloud, this);
        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/vio_test/velodyne", 1, &NormalComputeRos::ProcessCloud, this);
        pubClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud",1);


        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        reset();

        // std::cout<< "size: " << neighborIterator.size() << std::endl;
        // pcl::io::loadPCDFile<pcl::PointXYZI>("/home/nuc/pcd/test.pcd", *laserCloudIn); 

        // 可视化
        // visualNormal();
    }

    void ProcessCloud(const sensor_msgs::PointCloud2ConstPtr& cloudinMsg){

        cloudHeader = cloudinMsg->header;
        pcl::fromROSMsg(*cloudinMsg, *laserCloudIn);
        
        ros::Time time1 = ros::Time::now();
        projectPointCloud();
        groundRemoval();

        std::cout << "fullCloudDS_size: " << fullCloudDS->points.size() <<std::endl;
        calculateNormal1();
        regionGrowingSegmentation();
        publishCloud();
        reset();

        ros::Time time2 = ros::Time::now();
        std::cout<< "time_last: " << (time2 - time1).toSec() << std::endl;
    }

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullCloudDS.reset(new pcl::PointCloud<PointType>());
        tempCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        groundCloud.reset(new pcl::PointCloud<PointType>());
        ungroundCloud.reset(new pcl::PointCloud<PointType>());
        // segmentedCloud.reset(new pcl::PointCloud<PointType>());
        // segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        // outlierCloud.reset(new pcl::PointCloud<PointType>());
        tree.reset(new pcl::search::KdTree<pcl::PointXYZI>());

        normals.reset(new pcl::PointCloud<pcl::Normal>());
        indices.reset(new std::vector <int>);
        colored_cloud.reset(new pcl::PointCloud <pcl::PointXYZRGB>());

        std::pair<int8_t, int8_t> neighbor;

        for (int8_t i = -3; i<=3;++i){
            for(int8_t j = -3; j <= 3;++j){
                neighbor.first = -i; neighbor.second = -j; neighborIterator.push_back(neighbor);
            }
        }
        downSizeFilterfull.setLeafSize(0.1, 0.1, 0.1);
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

        // neighbor.first = -1; neighbor.second = -1; neighborIterator.push_back(neighbor);
        // neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        // neighbor.first = -1; neighbor.second =  1; neighborIterator.push_back(neighbor);

        // neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        // neighbor.first =  0; neighbor.second =  0; neighborIterator.push_back(neighbor);
        // neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);

        // neighbor.first =  1; neighbor.second = -1; neighborIterator.push_back(neighbor);
        // neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        // neighbor.first =  1; neighbor.second =  1; neighborIterator.push_back(neighbor);
    }

    void reset(){

        laserCloudIn->clear();
        groundCloud->clear();
        ungroundCloud->clear();
        fullCloudDS->clear();
        colored_cloud->clear();
        normals->clear();
        clusters.clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);

    }

    // void visualNormal(){

    //     colored_cloud = reg.getColoredCloud();
    //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cluster"));
        
    //     viewer->setBackgroundColor(0, 0, 0);
    //     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
    //     viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "cluster_cloud");

    //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_cloud" );
    //     viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 3, 0.15, "normals");

    //     viewer-> addCoordinateSystem(1.0);
    //     viewer-> initCameraParameters();

    //     // // int v1(0);
    //     // // viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1);
    //     // // viewer->setBackgroundColor(5,55, 10, v1);

    //     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(fullCloudDS, 0, 225, 0);
    //     // // viewer->addCoordinateSystem(0.0, "Normals" );


    //     // viewer->addPointCloud<pcl::PointXYZI>(fullCloudDS, single_color, "cloud" );
    //     // viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cluster_cloud");

    //     // // viewer->addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(fullCloudDS, normals, 1, 0.75, "normals");
    //     // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud", 4);
    //     // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cluster_cloud", 4);
    //     while (!viewer->wasStopped())
    //     {
    //         viewer->spinOnce(100);
    //         boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //     }

    //     // pcl::visualization::CloudViewer viewer("cluster viewer");
    //     // viewer.showCloud(colored_cloud);
    //     // while(!viewer.wasStopped())
    //     // {

    //     // }

    // }

    void regionGrowingSegmentation(){

        // pcl::PassThrough<pcl::PointXYZI> pass;
        // pass.setInputCloud (fullCloudDS);
        // pass.setFilterFieldName ("z");
        // pass.setFilterLimits (0.0, 1.0);
        // pass.filter (*indices);

        reg.setMinClusterSize (30);
        reg.setMaxClusterSize (5000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (fullCloudDS);
        // reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (0.1);
        reg.extract (clusters);

        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        for (int i = 0 ; i < clusters.size();++i){
            std::cout << "number " << i << " of the seg: " << clusters[i].indices.size() << std::endl;
        }
        // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
        // std::cout << "These are the indices of the points of the initial" <<
        // std::endl << "cloud that belong to the first cluster:" << std::endl;
        // int counter = 0;
        // while (counter < clusters[0].indices.size ())
        // {
        //     std::cout << clusters[0].indices[counter] << ", ";
        //     counter++;
        //     if (counter % 10 == 0)
        //         std::cout << std::endl;
        // }
        std::cout << std::endl;

        // colored_cloud = reg.getColoredCloud();
    }

    void calculateNormal1(){
        tree->setInputCloud(fullCloudDS);

        normalEstimation.setInputCloud(fullCloudDS);
        normalEstimation.setSearchMethod(tree);
        normalEstimation.setKSearch(20);
        normalEstimation.compute(*normals);


    }

    void publishCloud(){

        // std::cout <<"test1"<<std::endl;

        colored_cloud = reg.getColoredCloud();
        sensor_msgs::PointCloud2 cloudMsgtemp;
        pcl::toROSMsg(*colored_cloud, cloudMsgtemp);
        cloudMsgtemp.header.stamp = cloudHeader.stamp;
        cloudMsgtemp.header.frame_id = cloudHeader.frame_id;
        // std::cout <<"test2"<<std::endl;
        pubClusterCloud.publish(cloudMsgtemp);
        
    }

    // void calculateNormal(){

    //     // normalEstimation.setInputCloud(fullCloud);
    //     size_t _size = fullCloud->points.size();
    //     int Index;
    //     int thisIndX,thisIndY,fromIndX,fromIndY;
    //     PointType thisPoint;
    //     pcl::Normal this_normal;

    //     // for(size_t i = 1900; i < 2000; ++i){
    //     for(size_t i = 0; i < _size; ++i){
    //         tempCloud->clear();
    //         thisPoint = fullCloud->points[i];
    //         fromIndX = int(thisPoint.intensity);
    //         fromIndY = (thisPoint.intensity - fromIndX)*10000.0;

    //         // std::cout << "row and col: [" << fromIndX << "," << fromIndY << "]"<< std::endl;

    //         std::vector<int> _indices;
    //         Eigen::Vector4f _normal;
    //         float _curva;

    //         for(auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
    //             thisIndX = fromIndX + (*iter).first;
    //             thisIndY = fromIndY + (*iter).second;

    //             if (fromIndX == 0 && fromIndY == 0 && i!=0)
    //                 continue;
    //             if (thisIndX < 0 || thisIndX >= 16)
    //                 continue;

    //             if (thisIndY < 0)
    //                 thisIndY = 1800 - 1;
    //             if (thisIndY >= 1800)
    //                 thisIndY = 0;

    //             Index = thisIndY + thisIndX * 1800;
    //             // std::cout << "index: "<< Index << "," << (*iter).first << "," << (*iter).second << std::endl;
    //             _indices.push_back(Index);

    //             tempCloud->push_back(fullCloud->points[Index]);
    //         }

    //         normalEstimation.computePointNormal(*fullCloud, _indices, _normal, _curva);
    //         pcl::flipNormalTowardsViewpoint(thisPoint, 0.0, 0.0, 0.0, _normal);
    //         // normalEstimation.setViewPoint(0.0,0.0,0.5);
    //         this_normal.normal_x = _normal(0);
    //         this_normal.normal_y = _normal(1);
    //         this_normal.normal_z = _normal(2);
    //         this_normal.curvature = _curva;
    //         normals->push_back(this_normal);
            
    //     }
    // }

    void projectPointCloud(){
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;
        PointType thisPoint1;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y; // 0 - 15 row
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

// +y:1350 ; +x : 900 ; -y : 450 ; -x:0(1799)
            if (horizonAngle <= -90)
                columnIdn = -int(horizonAngle / ang_res_x) - 450; 
                // columnIdn = -int(horizonAngle / ang_res_x) - 504; 
            else if (horizonAngle >= 0)
                columnIdn = -int(horizonAngle / ang_res_x) + 1350;
                // columnIdn = -int(horizonAngle / ang_res_x) + 1512;
            else
                columnIdn = 1350 - int(horizonAngle / ang_res_x);
                // columnIdn = 1512 - int(horizonAngle / ang_res_x);

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            rangeMat.at<float>(rowIdn, columnIdn) = range; // range image

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;

            // fullInfoCloud->points[index].intensity = range;
        }
        // downSizeFilterfull.setInputCloud(fullCloud);
        // downSizeFilterfull.filter(*fullCloudDS);
    }

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN; // i,j
                upperInd = j + (i+1)*Horizon_SCAN; // i+1 , j

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                    
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX || rangeMat.at<float>(i,j) <= remove_range_points){
                    labelMat.at<int>(i,j) = -1;
                }
                if (groundMat.at<int8_t>(i,j) == 1) {
                    groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                } else{
                    if(labelMat.at<int>(i,j) == 0 )
                    ungroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
        downSizeFilterfull.setInputCloud(ungroundCloud);
        downSizeFilterfull.filter(*fullCloudDS);
        fullCloudDS->width = fullCloudDS->points.size();
        fullCloudDS->height = 1;

    }



    ~NormalComputeRos(){}

};

int main(int argc, char** argv){
    ros::init(argc, argv, "normal_compute");

    NormalComputeRos NC;

    ROS_INFO("\033[1;32m---->\033[0m Normal Computation Started.");

    ros::spin();
    return 0;
}
