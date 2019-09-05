// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
// #include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


class SegAndCluster{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;
    ros::Publisher pubGroundSAC;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud; // seg + ground (sparase) pub
    ros::Publisher pubSegmentedCloudPure; // seg points pub
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;
    ros::Publisher pubClusterCloud;

    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::Normal> normalEstimation;
    pcl::IndicesPtr indices;

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
    // pcl::IntegralImageNormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::VoxelGrid<PointType> downSizeFilterSegPure;

    pcl::SACSegmentation<PointType> seg;

    pcl::PointCloud<PointType>::Ptr laserCloudIn; // raw lidar data

    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr groundCloudSAC;
    pcl::PointCloud<PointType>::Ptr segmentedCloud; // seg + ground (sparase) points
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure; // seg points
    pcl::PointCloud<PointType>::Ptr segmentedCloudPureDS; // seg points
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

    std::vector<std::pair<int8_t, int8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;
    float remove_range_points;
    int groundScanInd;
    float segmentTheta;
    int N_SCAN;
    int Horizon_SCAN;
    std::string lidarTopics;

public:
    SegAndCluster():
        nh("~"){

        nh.param<float>("remove_range",remove_range_points,0.5);
        nh.param<int>("groundscanind",groundScanInd,7);
        nh.param<float>("segtheta",segmentTheta,0.1745);
        nh.param<int>("n_scan",N_SCAN,16);
        nh.param<int>("h_scan",Horizon_SCAN,1800);
        nh.param<std::string>("lidarTopics",lidarTopics,"/vio_test/velodyne");
        // nh.param<std::string>("lidarTopics",lidarTopics,"/velodyne_ns/velodyne_points");
        // nh.param<std::string>("lidarTopics",lidarTopics,"/velodyne_points");

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopics, 1, &SegAndCluster::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubGroundSAC = nh.advertise<sensor_msgs::PointCloud2>("/SAC_groundCloud",1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        pubClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud",1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        groundCloudSAC.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPureDS.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());


        tree.reset(new pcl::search::KdTree<pcl::PointXYZI>());
        normals.reset(new pcl::PointCloud<pcl::Normal>());
        indices.reset(new std::vector <int>);
        colored_cloud.reset(new pcl::PointCloud <pcl::PointXYZRGB>());
        downSizeFilterSegPure.setLeafSize(0.1, 0.1, 0.1);

        // coefficientsPtr.reset(new pcl::ModelCoefficients());
        // inliersPtr.reset(new pcl::PointIndices());
        

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        groundCloudSAC->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        segmentedCloudPureDS->clear();
        outlierCloud->clear();
        colored_cloud->clear();
        normals->clear();
        clusters.clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~SegAndCluster(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        ros::Time time1 = ros::Time::now();
        // std::cout <<"test" <<std::endl;
        copyPointCloud(laserCloudMsg);
        findStartEndAngle();
        projectPointCloud(); // range image : range , row , col 
        groundRemoval();
        cloudSegmentation();
        ransacPlane();
        // calculateNormal();
 
        // std::cout << "SegentedCloudPureDS_size: " << segmentedCloudPureDS->points.size()<<std::endl;
 
        // regionGrowingSegmentation();

        publishCloud();
        resetParameters();

        ros::Time time2 = ros::Time::now();
        std::cout<< "time_last: " << (time2 - time1).toSec() << std::endl; 
    }

    void findStartEndAngle(){
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    void projectPointCloud(){
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

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

            fullInfoCloud->points[index].intensity = range;
        }
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
            }
        }
        // if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        // }
    }

    void cloudSegmentation(){
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){ // label or ground
                    if (labelMat.at<int>(i,j) == 999999){ // do not use label:999999
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue; // sparase ground 1/5
                    }
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]); // seg + ground(sparase)
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        // if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        // }
    }

    void labelComponents(int row, int col){
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){ // left right down up

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }


        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){ // 5 =< allPushedIndSize < 30
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum) //3
                feasibleSegment = true;            
        }

        if (feasibleSegment == true){
            ++labelCount;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void calculateNormal(){

        downSizeFilterSegPure.setInputCloud(segmentedCloudPure);
        downSizeFilterSegPure.filter(*segmentedCloudPureDS);

        tree->setInputCloud(segmentedCloudPureDS);

        normalEstimation.setInputCloud(segmentedCloudPureDS);
        normalEstimation.setSearchMethod(tree);
        normalEstimation.setKSearch(20);
        normalEstimation.compute(*normals);  


    }

    void regionGrowingSegmentation(){

        reg.setMinClusterSize (30);
        reg.setMaxClusterSize (5000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (segmentedCloudPureDS);
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

        colored_cloud = reg.getColoredCloud();

    }

    bool ransacPlane(){
        
        pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(groundCloud);
        seg.segment(*inliersPtr, *coefficientsPtr);

        if (inliersPtr->indices.size() == 0){
            std::cout << "error: no plane points"<< std::endl;
            return false;
        }    
        
        std::cout << "floor plane coefficients: " << coefficientsPtr->values[0] << " "
         << coefficientsPtr->values[1] << " "
         << coefficientsPtr->values[2] << " "
         << coefficientsPtr->values[3] << std::endl;

        double wa = coefficientsPtr->values[0];
        double wb = coefficientsPtr->values[1];
        double wc = coefficientsPtr->values[2];
        double wd = coefficientsPtr->values[3];
        double normal_Norm = sqrt(wa*wa + wb*wb + wc*wc);
        // std::cout << "normal_Norm: "<< normal_Norm <<std::endl;
        printf("normal_Norm: %f", normal_Norm);

        std::cout << std::endl;

        pcl::ExtractIndices<PointType> extract;
        extract.setIndices(boost::make_shared<const pcl::PointIndices>(*inliersPtr));
        extract.setInputCloud(groundCloud);

        std::cout << "groundCloud: "<<groundCloud->points.size()<<std::endl;
        std::cout << std::endl;

        extract.filter(*groundCloudSAC);
        std::cout << "groundCloudSAC: "<<groundCloudSAC->points.size()<<std::endl;
        std::cout << std::endl;

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(*groundCloudSAC, cloudMsg);
        cloudMsg.header.stamp = cloudHeader.stamp;
        cloudMsg.header.frame_id = cloudHeader.frame_id;
        pubGroundSAC.publish(cloudMsg);
        
        return true;

    }

    
    void publishCloud(){

        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);

        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = cloudHeader.frame_id;
        pubOutlierCloud.publish(laserCloudTemp);
        // std::cout <<"outliercloud: "<< outlierCloud->points.size() <<std::endl;

        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        // lserCloudTemp.header.frame_id = "camera";
        laserCloudTemp.header.frame_id = cloudHeader.frame_id;
        pubSegmentedCloud.publish(laserCloudTemp);
        // std::cout <<"segmentcloud: "<< segmentedCloud->points.size() <<std::endl;

        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "camera";
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubFullCloud.publish(laserCloudTemp);
        }

        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "camera";
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubGroundCloud.publish(laserCloudTemp);
        }

        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "camera";
        laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }

        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            // laserCloudTemp.header.frame_id = "camera";
            pubFullInfoCloud.publish(laserCloudTemp);
        }

        if (pubClusterCloud.getNumSubscribers() !=0){
            pcl::toROSMsg(*colored_cloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubClusterCloud.publish(laserCloudTemp); 
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    SegAndCluster IP;

    ROS_INFO("\033[1;32m---->\033[0m Segmentation and Cluster Started.");

    ros::spin();
    return 0;
}