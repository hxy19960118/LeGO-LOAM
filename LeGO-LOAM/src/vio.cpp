#include "utility.h"

class VIOAssociation{

private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subAtt;
    ros::Subscriber subPos;

    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubOutlierCloudLast;
    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr segmentedCloud; // seg + ground (sparase)
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;


    pcl::VoxelGrid<PointType> downSizeFilter;

    double timeScanCur;
    double timeNewSegmentedCloud;
    double timeNewPos;
    double timeNewAtt;
    double timeNewSegmentedCloudInfo;
    double timeNewOutlierCloud;

    bool newSegmentedCloud;
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;
    bool newatt;
    bool newpos;

    std_msgs::Header cloudHeader;
    cloud_msgs::cloud_info segInfo;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];

    int imuPointerFront;
    int imuPointerLast;
    int imuPointerLastIteration;

    float imuRollStart, imuPitchStart, imuYawStart;
    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
    float imuRollCur, imuPitchCur, imuYawCur;

    float imuVeloXStart, imuVeloYStart, imuVeloZStart;
    float imuShiftXStart, imuShiftYStart, imuShiftZStart;

    float imuVeloXCur, imuVeloYCur, imuVeloZCur;
    float imuShiftXCur, imuShiftYCur, imuShiftZCur;

    float transformSum[6];

    std::vector<smoothness_t> cloudSmoothness;
    float cloudCurvature[40000];
    int cloudNeighborPicked[40000];
    int cloudLabel[40000];

    int skipFrameNum;
    int frameCount;

    nav_msgs::Odometry laserOdometry;
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    string attTopic;
    string posTopic;
    int N_SCAN;
    int Horizon_SCAN;

public:

    VIOAssociation():
        nh("~")
        {
            nh.param<string>("vio_att_topic",attTopic,"/dat");
            nh.param<string>("vio_pos_topic",posTopic,"/dat");
            nh.param<int>("n_scan",N_SCAN,16);
            nh.param<int>("h_scan",Horizon_SCAN,1800);

            subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &VIOAssociation::laserCloudHandler, this);
            subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &VIOAssociation::laserCloudInfoHandler, this);
            subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &VIOAssociation::outlierCloudHandler, this);
            subAtt = nh.subscribe<geometry_msgs::PoseStamped>(attTopic, 50, &VIOAssociation::attHandler, this);
            // subVel = nh.subscribe<geometry_msgs::Vector3Stamped>("/vio_data_rigid1/vel", 50, &VIOAssociation::velHandler, this);
            subPos = nh.subscribe<geometry_msgs::PoseStamped>(posTopic, 50, &VIOAssociation::posHandler, this);

            pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
            pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
            pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
            pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

            pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
            pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
            pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
            pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
        
            initializationValue();
    
        }

    void initializationValue(){
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        timeScanCur = 0;
        timeNewSegmentedCloud = 0;
        timeNewPos= 0;
        timeNewAtt= 0;
        timeNewSegmentedCloudInfo = 0;
        timeNewOutlierCloud = 0;

        newSegmentedCloud = false;
        newSegmentedCloudInfo = false;
        newOutlierCloud = false;
        newatt= false;
        newpos = false;

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
        cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
        sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
        imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

        imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
        imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

        imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
        imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
        }

        for (int i = 0; i < 6; ++i){
            transformSum[i] = 0;
        }

        laserOdometry.header.frame_id = "/camera_init";
        laserOdometry.child_frame_id = "/laser_odom";

        laserOdometryTrans.frame_id_ = "/camera_init";
        laserOdometryTrans.child_frame_id_ = "/laser_odom";

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());

        skipFrameNum = 1;
        frameCount = skipFrameNum;


    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        timeScanCur = cloudHeader.stamp.toSec();
        timeNewSegmentedCloud = timeScanCur;

        segmentedCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

        newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

        timeNewOutlierCloud = msgIn->header.stamp.toSec();

        outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *outlierCloud);

        newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn)
    {
        timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        segInfo = *msgIn;
        newSegmentedCloudInfo = true;
    }

    void posHandler(const geometry_msgs::PoseStamped::ConstPtr& imuIn)
    {
        if (imuPointerLast == -1)
            return;

        timeNewPos = imuIn->header.stamp.toSec();
        imuShiftX[imuPointerLast] = imuIn->pose.position.y; 
        imuShiftY[imuPointerLast] = imuIn->pose.position.z; 
        imuShiftZ[imuPointerLast] = imuIn->pose.position.x; 
        // newpos = true;
        // AccumulateIMUShiftAndRotation();
    }

    void attHandler(const geometry_msgs::PoseStamped::ConstPtr& imuIn)
    {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);


        imuPointerLast = (imuPointerLast + 1) % imuQueLength;

        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        timeNewAtt = imuTime[imuPointerLast];

        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;
        // newatt = true;

    }

    void TransformToMap(PointType *p){
        float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
        float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
        float z1 = p->z;

        float x2 = x1;
        float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
        float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

        p->x = cos(imuYawCur) * x2 + sin(imuYawCur) * z2 + imuShiftXCur;
        p->y = y2 + imuShiftYCur;
        p->z = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2 + imuShiftZCur;
    }
    
    void adjustDistortion()
    {
        bool halfPassed = false;
        int cloudSize = segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {

            point.x = segmentedCloud->points[i].y;
            point.y = segmentedCloud->points[i].z;
            point.z = segmentedCloud->points[i].x;

            float ori = -atan2(point.x, point.z);
            if (!halfPassed) {
                if (ori < segInfo.startOrientation - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - segInfo.startOrientation > M_PI)
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;

                if (ori < segInfo.endOrientation - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.endOrientation + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
            point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;// row + 0.1*relTime

            if (imuPointerLast >= 0) {
                float pointTime = relTime * scanPeriod;
                imuPointerFront = imuPointerLastIteration;

            // time not sync
                    // imuRollCur = imuRoll[imuPointerLast];
                    // imuPitchCur = imuPitch[imuPointerLast];
                    // imuYawCur = imuYaw[imuPointerLast];

                    // imuVeloXCur = imuVeloX[imuPointerLast];
                    // imuVeloYCur = imuVeloY[imuPointerLast];
                    // imuVeloZCur = imuVeloZ[imuPointerLast];

                    // imuShiftXCur = imuShiftX[imuPointerLast];
                    // imuShiftYCur = imuShiftY[imuPointerLast];
                    // imuShiftZCur = imuShiftZ[imuPointerLast]; 

            // time sync
                while (imuPointerFront != imuPointerLast) {
                    if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
                        break;
                    }
                    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
                }

 

                if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                    imuRollCur = imuRoll[imuPointerFront];
                    imuPitchCur = imuPitch[imuPointerFront];
                    imuYawCur = imuYaw[imuPointerFront];

                    // imuVeloXCur = imuVeloX[imuPointerFront];
                    // imuVeloYCur = imuVeloY[imuPointerFront];
                    // imuVeloZCur = imuVeloZ[imuPointerFront];

                    imuShiftXCur = imuShiftX[imuPointerFront];
                    imuShiftYCur = imuShiftY[imuPointerFront];
                    imuShiftZCur = imuShiftZ[imuPointerFront];   
                    // std::cout<<"test1";
                } 
                else {
                    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                    imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                    imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                    if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                    }

                    // imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                    // imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                    // imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                    imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                    imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                    imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
                    // std::cout<<"test2";
                } 

                if (i == 0) {
                    imuRollStart = imuRollCur;
                    imuPitchStart = imuPitchCur;
                    imuYawStart = imuYawCur;

                    // imuVeloXStart = imuVeloXCur;
                    // imuVeloYStart = imuVeloYCur;
                    // imuVeloZStart = imuVeloZCur;

                    imuShiftXStart = imuShiftXCur;
                    imuShiftYStart = imuShiftYCur;
                    imuShiftZStart = imuShiftZCur;

                // time not sync
                    // imuAngularRotationXCur = imuAngularRotationX[imuPointerLast];
                    // imuAngularRotationYCur = imuAngularRotationY[imuPointerLast];
                    // imuAngularRotationZCur = imuAngularRotationZ[imuPointerLast];

                // time sync
                    // if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                    //     imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                    //     imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                    //     imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
                    // }else{
                    //     int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    //     float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                    //                                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    //     float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                    //                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    //     imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                    //     imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                    //     imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
                    // }

                    // imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
                    // imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
                    // imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

                    // imuAngularRotationXLast = imuAngularRotationXCur;
                    // imuAngularRotationYLast = imuAngularRotationYCur;
                    // imuAngularRotationZLast = imuAngularRotationZCur;

                    // updateImuRollPitchYawStartSinCos();
                } else {
                    // ShiftToMap(pointTime);
                    // VeloToStartIMU();
                    TransformToMap(&point);
                }
            }
            segmentedCloud->points[i] = point;
        }

        imuPointerLastIteration = imuPointerLast;
    }

    void calculateSmoothness()
    {
        int cloudSize = segmentedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {

            float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                            + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                            + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
                            + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
                            + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
                            + segInfo.segmentedCloudRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = segmentedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i){

            float depth1 = segInfo.segmentedCloudRange[i];
            float depth2 = segInfo.segmentedCloudRange[i+1];
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));

            if (columnDiff < 10){

                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]);
            float diff2 = std::abs(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]);

            if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        for (int i = 0; i < N_SCAN; i++) {

            surfPointsLessFlatScan->clear();

            for (int j = 0; j < 6; j++) {

                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == false) {
                    
                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold 
                        // segInfo.segmentedCloudGroundFlag[ind] == true) {
                    ){

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(segmentedCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }
    }

    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudOutMsg;

	    if (pubCornerPointsSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsSharp.publish(laserCloudOutMsg);
	    }

	    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsLessSharp.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsFlat.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsLessFlat.publish(laserCloudOutMsg);
	    }
    }

    void updatetransformEnd(){
        transformSum[0] = imuPitchCur;
        transformSum[1] = imuYawCur;
        transformSum[2] = imuRollCur;
        transformSum[3] = imuShiftXCur;
        transformSum[4] = imuShiftYCur;
        transformSum[5] = imuShiftZCur;
    }

    void publishOdometry(){
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

        laserOdometry.header.stamp = cloudHeader.stamp;
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = transformSum[3];
        laserOdometry.pose.pose.position.y = transformSum[4];
        laserOdometry.pose.pose.position.z = transformSum[5];
        pubLaserOdometry.publish(laserOdometry);

        laserOdometryTrans.stamp_ = cloudHeader.stamp;
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);
    }

    void adjustOutlierCloud(){
        PointType point;
        int cloudSize = outlierCloud->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            point.x = outlierCloud->points[i].y;
            point.y = outlierCloud->points[i].z;
            point.z = outlierCloud->points[i].x;
            point.intensity = outlierCloud->points[i].intensity;
            outlierCloud->points[i] = point;
        }
    }


    void TransformToEnd(PointType const * const pi, PointType * const po)
    {
        float x1 = cos(transformSum[1]) * (pi->x - transformSum[3]) - sin(transformSum[1]) * (pi->z - transformSum[5]);
        float y1 = pi->y - transformSum[4];
        float z1 = sin(transformSum[1]) * (pi->x - transformSum[3]) + cos(transformSum[1]) * (pi->z - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1; 
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) *z1;

        po->x = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        po->y = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        po->z = z2;
        po->intensity = int(pi->intensity);

    }

    void publishCloudsLast(){


        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        // laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        // laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        // if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
        //     kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        //     kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        // }

        frameCount++;

        if (frameCount >= skipFrameNum + 1) {

            frameCount = 0;

            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = cloudHeader.stamp;
            outlierCloudLast2.header.frame_id = "/camera";
            pubOutlierCloudLast.publish(outlierCloudLast2);

            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
            laserCloudCornerLast2.header.frame_id = "/camera";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
            laserCloudSurfLast2.header.frame_id = "/camera";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    void runVIOAssociation(){
        if (imuPointerLast == -1) 
            return;

        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewAtt - timeNewPos) < 0.01 ){

            newSegmentedCloud = false;
            newSegmentedCloudInfo = false;
            newOutlierCloud = false;
        } else{
            return;
        }

        adjustDistortion(); // pre regist

        calculateSmoothness(); // smooth

        markOccludedPoints();

        extractFeatures();

        publishCloud();

        updatetransformEnd();

        publishOdometry();

        publishCloudsLast();  

    }
};


int main(int argc, char**argv){
    ros::init(argc,argv,"lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m VIO Association Started.");

    VIOAssociation VA;

    ros::Rate rate(400);
    while (ros::ok()){
        ros::spinOnce();

        VA.runVIOAssociation();

        rate.sleep();
    }

    ros::spin();
    return 0;
}