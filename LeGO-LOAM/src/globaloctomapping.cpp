
#include "utility.h"

class Globaloctomapping{

private:
    ros::NodeHandle nh;

    octomap::OcTree* Global_tree;
    nav_msgs::OccupancyGrid Global_gridmap;

    
    ros::Publisher pubGlobal2DgridMap;
    ros::Publisher pubBinaryMap;
    ros::Publisher pubGlobaloccupiedcells;
    ros::Publisher pubGlobalfreecells;

    ros::Subscriber subGlobal2Dpointcloud;

    pcl::PointCloud<PointType>::Ptr Global2Dpointcloud;
    // octomap::Pointcloud Global_cloud;

    std_msgs::Header cloudin_header;

    octomap::OcTreeKey m_paddedMinKey;

    double padding_minSizeX;
    double padding_minSizeY;

    double timeGlobal2Dpointcloud;
    std::string global_2Dcloud_topic;
    bool newGlobal2Dcloud_flag;
    double Global_max_range;
    float mapping_resolution;

    unsigned tree_depth;
    unsigned max_tree_depth;
    unsigned multires2DScale;


public:
   Globaloctomapping():
   nh("~")
   {
       nh.param<string>("global_2Dcloud_topic",global_2Dcloud_topic,"/laser_cloud_surround");
       nh.param<float>("mapping_resolution", mapping_resolution, 1.0);

       subGlobal2Dpointcloud = nh.subscribe<sensor_msgs::PointCloud2>(global_2Dcloud_topic, 2, &Globaloctomapping::GlobalmappingHandler ,this);

       pubBinaryMap = nh.advertise<octomap_msgs::Octomap>("/Global_BinaryOctomap",2);
       pubGlobaloccupiedcells = nh.advertise<visualization_msgs::Marker>("/Global_occupied_cells", 2);
       pubGlobalfreecells = nh.advertise<visualization_msgs::Marker>("/Global_free_cells",2);
       pubGlobal2DgridMap = nh.advertise<nav_msgs::OccupancyGrid>("/Global_gridmap", 2);

       initialize();

   }


    void initialize()
    {
       
       Global2Dpointcloud.reset(new pcl::PointCloud<PointType>());

       Global_tree = new octomap::OcTree(mapping_resolution);
       Global_gridmap.info.resolution = mapping_resolution;

       tree_depth = 0;
       max_tree_depth = 0;
       multires2DScale = 0;

       padding_minSizeX = 0;
       padding_minSizeY = 0;
       
       timeGlobal2Dpointcloud = 0;

       Global_max_range = 1000;

       newGlobal2Dcloud_flag = false;
    }


    void GlobalmappingHandler(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
       ros::Time starttime =  ros::Time::now();

       cloudin_header = cloud->header;
       timeGlobal2Dpointcloud = cloudin_header.stamp.toSec();
       Global2Dpointcloud->clear();
       pcl::fromROSMsg(*cloud, *Global2Dpointcloud);
       newGlobal2Dcloud_flag = true;

       Globalmappingprocess();

       VisualizeMapping();

       double time_diff = (ros::Time::now() - starttime).toSec();
       std::cout<<"GLobal_mapping time: "<<time_diff<<std::endl;



    }

    void Globalmappingprocess()
    {
        // ros::Time starttime =  ros::Time::now();

        Global_tree->clear();
        octomap_msgs::Octomap Global_msg_octomap;

        octomap::Pointcloud Global_cloud;
        // Global_cloud.clear();
        for (auto p : Global2Dpointcloud->points)
              Global_cloud.push_back(p.x, p.y, p.z);

        // std::cout << Global_cloud.size() << std::endl;

        Global_tree->insertPointCloud(Global_cloud, octomap::point3d(0,0,0));
        Global_tree->updateInnerOccupancy();
        // Global_tree->write("Global2Dmap.ot");
        octomap_msgs::binaryMapToMsg(*Global_tree, Global_msg_octomap);
        Global_msg_octomap.header = cloudin_header;
        pubBinaryMap.publish(Global_msg_octomap);
        
        // double time_diff = (ros::Time::now() - starttime).toSec();
        // std::cout<<"GLobal_mapping time: "<<time_diff<<std::endl;
    }


    void VisualizeMapping()
    {
        tree_depth = Global_tree->getTreeDepth();
        max_tree_depth = tree_depth;
        
        visualization_msgs::Marker occupiedNodesVis1;
        visualization_msgs::Marker freeNodesVis1;

        HandlePreNodeTraversal();

        occupiedNodesVis1.header.frame_id = cloudin_header.frame_id;
        occupiedNodesVis1.header.stamp = cloudin_header.stamp;
        occupiedNodesVis1.ns = "_occupied";
        occupiedNodesVis1.id = 2;
        occupiedNodesVis1.type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis1.scale.x = mapping_resolution;
        occupiedNodesVis1.scale.y = mapping_resolution;
        occupiedNodesVis1.scale.z = mapping_resolution;
        occupiedNodesVis1.action = visualization_msgs::Marker::ADD;

        freeNodesVis1.header.frame_id = cloudin_header.frame_id;
        freeNodesVis1.header.stamp = cloudin_header.stamp;
        freeNodesVis1.ns = "free";
        freeNodesVis1.id = 1;
        freeNodesVis1.type = visualization_msgs::Marker::CUBE_LIST;
        freeNodesVis1.scale.x = mapping_resolution;
        freeNodesVis1.scale.y = mapping_resolution;
        freeNodesVis1.scale.z = mapping_resolution;
        freeNodesVis1.action = visualization_msgs::Marker::ADD;

        std_msgs::ColorRGBA collision_color;
        collision_color.r = 1.0;
        collision_color.g = 0.1;
        collision_color.b = 0.1;
        collision_color.a = 1;

        std_msgs::ColorRGBA free_color;
        free_color.r = 0.5;
        free_color.g = 0.5;
        free_color.b = 1.0;
        free_color.a = 0.1;
        
        int occupied_num;
        int free_num;
        for (octomap::OcTree::iterator it = Global_tree->begin(),
                                         end = Global_tree->end(); it != end; ++it)
         {
             if (Global_tree->isNodeOccupied(*it)){ // occupied
                 double _size = it.getSize();
                 double x = it.getX();
                 double y = it.getY();
                 double z = it.getZ();

                 UpdateGridmap(it, true);

                  geometry_msgs::Point cubeCenter;
                  cubeCenter.x = x;
                  cubeCenter.y = y;
                  cubeCenter.z = z;

                  occupiedNodesVis1.points.push_back(cubeCenter);
                  occupiedNodesVis1.colors.push_back(collision_color);

                  occupied_num++;

             }
             else{ //free
                
                UpdateGridmap(it, false);

                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();


                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                freeNodesVis1.points.push_back(cubeCenter);
                freeNodesVis1.colors.push_back(free_color);
                free_num++;

             }
          }

        
        if(pubGlobaloccupiedcells.getNumSubscribers() > 0){
            pubGlobaloccupiedcells.publish(occupiedNodesVis1);
        }
        if(pubGlobalfreecells.getNumSubscribers() > 0){
            pubGlobalfreecells.publish(freeNodesVis1);
        }
        if(pubGlobal2DgridMap.getNumSubscribers() > 0){
            pubGlobal2DgridMap.publish(Global_gridmap);
        }


    }

    void HandlePreNodeTraversal()
    {
        // if(pubGlobal2DgridMap.getNumSubscribers()!=0){

            // std::cout<<"test"<<std::endl;
            Global_gridmap.header.frame_id = cloudin_header.frame_id;
            Global_gridmap.header.stamp = cloudin_header.stamp;
            // nav_msgs::MapMetaData oldmapinfo = Global_gridmap.info;

            double minX, minY, minZ, maxX, maxY, maxZ;
            Global_tree->getMetricMin(minX, minY, minZ);
            Global_tree->getMetricMax(maxX, maxY, maxZ);

            octomap::point3d minPt(minX, minY, minZ);
            octomap::point3d maxPt(maxX, maxY, maxZ);
            octomap::OcTreeKey minKey = Global_tree->coordToKey(minPt, max_tree_depth);
            octomap::OcTreeKey maxKey = Global_tree->coordToKey(maxPt, max_tree_depth);

            ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

            double halfPaddedX = 0.5*padding_minSizeX;
            double halfPaddedY = 0.5*padding_minSizeY;
            minX = std::min(minX, -halfPaddedX);
            maxX = std::max(maxX, halfPaddedX);
            minY = std::min(minY, -halfPaddedY);
            maxY = std::max(maxY, halfPaddedY);
            minPt = octomap::point3d(minX, minY, minZ);
            maxPt = octomap::point3d(maxX, maxY, maxZ);

            octomap::OcTreeKey paddedMaxKey;
              if (!Global_tree->coordToKeyChecked(minPt, max_tree_depth, m_paddedMinKey)){
                   ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
                     return;
                 }
              if (!Global_tree->coordToKeyChecked(maxPt, max_tree_depth, paddedMaxKey)){
                   ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
                     return;
                 }

            ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
            assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

            multires2DScale = 1 << (tree_depth - max_tree_depth);
            Global_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/multires2DScale +1;
            Global_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/multires2DScale +1;

            int mapOriginX = minKey[0] - m_paddedMinKey[0];
            int mapOriginY = minKey[1] - m_paddedMinKey[1];
            assert(mapOriginX >= 0 && mapOriginY >= 0);
            
            octomap::point3d origin = Global_tree->keyToCoord(m_paddedMinKey, tree_depth);
            double gridRes = Global_tree->getNodeSize(max_tree_depth);
 
            Global_gridmap.info.resolution = gridRes;
            Global_gridmap.info.origin.position.x = origin.x() - 0.5*gridRes;
            Global_gridmap.info.origin.position.y = origin.y() - 0.5*gridRes;
            if (max_tree_depth != tree_depth){
                               Global_gridmap.info.origin.position.x -= mapping_resolution/2.0;
                               Global_gridmap.info.origin.position.y -= mapping_resolution/2.0;
                  }       
            
            ROS_DEBUG("Rebuilding complete 2D map");
            Global_gridmap.data.clear();
            // init to unknown:
            Global_gridmap.data.resize(Global_gridmap.info.width * Global_gridmap.info.height, -1);

        // }

    }

    void UpdateGridmap(const octomap::OcTree::iterator& it, bool occupied)
    {
        if (it.getDepth() == max_tree_depth){
            unsigned idx = mapIdx(it.getKey());
            if (occupied)
                Global_gridmap.data[idx] = 100;
             else if (Global_gridmap.data[idx] == -1){
                 Global_gridmap.data[idx] = 0;
          }

        } else{
         int intSize = 1 << (max_tree_depth - it.getDepth());
         octomap::OcTreeKey minKey=it.getIndexKey();
         for(int dx=0; dx < intSize; dx++){
             int i = (minKey[0]+dx - m_paddedMinKey[0])/multires2DScale;
                for(int dy=0; dy < intSize; dy++){
                    unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/multires2DScale);
                     if (occupied)
                            Global_gridmap.data[idx] = 100;
                     else if ( Global_gridmap.data[idx] == -1){
                      Global_gridmap.data[idx] = 0;
                    }
                 }
             }
         }
    }

    unsigned mapIdx(int i, int j) const
   {
      return Global_gridmap.info.width * j + i;
   }

   unsigned mapIdx(const octomap::OcTreeKey& key) const 
   {
      return mapIdx((key[0] - m_paddedMinKey[0]) / multires2DScale,
                    (key[1] - m_paddedMinKey[1]) / multires2DScale);
   }

    

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    ROS_INFO("\033[1;32m---->\033[0m global octomapping Started.");

    Globaloctomapping Glo;

    ros::spin();

    return 0;
}