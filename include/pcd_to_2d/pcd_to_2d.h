#ifndef PCD_WINDOWS_H_
#define PCD_WINDOWS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/console/parse.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/search/organized.h>
 #include <pcl/search/kdtree.h>
 #include <pcl/features/normal_3d_omp.h>
 #include <pcl/filters/conditional_removal.h>
 #include <pcl/segmentation/extract_clusters.h>
 #include <pcl/features/don.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <cmath>
#include <grid_map_filters/MedianFillFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <cmath>
#include <iostream>

//optimised structure to save memory when working with large clouds
struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
}
EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

namespace pcdWindows {

    pcl::PointXYZ minPt, maxPt;
    float marginY = 1;
    float marginX = 1;
    float maxX;
    float minX;
    float maxY;
    float minY;
    float theta = M_PI / 2;
    /**
     * @class PCDWindows
     * @brief A class that helps iterate the point cloud in windows.
     */
    class PCDWindows {
        public:
        /**
         * @brief calculates the average of all poses
         * @param poses A reference to the pointcloud containing poses
         * @return an average pose of type PointTypePose
         */
        PointTypePose calcAveragePose(pcl::PointCloud < PointTypePose > ::Ptr poses);

        /**
         * @brief calculates the width of the point cloud (width --> along x-axis)
         * @param cloud A reference to the pointcloud whose width we have to find
         * @param marginX A float number to increase the width on both sides of the cloud (keep 0 to get original width)
         * @return the width of the cloud
         */
        float calcCloudWidth(pcl::PointCloud < pcl::PointXYZ > ::Ptr & coud, float marginX);

        /**
         * @brief calculates the height of the point cloud (height --> along x-axis)
         * @param cloud A reference to the pointcloud whose height we have to find
         * @param marginY A float number to increase the height on both sides of the cloud (keep 0 to get original height)
         * @return the height of the cloud
         */
        float calcCloudHeight(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, float marginY);

        /**
         * @brief the obstacle cloud is projected on the grid map to mark these points as obstacles
         * @param obstacles the point cloud containing obstacles
         * @param gridMap grid_map tpye of map
         * @return nothing
        */
        void mark_obstacles_on_grid_map(pcl::PointCloud < pcl::PointNormal > ::Ptr obstacles, grid_map::GridMap gridMap);
        
        /**
         * @brief calculate the fills the occupancy grid with terrain roughness takes in point cloud with z cut and returns occupancy grid
         * @param gridMap the input grid_map type map
         * @param max_slope_angle maximum slope_angle to classify the cell as non traversible
         * @param width the width of the map
         * @param height the height of the grid map
         * @param c_x x coordinate of the center
         * @param c_y y coordinate of the center
         * @return grid_map with additional layers of traversibility and slope
        */
        grid_map::GridMap calculate_terrain_roughness(grid_map::GridMap gridMap);
    
        /**
         * @brief converts the point cloud to elevation map
         * @param cloud the input point cloud
         * @param elevation_map the grid_map type map with an elevation layer
         * @return returns nothing
        */
        void point_cloud_to_elevation_map(pcl::PointCloud < pcl::PointXYZ > ::Ptr &cloud, grid_map::GridMap elevation_map) ;

    };
    
};
#endif
