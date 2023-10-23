#include <cmath>

#include "pcd_to_2d/pcd_to_2d.h"

#include <pcl/sample_consensus/ransac.h>

#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/opencv.hpp>

#include <grid_map_cv/GridMapCvConverter.hpp>

#include <algorithm>

#include <random>

#include <pcl/filters/voxel_grid.h>

namespace pcdWindows {

    ///This function calculates the average pose from the pose graph
    PointTypePose pcdWindows::PCDWindows::calcAveragePose(pcl::PointCloud < PointTypePose > ::Ptr poses) {
        PointTypePose averages;
        for (size_t i = 0; i < poses -> size(); ++i) {
            PointTypePose point = poses -> points[i];
            averages.roll += point.roll;
            averages.pitch += point.pitch;
            averages.yaw += point.yaw;
            averages.x += point.x;
            averages.y += point.y;
            averages.z += point.z;
        }
        size_t num_poses = poses -> size();
        averages.roll /= num_poses;
        averages.pitch /= num_poses;
        averages.yaw /= num_poses;
        averages.x /= num_poses;
        averages.y /= num_poses;
        averages.z /= num_poses;
        std::cout << "average pose x: " << averages.x << " average pose y: " << averages.y << " average pose z: " << averages.z << " average pose roll: " << averages.roll << " average pose pitch: " << averages.pitch << " average pose yaw: " << averages.yaw << std::endl;
        return averages;
    }

    ///This function calculates the maximum length of the point cloud in y direction
    float PCDWindows::calcCloudHeight(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, float marginY) {
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D( * cloud, minPt, maxPt);
        float minY = minPt.y - marginY;
        float maxY = maxPt.y + marginY;
        float cloudHeight = maxY - minY;
        return cloudHeight;
    }

    //This function projects the obstacle points that were detected from don on the gridmap
    void PCDWindows::mark_obstacles_on_grid_map(pcl::PointCloud < pcl::PointNormal > ::Ptr obstacles, grid_map::GridMap gridMap) {
        grid_map::Index index;
        // grid_map::Matrix& gridMapData = gridMap["elevation"]; // get elevation layer
        for (const auto & point: obstacles -> points) {
            if (point.z > -10 && point.z < 10) {
                if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
                gridMap.at("elevation", index) = 1000;
            }
        }
    }
    ///this function converts the point cloud to elevation map
    void PCDWindows::point_cloud_to_elevation_map(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, grid_map::GridMap elevation_map) {
        grid_map::Index index;
        // grid_map::Matrix& gridMapData = gridMap["elevation"]; // get elevation layer
        for (const auto & point: cloud -> points) {
            if (point.z > -10 && point.z < 10) {
                if (!elevation_map.getIndex(grid_map::Position(point.x, point.y), index)) continue;
                elevation_map.at("elevation", index) = point.z;
            }
        }
    }

    ///takes in the gridmap with elevation layer and traversibility layer and fills the traversibility from elevation
    grid_map::GridMap PCDWindows::calculate_terrain_roughness(grid_map::GridMap gridMap) {
        // grid_map::GridMap traversability_map;
        // traversability_map.setGeometry(grid_map::Length(width, height), 0.05, grid_map::Position(c_x, c_y));
        // traversability_map.add("traversability");
        // traversability_map.add("slope");
        float max_slope = -9999999999.0;
        float min_slope = 9999999999.0;
        for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
            /// Get the index of the current grid cell.
            const auto index = * iterator;
            /// Get the elevation values at the neighboring cells.
            double elev_n = gridMap.at("elevation", index + grid_map::Index(0, 2)); //north
            double elev_s = gridMap.at("elevation", index + grid_map::Index(0, -2)); //south
            double elev_e = gridMap.at("elevation", index + grid_map::Index(2, 0)); //east
            double elev_w = gridMap.at("elevation", index + grid_map::Index(-2, 0)); //west
            double elev_ne = gridMap.at("elevation", index + grid_map::Index(2, 2)); //north east
            double elev_se = gridMap.at("elevation", index + grid_map::Index(2, -2)); //south east
            double elev_nw = gridMap.at("elevation", index + grid_map::Index(-2, 2)); //north west
            double elev_sw = gridMap.at("elevation", index + grid_map::Index(-2, -2)); //south west
            double elev = gridMap.at("elevation", index);
            /// Compute the slope in the x direction.
            double dz_dx = (elev_e - elev_w) / (2.0 * gridMap.getResolution());
            // /Compute the slope in the y direction.
            double dz_dy = (elev_n - elev_s) / (2.0 * gridMap.getResolution());
            /// Compute the slope angle.
            double slope = atan(sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
            gridMap.at("traversability", index) = slope;
        }
        ///CREATE EMPTY IMAGE
        cv::Mat image;
        cv::Mat image_grey;
        ///CONVERT GRIDMAP LAYERS TO IMAGE
        grid_map::GridMapCvConverter::toImage < unsigned char, 3 > (gridMap, "traversability", CV_8UC3, image);
        grid_map::GridMapCvConverter::toImage < unsigned char, 1 > (gridMap, "traversability", CV_8UC1, image_grey);
        cv::Mat filtered_image;
        cv::Mat colorMap;
        cv::applyColorMap(image, colorMap, cv::COLORMAP_JET);
        ///BLUR
        cv::bilateralFilter(image, filtered_image, 9, 75, 75);
        ///WRITE IMAGES
        bool check = imwrite("/home/otto-testing/Desktop/traversibility.jpg", colorMap);
        check = imwrite("/home/otto-testing/Desktop/traversibility_grey.jpg", image_grey);
        cv::Mat elevation_img;
        cv::Mat elevation_grey;
        grid_map::GridMapCvConverter::toImage < unsigned char, 3 > (gridMap, "elevation", CV_8UC3, elevation_img);
        grid_map::GridMapCvConverter::toImage < unsigned char, 1 > (gridMap, "elevation", CV_8UC1, elevation_grey);
        cv::Mat elevation_colorMap;
        cv::applyColorMap(elevation_img, elevation_colorMap, cv::COLORMAP_JET);
        check = imwrite("/home/otto-testing/Desktop/elevation.jpg", elevation_colorMap);
        check = imwrite("/home/otto-testing/Desktop/elevation_grey.jpg", elevation_grey);
        return gridMap;
    }

    ///this function calculates the maximum length of the point cloud in x direction
    float PCDWindows::calcCloudWidth(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, float marginX) {
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D( * cloud, minPt, maxPt);
        float minX = minPt.x - marginX;
        float maxX = maxPt.x + marginX;
        float cloudWidth = maxX - minX;
        return cloudWidth;
    }
};

///main function
int main(int argc, char ** argv) {

    int int_var;
    double double_var;
    std::string pose_path;
    std::string pcd_path;
    double sml_scale, lrge_scale, thresh, seg_radius;

    ros::init(argc, argv, "pcdWindows");
    ros::NodeHandle nh;
    ros::Publisher gridMapPublisher;
    ros::Publisher slope_occ_pub;
    ros::Publisher obstacles_occ_pub;
    ros::Publisher elevation_occ_pub;

    ROS_INFO("PCD PATH:%s", pcd_path.c_str());

    ros::Rate loop_rate(10);
    gridMapPublisher = nh.advertise < grid_map_msgs::GridMap > ("grid_map", 1);
    slope_occ_pub = nh.advertise < nav_msgs::OccupancyGrid > ("terrain_roughness", 1);
    obstacles_occ_pub = nh.advertise < nav_msgs::OccupancyGrid > ("obstacles", 1);
    elevation_occ_pub = nh.advertise < nav_msgs::OccupancyGrid > ("elevation", 1);
    //LOAD THE POINT CLOUD
    std::cout << "LOADING THE FILES..." << std::endl;

    // LOAD THE POSES
    pcl::PointCloud < PointTypePose > ::Ptr poses(new pcl::PointCloud < PointTypePose > );
    pcl::io::loadPCDFile < PointTypePose > ("/home/otto-testing/slope_corrected_transform/6D_pose.pcd", * poses);
    std::cout << "DONE LOADING!" << std::endl;

    PointTypePose min, max;
    pcl::getMinMax3D( * poses, min, max);

    std::cout << min.x << "  " << min.y << "  " << min.z << std::endl;
    std::cout << max.x << "  " << max.y << "  " << max.z << std::endl;

    //OUTPUT CLOUD
    // pcl::PointCloud < pcl::PointXYZ > ::Ptr out_cloud(new pcl::PointCloud < pcl::PointXYZ > );
    // pcl::PointCloud < pcl::PointXYZ > ::Ptr out_segmented(new pcl::PointCloud < pcl::PointXYZ > );
    //CALCULATE THE AVERAGE POSE
    pcdWindows::PCDWindows myWindow; // create an object of the class
    PointTypePose averagePose = myWindow.calcAveragePose(poses); // call the member function using the object
    //CALCULATE CLOUD HEIGHT
  

    int i = 0;
    pcl::PointCloud < pcl::PointXYZ > ::Ptr obstacles_all(new pcl::PointCloud < pcl::PointXYZ > );
    pcl::PointCloud < pcl::PointXYZ > ::Ptr out_cloud(new pcl::PointCloud < pcl::PointXYZ > );


    for (const auto & point: poses -> points) {


        pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ > );
        pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud_small(new pcl::PointCloud < pcl::PointXYZ > );
        pcl::io::loadPCDFile("/home/otto-testing/slope_corrected_transform/surf_"+std::to_string(i)+".pcd", * cloud);

        // pcl::VoxelGrid<pcl::PointXYZ> downfilter;
        // downfilter.setLeafSize(0.1, 0.1, 0.1);
        // downfilter.setInputCloud(cloud);
        // downfilter.filter(*cloud);

        pcl::CropBox < pcl::PointXYZ > window_filter;

        PointTypePose pose_1;
        pose_1 = point;

        std::cout<<pose_1.x<< "   " <<pose_1.y<< "   "<<pose_1.z<<std::endl;
        window_filter.setInputCloud(cloud);
        window_filter.setMin(Eigen::Vector4f(-100000, -100000, -100000, 1.0));
        window_filter.setMax(Eigen::Vector4f(1000000,10000000, pose_1.z, 1.0));
        window_filter.filter( * cloud_small);

        for (const auto & point_temp: cloud_small -> points){out_cloud -> push_back(point_temp);}
        cloud_small -> clear();
        i = i + 1;
        std::cout<<i<<std::endl;

        window_filter.setInputCloud(cloud);
        window_filter.setMin(Eigen::Vector4f(-100000, -100000, pose_1.z, 1.0));
        window_filter.setMax(Eigen::Vector4f(1000000,10000000, pose_1.z+1, 1.0));
        window_filter.filter( * cloud_small);

        for (const auto & point: cloud_small -> points){obstacles_all -> push_back(point);}
        }

    pcl::StatisticalOutlierRemoval < pcl::PointXYZ > sor;
    sor.setInputCloud(out_cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1.0);
    sor.filter( * out_cloud);

    sor.setInputCloud(obstacles_all);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1.0);
    sor.filter( * obstacles_all);


    float cloudHeight = myWindow.calcCloudHeight(out_cloud, pcdWindows::marginY = 2);
    //CALCULATE CLOUD WIDTH
    float cloudWidth = myWindow.calcCloudWidth(out_cloud, pcdWindows::marginX = 2);
    //CALCULATE WINDOW PARAMETERS
    // pcdWindows::windowParams window = myWindow.calcWindowParams(cloud, cloudHeight, cloudWidth, pcdWindows::windowWidth, pcdWindows::windowHeight);
    //CALCULATE MAX AND MIN PTS OF THE CLOUD
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D( * out_cloud, minPt, maxPt);
    float minX = minPt.x - pcdWindows::marginX;
    float maxX = maxPt.x + pcdWindows::marginX;
    float minY = minPt.y - pcdWindows::marginY;
    float maxY = maxPt.y + pcdWindows::marginY;
    //CREATE A CROPBOX TO CROP THE POINTS OF THE CLOUD THAT ARE LYING IN THE WINDOW 

    //SET GEOMETRY OF GRIDMAP FROM CLOUD MIN MAX
    grid_map::GridMap gridMap;
    gridMap.setFrameId("grid_map");
    float width = cloudWidth;
    float height = cloudHeight;
    float c_x = (maxX + minX) / 2;
    float c_y = (minY + maxY) / 2;
    std::cout<<"width = "<<width<<"  height = "<<height<<std::endl;
    gridMap.setGeometry(grid_map::Length(width, height), 0.1, grid_map::Position(c_x, c_y));

    //ADD LAYERS TO THE GRIDMAP
    gridMap.add("obstacles");
    // gridMap.add("occ_grid");
    gridMap.add("elevation");
    // gridMap.add("slope");
    gridMap.add("traversability");
    grid_map::Index index;

    pcl::io::savePCDFileASCII("/home/otto-testing/catkin_ws/src/pcd_to_2d/output/z_cut.pcd", * out_cloud);
    obstacles_all -> width = obstacles_all -> size();
    obstacles_all -> height = 1;
    obstacles_all -> is_dense = true;
    pcl::io::savePCDFileASCII("/home/otto-testing/catkin_ws/src/pcd_to_2d/output/obstacles.pcd", * obstacles_all);
    //CONVERT THE POINT CLOUD TO ELEVATION
    for (const auto & point: out_cloud -> points) {
        if (point.z > -200 && point.z < 200) {
            if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            gridMap.at("elevation", index) = point.z;
        }
    }

    // Find all NaN values in the grid and their indices
    std::vector < grid_map::Index > nanIndices;
    const auto size = gridMap.getSize();
    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
        const auto index = * it;
        float & data = gridMap.at("elevation", index);
        if (std::isnan(data)) {
            nanIndices.push_back(index);
        }
    }

    // Shuffle the vector of NaN indices to iterate over them in random order
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::shuffle(nanIndices.begin(), nanIndices.end(), rng);

    // Iterate over the shuffled vector of NaN indices
    long int a = 0;
    for (const auto & index: nanIndices) {
        a = a + 1;
        std::cout << (100 * a) / nanIndices.size() << " percent complete" << std::endl;
        float & data = gridMap.at("elevation", index);
        // Define the size of the window to use
        const int windowSize = 2; // Change this to adjust the window size.
        float sum = 0.0;
        long int count = 0;
        // Iterate over neighboring cells within the window
        for (long int i = -windowSize; i <= windowSize; ++i) {
            for (long int j = -windowSize; j <= windowSize; ++j) {
                const auto neighborIndex = grid_map::Index(index.x() + i, index.y() + j);
                if (neighborIndex.x() >= 0 && neighborIndex.y() >= 0 && neighborIndex.x() < size.x() && neighborIndex.y() < size.y()) {
                    const auto neighborData = gridMap.at("elevation", neighborIndex);
                    // Check that the neighbor is not NaN
                    if (!std::isnan(neighborData)) {
                        sum += neighborData;
                        ++count;
                    }
                }
            }
        }
        if (count > 0) {
            // Update the current value with the average of the neighboring non-NaN values
            data = sum / count;
        }
    }

    //CLEAR TO SAVE RAM
    nanIndices.clear();
    out_cloud -> clear();

    //MARK OBSTACLES ON THE GRID MAP
    for (const auto & point: obstacles_all -> points) {
        if (point.z > -200 && point.z < 200) {
            if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            gridMap.at("obstacles", index) = 1000;
        }
    }
    // std::cout<<"720"<<std::endl;
    grid_map::Matrix & data = gridMap["obstacles"];
    // data = gridMap["obstacles"];
    // std::cout<<"722"<<std::endl;

    //FILL EMPTY CELLS WITH 0 IN THE OBSTACLES LAYER OF GRID MAP
    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
        if (std::isnan(data(iterator.getLinearIndex()))) {
            data(iterator.getLinearIndex()) = 0;
        }
    }
    // std::cout<<"728"<<std::endl;
    //CLEAR THE CLOUD TO SAVE RAM
    out_cloud -> clear();

    //DEFINE THE MESSAGES AND 
    grid_map_msgs::GridMap gridMapMessage;
    // std::cout<<"731"<<std::endl;
    nav_msgs::OccupancyGrid obstacles_occ;
    nav_msgs::OccupancyGrid terrain_roughness;
    nav_msgs::OccupancyGrid obstacle_slope;
    nav_msgs::OccupancyGrid elevation_occ;

    float max_slope = -9999999999.0;
    float min_slope = 9999999999.0;
    //ITERATE THROUGH THE CELLS OF ELEVATION MAP


float min_traversability = std::numeric_limits<float>::max();
float max_traversability = std::numeric_limits<float>::min();


    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
        // Get the index of the current grid cell.
        const auto index = * iterator;
        // Get the elevation values at the neighboring cells.
        //CALCULATE SLOPE WITH RESPECT TO NEIGHBORING CELLS
        double elev_n = gridMap.at("elevation", index + grid_map::Index(0, 1)); //north (0,2) MEANS TWO CELLS UP
        double elev_s = gridMap.at("elevation", index + grid_map::Index(0, -1)); //south
        double elev_e = gridMap.at("elevation", index + grid_map::Index(1, 1)); //east
        double elev_w = gridMap.at("elevation", index + grid_map::Index(-1, 0)); //west
        double elev = gridMap.at("elevation", index);
        double dz_dx = (elev_e - elev_w) / (2.0 * gridMap.getResolution());
        // Compute the slope in the y direction.
        double dz_dy = (elev_n - elev_s) / (2.0 * gridMap.getResolution());
        // Compute the slope angle.
        double slope = atan(sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
        //FILL TRAVERSABILITY LAYER WITH THE CALCULATED SLOPE
        gridMap.at("traversability", index) = slope;

        if (slope < min_traversability) min_traversability = slope;
    if (slope > max_traversability) max_traversability = slope;
    }
    // gridMap.add("traversability", traversability_map.get("traversability"));

    //CREATE EMPTY IMAGE
    cv::Mat image;
    cv::Mat image_grey;
    // bool result = grid_map::GridMapCvConverter::toImage<float, 1>(gridMap, "slope", CV_8UC1, min_slope, max_slope, image);

    //CONVERT GRIDMAP LAYERS TO IMAGE
    grid_map::GridMapCvConverter::toImage < unsigned char, 3 > (gridMap, "traversability", CV_8UC3, image);
    grid_map::GridMapCvConverter::toImage < unsigned char, 1 > (gridMap, "traversability", CV_8UC1, image_grey);
    cv::Mat filtered_image;
    cv::Mat colorMap;
    cv::applyColorMap(image, colorMap, cv::COLORMAP_JET);

    //BLUR
    cv::bilateralFilter(image, filtered_image, 9, 75, 75);
    bool check = imwrite("/home/otto-testing/Desktop/traversibility.jpg", colorMap);
    check = imwrite("/home/otto-testing/Desktop/traversibility_grey.jpg", image_grey);
    cv::Mat elevation_img;
    cv::Mat elevation_grey;
    //CONVERT GRIDMAP LAYER TO IMAGE
    grid_map::GridMapCvConverter::toImage < unsigned char, 3 > (gridMap, "elevation", CV_8UC3, elevation_img);
    grid_map::GridMapCvConverter::toImage < unsigned char, 1 > (gridMap, "elevation", CV_8UC1, elevation_grey);
    cv::Mat elevation_colorMap;
    cv::applyColorMap(elevation_img, elevation_colorMap, cv::COLORMAP_JET);
    //SAVE IMAGE
    check = imwrite("/home/otto-testing/Desktop/elevation.jpg", elevation_colorMap);
    check = imwrite("/home/otto-testing/Desktop/elevation_grey.jpg", elevation_grey);
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "traversability", min_traversability, max_traversability, terrain_roughness);
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", -10.0, 10.0, elevation_occ);

    // std::cout<<"744"<<std::endl;
    // nav_msgs::OccupancyGrid terrain_roughness;

    //PUBLISH EVERYTHING IN A LOOP
    while (ros::ok()) {
        // std::cout<<"748"<<std::endl;
        grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "obstacles", 0.0, 100.0, obstacles_occ);
        // std::cout<<"751"<<std::endl;
        // std::cout<<"753"<<std::endl;
        obstacles_occ_pub.publish(obstacles_occ);
        // std::cout<<"755"<<std::endl;
        slope_occ_pub.publish(terrain_roughness);
        elevation_occ_pub.publish(elevation_occ);

        // std::cout<<"757"<<std::endl;
        grid_map::GridMapRosConverter::toMessage(gridMap, gridMapMessage);
        // std::cout<<"760"<<std::endl;
        gridMapPublisher.publish(gridMapMessage);
        // std::cout<<"762"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
        // std::cout<<"765"<<std::endl;
        std::cout << "PUBLISHED EVERYTHING" << std::endl;
    }
    return 0;
}