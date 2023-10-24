#include <cmath>
#include "pcdWindows/pcdWindows.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <algorithm>
#include <random>

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

    ///This function calculates the maimum length of the point cloud in y direction
    float PCDWindows::calcCloudHeight(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, float marginY) {
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D( * cloud, minPt, maxPt);
        float minY = minPt.y - marginY;
        float maxY = maxPt.y + marginY;
        float cloudHeight = maxY - minY;
        return cloudHeight;
    }

    ///This function calculates the parameters of windiws (length,height), index of current window, location (maxX, maxY, minX, minY)
    windowParams PCDWindows::calcWindowParams(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, float cloudHeight, float cloudWidth, float windowWidth, float windowHeight) {
        windowParams window;
        window.numWindowY = floor(cloudHeight / windowHeight);
        if (window.numWindowY == 0) {
            window.numWindowY = 1;
        }
        window.windowHeight = cloudHeight / window.numWindowY;
        std::cout << "windowHeight :" << window.windowHeight << "    " << "numWindowY :" << window.numWindowY << std::endl;

        window.numWindowX = floor(cloudWidth / windowWidth);
        if (window.numWindowX == 0) {
            window.numWindowX = 1;
        }
        window.windowWidth = cloudWidth / window.numWindowX;
        std::cout << "windowWidth :" << window.windowWidth << "    " << "numWindowX :" << window.numWindowX << std::endl;
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D( * cloud, minPt, maxPt);
        float minX = minPt.x - marginX;
        float maxX = maxPt.x + marginX;
        float minY = minPt.y - marginY;
        float maxY = maxPt.y + marginY;
        window.windowMinX = minX;
        window.windowMaxX = minX + window.windowWidth;
        window.windowMinY = minY;
        window.windowMaxY = minY + window.windowHeight;
        // window.window_center_x = (window.windowMinX+window.windowMaxX)/2;
        // window.window_center_y = (window.windowMinY+window.windowMaxY)/2;
        return window;
    }

    ///This function calculates the nearest pose from the current window
    PointTypePose PCDWindows::findClosestPose(float window_center_x, float window_center_y, pcl::PointCloud < PointTypePose > ::Ptr poses) {
        float minDistance = std::numeric_limits < float > ::max();
        PointTypePose closestPose;
        for (size_t i = 0; i < poses -> size(); ++i) {
            const PointTypePose & p = ( * poses)[i];
            float distance = std::sqrt(std::pow(p.x - window_center_x, 2) + std::pow(p.y - window_center_y, 2));
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = p;
            }
        }
        return closestPose;
    }

    ///Remove points above a certain height
    pcl::PointCloud < pcl::PointXYZ > ::Ptr PCDWindows::performZCutOnWindow(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloudIn, pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloudOut, windowParams window, float minimum_z, float maximum_z, float closestPose_z) {
        pcl::CropBox < pcl::PointXYZ > zcut;
        zcut.setInputCloud(cloudIn);
        zcut.setMin(Eigen::Vector4f(window.windowMinX, window.windowMinY, closestPose_z + minimum_z, 1.0));
        zcut.setMax(Eigen::Vector4f(window.windowMaxX, window.windowMaxY, closestPose_z + maximum_z, 1.0));
        //zcut.setRotation(Eigen::Vector3f(0, 0, 0.0f));
        zcut.filter( * cloudOut);
        return cloudOut;
    }

    ///perform don segmentation
    int PCDWindows::get_obstacles(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, pcl::PointCloud < pcl::PointNormal > ::Ptr & obstacles, pcl::PointCloud < pcl::PointNormal > ::Ptr & obstacles_all, float scale1, float scale2, float threshold, float segradius, int windowNumber) {
        // Create a search tree, use KDTreee for non-organized data.
        pcl::search::Search < pcl::PointXYZ > ::Ptr tree;
        if (cloud -> isOrganized()) {
            tree.reset(new pcl::search::OrganizedNeighbor < pcl::PointXYZ > ());
        } else {
            tree.reset(new pcl::search::KdTree < pcl::PointXYZ > (false));
        }
        /// Set the input pointcloud for the search tree
        tree -> setInputCloud(cloud);

        if (scale1 >= scale2) {
            std::cerr << "Error: Large scale must be > small scale!" << std::endl;
            exit(EXIT_FAILURE);
        }
        /// Compute normals using both small and large scales at each point
        pcl::NormalEstimationOMP < pcl::PointXYZ, pcl::PointNormal > ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        /**
         * NOTE: setting viewpoint is very important, so that we can ensure
         * normals are all pointed in the same direction!
         */
        ne.setViewPoint(std::numeric_limits < float > ::max(), std::numeric_limits < float > ::max(), std::numeric_limits < float > ::max());
        /// calculate normals with the small scale
        std::cout << "Calculating normals for scale..." << scale1 << std::endl;
        pcl::PointCloud < pcl::PointNormal > ::Ptr normals_small_scale(new pcl::PointCloud < pcl::PointNormal > );
        ne.setRadiusSearch(scale1);
        ne.compute( * normals_small_scale);
        /// calculate normals with the large scale
        std::cout << "Calculating normals for scale..." << scale2 << std::endl;
        pcl::PointCloud < pcl::PointNormal > ::Ptr normals_large_scale(new pcl::PointCloud < pcl::PointNormal > );
        ne.setRadiusSearch(scale2);
        ne.compute( * normals_large_scale);
        /// Create output cloud for DoN results
        pcl::PointCloud < pcl::PointNormal > ::Ptr doncloud(new pcl::PointCloud < pcl::PointNormal > );
        copyPointCloud( * cloud, * doncloud);
        std::cout << "Calculating DoN... " << std::endl;
        /// Create DoN operator
        pcl::DifferenceOfNormalsEstimation < pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal > don;
        don.setInputCloud(cloud);
        don.setNormalScaleLarge(normals_large_scale);
        don.setNormalScaleSmall(normals_small_scale);
        if (!don.initCompute()) {
            std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
            exit(EXIT_FAILURE);
        }
        /// Compute DoN
        don.computeFeature( * doncloud);
        /// Save DoN features
        pcl::PCDWriter writer;
        if (doncloud -> size() > 0) {
            writer.write < pcl::PointNormal > ("don.pcd", * doncloud, false);
        }
        /// Filter by magnitude
        std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;
        /// Build the condition for filtering
        pcl::ConditionOr < pcl::PointNormal > ::Ptr range_cond(
            new pcl::ConditionOr < pcl::PointNormal > ()
        );
        range_cond -> addComparison(pcl::FieldComparison < pcl::PointNormal > ::ConstPtr(
            new pcl::FieldComparison < pcl::PointNormal > ("curvature", pcl::ComparisonOps::GT, threshold)));
        /// Build the filter
        pcl::ConditionalRemoval < pcl::PointNormal > condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(doncloud);
        pcl::PointCloud < pcl::PointNormal > ::Ptr doncloud_filtered(new pcl::PointCloud < pcl::PointNormal > );
        /// Apply filter
        condrem.filter( * doncloud_filtered);
        doncloud = doncloud_filtered;
        /// Save filtered output
        std::cout << "Filtered Pointcloud: " << doncloud -> size() << " data points." << std::endl;
        if (doncloud -> size() > 0) {
            writer.write < pcl::PointNormal > ("don_filtered.pcd", * doncloud, false);
        }
        /// Filter by magnitude
        std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;
        pcl::search::KdTree < pcl::PointNormal > ::Ptr segtree(new pcl::search::KdTree < pcl::PointNormal > );
        segtree -> setInputCloud(doncloud);
        std::vector < pcl::PointIndices > cluster_indices;
        pcl::EuclideanClusterExtraction < pcl::PointNormal > ec;
        ec.setClusterTolerance(segradius);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(segtree);
        ec.setInputCloud(doncloud);
        ec.extract(cluster_indices);
        int j = 0;
        for (const auto & cluster: cluster_indices) {
            pcl::PointCloud < pcl::PointNormal > ::Ptr cloud_cluster_don(new pcl::PointCloud < pcl::PointNormal > );
            //push points to a obstacles_all cloud that has points from all windows
            for (const auto & idx: cluster.indices) {
                cloud_cluster_don -> points.push_back(( * doncloud)[idx]);
                obstacles -> points.push_back(( * doncloud)[idx]);
                obstacles_all -> points.push_back(( * doncloud)[idx]);
            }
            cloud_cluster_don -> width = cloud_cluster_don -> size();
            cloud_cluster_don -> height = 1;
            cloud_cluster_don -> is_dense = true;
            ///Save cluster
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don -> size() << " data points." << std::endl;
            // writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
            ++j;
        }
        obstacles -> width = obstacles -> size();
        obstacles -> height = 1;
        obstacles -> is_dense = true;
        std::stringstream ss;
        ss << "win_" << j << windowNumber << ".pcd";
        //save obstacle cloud of individual window
        if (obstacles -> size() > 0) {
            writer.write < pcl::PointNormal > (ss.str(), * obstacles, false);
        }
        return 0;
    }

    //not working
    int PCDWindows::segment_road_sidewalk(pcl::PointCloud < pcl::PointXYZ > ::Ptr & cloud, std::vector < pcl::PointCloud < pcl::PointXYZ > ::Ptr > & planes, float eps_angle, float distance_threshold, int windowNumber) {
        const int max_planes = 2;
        float z_intercepts[2];
        const Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
        for (int i = 0; i < max_planes; i++) {
            // Create a segmentation object with the point cloud as input
            pcl::SampleConsensusModelPerpendicularPlane < pcl::PointXYZ > ::Ptr model(new pcl::SampleConsensusModelPerpendicularPlane < pcl::PointXYZ > (cloud));
            model -> setAxis(axis);
            model -> setEpsAngle(30*3.14/180);
            // Create a random sample consensus object with the model as input
            pcl::RandomSampleConsensus < pcl::PointXYZ > ransac(model);
            ransac.setDistanceThreshold(distance_threshold);
            // Compute the model and extract the inliers
            std::vector < int > inliers;
            ransac.computeModel();
            ransac.getInliers(inliers);
            Eigen::VectorXf coeffs;
            ransac.getModelCoefficients(coeffs);
            float z_intercept = -coeffs[3] / coeffs[2];
            z_intercepts[i] = z_intercept;
            if (inliers.size() < 3) {
                // Stop if there are not enough inliers for the current plane
                break;
            }
            // Extract the inlier points into a new point cloud object
            pcl::PointCloud < pcl::PointXYZ > ::Ptr plane(new pcl::PointCloud < pcl::PointXYZ > );
            pcl::ExtractIndices < pcl::PointXYZ > extract;
            extract.setInputCloud(cloud);
            extract.setIndices(boost::make_shared < std::vector < int >> (inliers));
            extract.setNegative(false);
            extract.filter( * plane);
            // Add the plane to the vector of extracted planes
            planes.push_back(plane);
            // Remove the inliers from the original point cloud
            extract.setNegative(true);
            extract.filter( * cloud);
        }
        std::stringstream ss;
        ss << "road_" << windowNumber << ".pcd";
        std::stringstream sss;
        sss << "sidewalk" << windowNumber << ".pcd";
        if (planes[0] -> size() > 0 && planes[1] -> size() > 0) {
            if (z_intercepts[0] > z_intercepts[1]) {
                pcl::io::savePCDFileASCII(ss.str(), * planes[1]);
                pcl::io::savePCDFileASCII(sss.str(), * planes[0]);
            }
            if (z_intercepts[0] < z_intercepts[1]) {
                pcl::io::savePCDFileASCII(sss.str(), * planes[1]);
                pcl::io::savePCDFileASCII(ss.str(), * planes[0]);
            }
        }
        return 0;
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


            // double mean_elevation = (elev_n + elev_s + elev_e + elev_w + elev_ne + elev_se + elev_nw + elev_sw)/8;


            // double dz_dne = (elev_ne - elev_w) / (2.0 * sqrt(2.0) * gridMap.getResolution()); //slope northeast
            // double dz_dse = (elev_se - elev_w) / (2.0 * sqrt(2.0) * gridMap.getResolution()); //slope southeast
            // double dz_dnw = (elev_nw - elev_e) / (2.0 * sqrt(2.0) * gridMap.getResolution()); //slope northwest
            // double dz_dsw = (elev_sw - elev_e) / (2.0 * sqrt(2.0) * gridMap.getResolution()); //slope southwest
            // float dz_n=abs(elev-elev_n);
            // float dz_s=abs(elev-elev_s);
            // float dz_e=abs(elev-elev_e);
            // float dz_w=abs(elev-elev_w);
            // float dz_ne=abs(elev-elev_ne);
            // float dz_se=abs(elev-elev_se);
            // float dz_nw=abs(elev-elev_nw);
            // float dz_sw=abs(elev-elev_sw);
            // //CALCULATE THE ANGLES IN ALL DIRECTIONS
            // float ang_n = atan(dz_n/gridMap.getResolution());
            // float ang_s = atan(dz_s/gridMap.getResolution());
            // float ang_e = atan(dz_e/gridMap.getResolution());
            // float ang_w = atan(dz_w/gridMap.getResolution());
            // float ang_ne = atan(dz_ne/(gridMap.getResolution()*sqrt(2)));
            // float ang_se = atan(dz_se/(gridMap.getResolution()*sqrt(2)));
            // float ang_nw = atan(dz_ne/(gridMap.getResolution()*sqrt(2)));
            // float ang_sw = atan(dz_sw/(gridMap.getResolution()*sqrt(2)));
            // float slopes[8]={ang_n,ang_s,ang_e,ang_w,ang_ne, ang_se, ang_nw, ang_sw};
            //     float max_slope = slopes[0];
            //     int i;
            // Traverse array elements
            // from second and compare
            // every element with current max
            // float avg_slope=0;
            // float sum=0;
            // for (i = 1; i < 8; i++){
            //     sum = sum+slopes[i];
            // }
            // avg_slope=sum/8;
            // for (i = 1; i < 8; i++)
            //     if (slopes[i] > max_slope)
            //         max_slope = slopes[i];
            /// Compute the slope in the x direction.
            double dz_dx = (elev_e - elev_w) / (2.0 * gridMap.getResolution());
            // /Compute the slope in the y direction.
            double dz_dy = (elev_n - elev_s) / (2.0 * gridMap.getResolution());
            /// Compute the slope angle.
            double slope = atan(sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
            gridMap.at("traversability", index) = slope;
            // if (slope > max_slope) {
            //     max_slope = slope;
            // }
            // if (slope < min_slope) {
            //     min_slope = slope;
            // }
            // Determine the traversability of each grid cell based on its slope angle.
            // if (slope <= M_PI / 180.0 * max_slope_angle || slope >= M_PI / 180.0 * 40) {
            //     // traversability_map.at("traversability", index) = 1.0;
            //     gridMap.at("traversability", index) = 1.0;
            // } else {
            //     // traversability_map.at("traversability", index) = 0.0;
            //     gridMap.at("traversability", index) = 0.0;
            // }
        }
        // gridMap.add("traversability", traversability_map.get("traversability"));

        ///CREATE EMPTY IMAGE
        cv::Mat image;
        cv::Mat image_grey;
        // bool result = grid_map::GridMapCvConverter::toImage<float, 1>(gridMap, "slope", CV_8UC1, min_slope, max_slope, image);

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
    // ros::param::get("/my_integer", int_var);
    ///get rosparams

     
    // ROS_INFO("Int: %d, Float: %lf, String: %s", int_var, double_var, string_var.c_str());

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
    // pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud_raw(new pcl::PointCloud < pcl::PointXYZ > );
    pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ > );
    pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud_raw(new pcl::PointCloud < pcl::PointXYZ > );
    // pcl::io::loadPCDFile("/home/otto-testing/zaragoza/fullMapNODS.pcd", * cloud);
    // pcl::io::loadPCDFile("/home/otto-testing/posten_pcd/fullMapNODS.pcd", * cloud);
    pcl::io::loadPCDFile("/home/otto-testing/catkin_ws/src/window_to_work.pcd", * cloud);
    // pcl::io::loadPCDFile("/home/otto-testing/tom_tom/zara.pcd", * cloud);

    // pcl::io::loadPCDFile("/home/otto-testing/octomap.pcd.pcd", * cloud);


    //APPLY SOR TO REMOVE BAD POINTS ON SIDEWALK
    pcl::StatisticalOutlierRemoval < pcl::PointXYZ > sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1.3);
    sor.filter( * cloud);
    pcl::PointCloud < pcl::PointXYZ > ::Ptr temp_cloud(new pcl::PointCloud < pcl::PointXYZ > );
    pcl::CropBox < pcl::PointXYZ > temp_filter;
    temp_filter.setInputCloud(cloud);
    temp_filter.setMin(Eigen::Vector4f(-71, 224, -1000, 1.0));
    temp_filter.setMax(Eigen::Vector4f(-17, 271, 1000, 1.0));
        // temp_filter.setMin(Eigen::Vector4f(-600, -1213, -1000, 1.0));
    // temp_filter.setMax(Eigen::Vector4f(-130, -660, 1000, 1.0));
    temp_filter.filter( * temp_cloud);
    pcl::io::savePCDFileASCII("sor_new_window.pcd", * temp_cloud);

    // LOAD THE POSES
    pcl::PointCloud < PointTypePose > ::Ptr poses(new pcl::PointCloud < PointTypePose > );
    // pcl::io::loadPCDFile < PointTypePose > ("/home/otto-testing/posten_pcd/6D_pose.pcd", * poses);
    pcl::io::loadPCDFile < PointTypePose > ("/home/otto-testing/zaragoza/6D_pose.pcd", * poses);
    // pcl::io::loadPCDFile < PointTypePose > ("/home/otto-testing/tom_tom/6D_pose.pcd", * poses);
    std::cout << "DONE LOADING!" << std::endl;


    PointTypePose min, max;
    pcl::getMinMax3D( * poses, min, max);

    std::cout<<min.x<<"  "<<min.y<<"  "<<min.z<<std::endl;
    std::cout<<max.x<<"  "<<max.y<<"  "<<max.z<<std::endl;
    // float ransac_thresh = (max.z + min.z)/2;

    // pcl::SampleConsensusModelPerpendicularPlane < PointTypePose > ::Ptr pose_plane_model(new pcl::SampleConsensusModelPerpendicularPlane < PointTypePose > (poses));
    // const Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
    // pose_plane_model -> setAxis(axis);
    // pose_plane_model -> setEpsAngle(30*3.14/180);
    // // // Create a random sample consensus object with the model as input
    // pcl::RandomSampleConsensus < PointTypePose > ransac_poses(pose_plane_model);
    // ransac_poses.setDistanceThreshold(ransac_thresh);
    // // // Compute the model and extract the inliers
    // ransac_poses.computeModel();
    // Eigen::VectorXf coeffs;
    // ransac_poses.getModelCoefficients(coeffs);  //ax + by + cz + d = 0



    // float z_intercept = -coeffs[3] / coeffs[2];
    //OUTPUT CLOUD
    pcl::PointCloud < pcl::PointXYZ > ::Ptr out_cloud(new pcl::PointCloud < pcl::PointXYZ > );
    pcl::PointCloud < pcl::PointXYZ > ::Ptr out_segmented(new pcl::PointCloud < pcl::PointXYZ > );
    //CALCULATE THE AVERAGE POSE
    pcdWindows::PCDWindows myWindow; // create an object of the class
    PointTypePose averagePose = myWindow.calcAveragePose(poses); // call the member function using the object
    //CALCULATE CLOUD HEIGHT
    float cloudHeight = myWindow.calcCloudHeight(cloud, pcdWindows::marginY = 2);
    //CALCULATE CLOUD WIDTH
    float cloudWidth = myWindow.calcCloudWidth(cloud, pcdWindows::marginX = 2);
    //CALCULATE WINDOW PARAMETERS
    pcdWindows::windowParams window = myWindow.calcWindowParams(cloud, cloudHeight, cloudWidth, pcdWindows::windowWidth, pcdWindows::windowHeight);
    //CALCULATE MAX AND MIN PTS OF THE CLOUD
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D( * cloud, minPt, maxPt);
    float minX = minPt.x - pcdWindows::marginX;
    float maxX = maxPt.x + pcdWindows::marginX;
    float minY = minPt.y - pcdWindows::marginY;
    float maxY = maxPt.y + pcdWindows::marginY;
    //CREATE A CROPBOX TO CROP THE POINTS OF THE CLOUD THAT ARE LYING IN THE WINDOW 
    pcl::CropBox < pcl::PointXYZ > window_filter;
    window_filter.setInputCloud(cloud);
    pcl::PointCloud < pcl::PointNormal > ::Ptr obstacles_all(new pcl::PointCloud < pcl::PointNormal > );

    //SET GEOMETRY OF GRIDMAP FROM CLOUD MIN MAX
    grid_map::GridMap gridMap;
    gridMap.setFrameId("grid_map");
    float width = cloudWidth;
    float height = cloudHeight;
    float c_x = (maxX + minX) / 2;
    float c_y = (minY + maxY) / 2;
    gridMap.setGeometry(grid_map::Length(width, height), 0.05, grid_map::Position(c_x, c_y));

    //ADD LAYERS TO THE GRIDMAP
    gridMap.add("obstacles");
    // gridMap.add("occ_grid");
    gridMap.add("elevation");
    // gridMap.add("slope");
    gridMap.add("traversability");
    grid_map::Index index;

    // pcl::PointXYZ high;
    // cloud -> push_back(point_add);

    //LOOP THROUGH THE WINDOWS
    while (window.windowMaxY <= maxY) {
        while (window.windowMaxX <= maxX) {
            //OBSTACLE CLOUD
            pcl::PointCloud < pcl::PointNormal > ::Ptr obstacles(new pcl::PointCloud < pcl::PointNormal > );
            std::vector < int > inliers;
            pcl::PointCloud < pcl::PointXYZ > ::Ptr final_window(new pcl::PointCloud < pcl::PointXYZ > );
            //POINTS LYING IN THE CURRENT WINDOW WILL BE STORED HERE
            pcl::PointCloud < pcl::PointXYZ > ::Ptr filtered_cloud(new pcl::PointCloud < pcl::PointXYZ > );
            //SET THE BOUNDS TO APPLY FILTER
            window_filter.setMin(Eigen::Vector4f(window.windowMinX, window.windowMinY, -1000, 1.0));
            window_filter.setMax(Eigen::Vector4f(window.windowMaxX, window.windowMaxY, 1000, 1.0));
            // APPLY FILTER TO GET POINTS LYING IN THE CURRENT WINDOW
            window_filter.filter( * filtered_cloud);
            //FIND CENTER OF THE WINDOW
            window.window_center_x = (window.windowMaxX + window.windowMinX) / 2;
            window.window_center_y = (window.windowMaxY + window.windowMinY) / 2;
            //FIND POSE CLOSEST TO THE CENTER
            PointTypePose closestPose = myWindow.findClosestPose(window.window_center_x, window.window_center_y, poses);
            //PERFORM Z CUT ON CURRENT WINDOW
            pcl::PointCloud < pcl::PointXYZ > ::Ptr z_cut_cloud(new pcl::PointCloud < pcl::PointXYZ > );
            float closestPose_z = closestPose.z;
            // z_cut_cloud=filtered_cloud;
            z_cut_cloud = myWindow.performZCutOnWindow(filtered_cloud, z_cut_cloud, window, -200, 2, closestPose_z);
            // z_cut_cloud = myWindow.performZCutOnWindow(filtered_cloud, z_cut_cloud, window, -200000, 0, 1);

            //GET OBSTACLES
            std::cout << "num_pts :" << z_cut_cloud -> size() << std::endl;
            if (z_cut_cloud -> size() > 300) {
                int temp = myWindow.get_obstacles(z_cut_cloud, obstacles, obstacles_all, 0.2, 2, 0.25, 0.2, window.windowNumber);
            }
            for (size_t i = 0; i < z_cut_cloud -> size(); ++i) {
                pcl::PointXYZ point_add = z_cut_cloud -> points[i];
                out_cloud -> push_back(point_add);
            }
            std::vector < pcl::PointCloud < pcl::PointXYZ > ::Ptr > planes;
            // if (filtered_cloud->size()>100){
            // int temp = myWindow.segment_road_sidewalk(filtered_cloud, planes, 5, 0.007, window.windowNumber);}


            //ax + by + cz + d = 0
            //z=(-d-by-ax)/c
            // for (const auto & point: z_cut_cloud -> points) {
            //     if (point.z > -10 && point.z < 10) {
            //         if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            //         PointTypePose nearest_pose_to_point = myWindow.findClosestPose(point.x, point.y, poses);
            //         // gridMap.at("elevation", index) = point.z;//-nearest_pose_to_point.z;
            //         //ax+by+cz+d=0
            //         //z=-(d+ax+by)/c
            //         float aa = coeffs[0];
            //         float bb = coeffs[1];
            //         float cc = coeffs[2];
            //         float dd = coeffs[3];
            //         gridMap.at("elevation", index) = point.z - (-((dd+(aa*point.x)+(bb*point.y))/cc));//math

            //     }
            // }
            int num = window.windowNumber;
            // myWindow.get_obstacles(z_rgb_cloud,num, 0.2,2,0.25,0.2);
            std::cout << "windowNumber :" << window.windowNumber << "/" << window.numWindowX * window.numWindowY << "    " << "windowIndexX :" << window.windowIndexX << "    " << "windowIndexY :" << window.windowIndexY << std::endl;
            //UPDATE WINDOW PARAMS TO MOVE TO NEXT WINDOW
            window.windowMaxX = window.windowMaxX + window.windowWidth;
            window.windowMinX = window.windowMinX + window.windowWidth;
            window.windowNumber = window.windowNumber + 1;
            window.windowIndexX += 1;
        }
        //UPDATE WINDOW PARAMS TO MOVE TO NEXT WINDOW
        window.windowMaxY = window.windowMaxY + window.windowHeight;
        window.windowMinY = window.windowMinY + window.windowHeight;
        window.windowMaxX = minX + window.windowWidth;
        window.windowMinX = minX;
        window.windowIndexY += 1;
        window.windowIndexX = 0;
    }
    //SAVE CLOUD AFTER PERFORMING Z CUT
    pcl::io::savePCDFileASCII("processed_new_new.pcd", * out_cloud);
    obstacles_all -> width = obstacles_all -> size();
    obstacles_all -> height = 1;
    obstacles_all -> is_dense = true;
    // pcl::io::savePCDFileASCII("obstacles_all.pcd", * obstacles_all);

    // float x1=-0.484421;
    // float y1=-1.263416;
    // float z1=-0.005683;
    // float x2=206.648123;
    // float y2=464.898895;
    // float z2=4.808763;
    // float x3=386.618469;
    // float y3=239/149780;
    // float z3=19.715012;
    // float a1 = x2 - x1;
    // float b1 = y2 - y1;
    // float c1 = z2 - z1;
    // float a2 = x3 - x1;
    // float b2 = y3 - y1;
    // float c2 = z3 - z1;
    // float aa = b1 * c2 - b2 * c1;
    // float bb = a2 * c1 - a1 * c2;
    // float cc = a1 * b2 - b1 * a2;
    //float dd = (- aa * x1 - bb * y1 - cc * z1);
            // for (const auto & point: out_cloud -> points) {
            //     if (point.z > -2) {
            //         if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            //         gridMap.at("elevation", index) = -2+point.z;//- (-((-dd+(aa*point.x)+(bb*point.y))/cc));//math
            //     }
            //     if (point.z < -2) {
            //         if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            //         gridMap.at("elevation", index) = -2;//- (-((-dd+(aa*point.x)+(bb*point.y))/cc));//math
            //     }
            // }

        //CONVERT THE POINT CLOUD TO ELEVATION
        for (const auto & point: out_cloud -> points) {
        if (point.z > -200 && point.z < 200) {
            if (!gridMap.getIndex(grid_map::Position(point.x, point.y), index)) continue;
            gridMap.at("elevation", index) = point.z;
        }
    }
    // std::vector < grid_map::Index > indices;
    // for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    //       if (std::isnan(gridMap.at("elevation", *it))) {
    //     indices.push_back( * it);}
    // }
    // // Shuffle the vector of indices to iterate over them in random order
    // //the iteration is done in a random order because when we are interpolating the cells while iterating linearly, it causes a smudge as it goes ahead
    // index.x() = 0;
    // index.y() = 0;
    // long int a = 0;
    // const auto size = gridMap.getSize();
    // std::random_device rd;
    // std::default_random_engine rng(rd());
    // std::shuffle(indices.begin(), indices.end(), rng);
    // // Iterate over the shuffled vector of indices
    // for (const auto & index: indices) {
    //     a = a + 1;
    //     std::cout << (100 * a) / (size.x() * size.y()) << " percent complete" << std::endl;
    //     float & data = gridMap.at("elevation", index);
    //     // Check if the current value is NaN
    //     if (std::isnan(data)) {
    //         // Define the size of the window to use
    //         const int windowSize = 2; // Change this to adjust the window size.
    //         float sum = 0.0;
    //         long int count = 0;
    //         // Iterate over neighboring cells within the window
    //         for (long int i = -windowSize; i <= windowSize; ++i) {
    //             for (long int j = -windowSize; j <= windowSize; ++j) {
    //                 const auto neighborIndex = grid_map::Index(index.x() + i, index.y() + j);
    //                 if (neighborIndex.x() >= 0 && neighborIndex.y() >= 0 && neighborIndex.x() < size.x() && neighborIndex.y() < size.y()) {
    //                     const auto neighborData = gridMap.at("elevation", neighborIndex);
    //                     // Check that the neighbor is not NaN
    //                     if (!std::isnan(neighborData)) {
    //                         sum += neighborData;
    //                         ++count;
    //                     }
    //                 }
    //             }
    //         }
    //         if (count > 0) {
    //             // Update the current value with the average of the neighboring non-NaN values
    //             data = sum / count;
    //         }
    //     }
    // }


// Find all NaN values in the grid and their indices
std::vector<grid_map::Index> nanIndices;
const auto size = gridMap.getSize();
for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    const auto index = *it;
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
for (const auto & index : nanIndices) {
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

// z_cut_cloud->clear();
// obstacles_all->clear();

    //CLEAR TO SAVE RAM
    nanIndices.clear();
    cloud->clear();

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
        out_cloud->clear();

    //DEFIN THE MESSAGES AND 
    grid_map_msgs::GridMap gridMapMessage;
    // std::cout<<"731"<<std::endl;
    nav_msgs::OccupancyGrid obstacles_occ;
    nav_msgs::OccupancyGrid terrain_roughness;
    nav_msgs::OccupancyGrid obstacle_slope;
    nav_msgs::OccupancyGrid elevation_occ;

    // std::cout<<"735"<<std::endl;
    // terrain_roughness=myWindow.calculate_terrain_roughness(gridMap,10);
    // width=cloudWidth;
    // height=cloudHeight;
    // c_x=(maxX+minX)/2;
    // c_y=(minY+maxY)/2;
    // gridMap = myWindow.calculate_terrain_roughness(gridMap);

    
    float max_slope = -9999999999.0;
        float min_slope = 9999999999.0;
        //ITERATE THROUGH THE CELLS OF ELEVATION MAP
        for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
            // Get the index of the current grid cell.
            const auto index = * iterator;
            // Get the elevation values at the neighboring cells.
            //CALCULATE SLOPE WITH RESPECT TO NEIGHBORING CELLS
            double elev_n = gridMap.at("elevation", index + grid_map::Index(0, 2)); //north (0,2) MEANS TWO CELLS UP
            double elev_s = gridMap.at("elevation", index + grid_map::Index(0, -2)); //south
            double elev_e = gridMap.at("elevation", index + grid_map::Index(2, 0)); //east
            double elev_w = gridMap.at("elevation", index + grid_map::Index(-2, 0)); //west
            double elev = gridMap.at("elevation", index);
            double dz_dx = (elev_e - elev_w) / (2.0 * gridMap.getResolution());
            // Compute the slope in the y direction.
            double dz_dy = (elev_n - elev_s) / (2.0 * gridMap.getResolution());
            // Compute the slope angle.
            double slope = atan(sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
            //FILL TRAVERSABILITY LAYER WITH THE CALCULATED SLOPE
            gridMap.at("traversability", index) = slope;
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
        grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "traversability", -10.0, 10.0, terrain_roughness);
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
