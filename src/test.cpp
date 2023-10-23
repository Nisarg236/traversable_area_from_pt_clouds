#include <iostream>
#include <cmath>

float getDistance(float x_1, float y_1, float x_2, float y_2){
    float distance = std::sqrt(std::pow(x_1 - x_2, 2) + std::pow(y_1 - y_2, 2));
    return distance;
}

//perform differnce of normals based segmentation to detect obstacles
//find the pose closest to the center of the window
PointTypePose findClosestPose(float window_center_x, float window_center_y, pcl::PointCloud<PointTypePose>::Ptr poses){
    float minDistance = std::numeric_limits<float>::max();
    PointTypePose closestPose;
    for (size_t i = 0; i < filtered_poses->size(); ++i)
        {
        const PointTypePose &p = (*filtered_poses)[i];
        float distance = std::sqrt(std::pow(p.x - window_center_x, 2) + std::pow(p.y - window_center_y, 2));
        if (distance < minDistance)
            {
            minDistance = distance;
            closestPose = p;
            }
        }
        return closestPose;
}

//find poses lying in this window
pcl::PointCloud<PointTypePose>::Ptr findPosesInWindow(float windowMinX, float windowMaxX, float windowMinY, float windowMaxy, pcl::PointCloud<PointTypePose>::Ptr poses){  
    pcl::PointCloud<PointTypePose>::Ptr filtered_poses(new pcl::PointCloud < PointTypePose>);
    for (size_t i = 0; i < poses->size(); ++i)
       {
        PointTypePose pose = poses->points[i];
       if (pose.x >= windowMinX && pose.x <= windowMaxX && pose.y >= windowMinY && pose.y <= windowMaxY)
         {
       filtered_poses->push_back(pose);
       }
       }
      return filtered_poses;
}

windowParams calcWindowParams(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cloudHeight, float cloudWidth, float windowWidth, float windowHeight){
    windowParams window;
    window.numWindowY=floor(cloudHeight / windowHeight);
    if (window.numWindowY == 0)
    {
        window.numWindowY = 1;
    }
    window.windowHeight = cloudHeight / window.numWindowY;
    std::cout << "windowHeight :" << window.windowHeight << "    " << "numWindowY :" << window.numWindowY << std::endl;
    
    window.numWindowX=floor(cloudWidth / windowWidth);
    if (window.numWindowX == 0)
    {
        window.numWindowX = 1;
    }
    window.windowWidth = cloudWidth / windowWidth;
    std::cout << "windowWidth :" << window.windowWidth << "    " << "numWindowX :" << window.numWindowX << std::endl;
    
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(cloud, minPt, maxPt);
    float minX = minPt.X - marginX;
    float maxX = maxPt.X + marginX;
    window.windowMinX = minX;
    window.windowMaxX = minX + window.windowWidth;
    window.windowMinY = minY;
    window.windowMaxY = minY + window.windowHeight;
    window.window_center_x = (window.windowMinX+window.windowMaxX)/2;
    window.window_center_y = (window.windowMinY+window.windowMaxY)/2
    return window
}

float calcCloudHeight (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float marginY, float marginX){
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(cloud, minPt, maxPt);
    float minX = minPt.X - marginX;
    float maxX = maxPt.X + marginX;
    float cloudHeight = maxX - minX;
    return cloudHeight;
}
float calcCloudWidth (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float marginY, float marginX){
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(cloud, minPt, maxPt);
    float minY = minPt.Y - marginY;
    float maxY = maxPt.Y + marginY;
    float cloudWidth = maxY - minY;
    return cloudWidth;
}


PointTypePose calcAveragePose(pcl::PointCloud<PointTypePose>::Ptr poses){
    PointTypePose averagePose;
    for (size_t i = 0; i < poses->size(); ++i){
            PointTypePose point = poses->points[i];
            averagePose.roll += point.roll;
            averagePose.pitch += point.pitch;
            averagePose.yaw += point.yaw;
            averagePose.x += point.x;
            averagePose.y += point.y;
            averagePose.z += point.z;
        }
    size_t num_poses = poses->size();
    averages.roll /= num_poses;
    averages.pitch /= num_poses;
    averages.yaw /= num_poses;
    averages.x /= num_poses;
    averages.y /= num_poses;
    averages.z /= num_poses;

    return averagePose;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr performZCutOnWimdow(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn, float maximum_Z, float minimum_z, windowParams window, PointTypePose closestPose){
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_cut_window(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ > zcut;
    zcut.setInputCloud(cloudIn);
    zcut.setMin(Eigen::Vector4f(window.windowMinX, window.windowMinY, closestPose.z+minimum_z, 1.0));
    zcut.setMax(Eigen::Vector4f(window.windowMaxX, window.windowMaxY, closestPose.z+maximum_z, 1.0));
    zcut.filter(*z_cut_window;
    return z_cut_window;
}


pcl::PointXYZ minPt, maxPt;
float windowWidth = 11;
float windowHeight = 11;
float numWindowX;	// calculate
float numWindowY;	// calculate
float marginY = 1;
float marginX = 1;
float theta = M_PI / 2;

// pcdWindows::pcdWindows(/*args */)
// {
//     windowWidth = 5;
//     windowHeight = 5;
//     numWindowX;	// calculate
//     numWindowY;	// calculate
//     marginY = 1;
//     marginX = 1;
//     theta = M_PI / 2;
// }

// pcdWindows::~pcdWindows()
// {
//     ROS_ERROR("Exiting from pcdWindows class");
// }

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;	// preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// make sure our new allocators are aligned
}-

EIGEN_ALIGN16;	// enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

int main()
{



   	// pcdWindows new_pcd_windos()
   	// Load PCD file
    std::cout << "LOADING THE FILES..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/otto-testing/Desktop/Assotech_map/fullMapDS.pcd", *cloud);

   	// Load Poses
    pcl::PointCloud<PointTypePose>::Ptr poses(new pcl::PointCloud < PointTypePose>);
    pcl::io::loadPCDFile<PointTypePose> ("/home/otto-testing/Desktop/Assotech_map/6D_pose.pcd", *poses);
    std::cout << "DONE LOADING!" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    PointTypePose average_pose = calcAveragePose(pcl::PointCloud<PointTypePose>::Ptr poses)
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    float cloudHeight= calcCloudHeight (cloud, float marginY=2, float marginX=2);
    float cloudWidth= calcCloudHeight (cloud, float marginY2, float marginX=2);
    windowParams window = calcWindowParams(*cloud, cloudHeight, cloudWidth, windowWidth, windowHeight);

   	// Create window filter
    pcl::CropBox<pcl::PointXYZ > window_filter;
    window_filter.setInputCloud(cloud);

   	// Start iterating through the windows
    int windowNumber = 1;
    int windowIndexX = 0;
    int windowIndexY = 0;
    while (windowMaxY <= maxY)
    {
        while (windowMaxX <= maxX)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr z_cut_window(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<PointTypePose>::Ptr filtered_poses(new pcl::PointCloud < PointTypePose>);
            PointTypePose closestPose;

           	// Set the filter bounds
            window_filter.setMin(Eigen::Vector4f(windowMinX, windowMinY, min_z, 1.0));
            window_filter.setMax(Eigen::Vector4f(windowMaxX, windowMaxY, max_z, 1.0));
           	// Apply filter to extract points in the window
            window_filter.filter(*filtered_cloud);
            std::cout << "num_pts :" << filtered_cloud->size() << std::endl;
           	//find center of the window
            float window_center_x = (windowMaxX + windowMinX) / 2;
            float window_center_y = (windowMaxY + windowMinY) / 2;

            // find closest pose to window center
            cloasestPose = findClosestPose(window.window_center_x,window.window_center_y, poses);

            //now perform z cut using this pose as reference
            z_cut_window = performZCutOnWimdow(filtered_cloud, 0.0, -1000, window.windowMinX, window.windowMinY, window.windowMaxX, window.windowMaxY, closestPose);

            //add points of this cloud to processed_cloud
            for (size_t i = 0; i < z_cut_window->size(); ++i)
            {
                pcl::PointXYZ point_add = z_cut_window->points[i];
                processed_cloud->push_back(point_add);
            }
            windowMaxX = windowMaxX + windowWidth;
            windowMinX = windowMinX + windowWidth;
            std::cout << "windowNumber :" << windowNumber << "    " <<
                "windowIndexX :" << windowIndexX << "    " <<
                "windowIndexY :" << windowIndexY << std::endl;
            windowNumber = windowNumber + 1;
            windowIndexX += 1;
        }

        windowMaxY = windowMaxY + windowHeight;
        windowMinY = windowMinY + windowHeight;
        windowMaxX = minX + windowWidth;
        windowMinX = minX;
        windowIndexY += 1;
        windowIndexX = 0;
    }

    pcl::io::savePCDFileASCII("processed1.pcd", *processed_cloud);

    return 0;
}