// PCL lib Functions for processing point clouds 

#include "pcl_seg/processPointClouds.h"

// using templates so also include .cpp to help linker
#include "cluster3d.cpp"
#include "ransac3d.cpp"


using namespace lidar_obstacle_detection;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud) { std::cout << cloud->points.size() << std::endl; }

template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol) {
    // Count time
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));

    int num_points = cloud->points.size();
    auto cloud_points = cloud->points;
    Ransac<PointT> RansacSeg(maxIterations, distanceTol, num_points);

    // Get inliers from RANSAC implementation
    // 随机抽样一致算法，找到最多的局内点，这些点符合某个平面，该平面就是地面
    std::unordered_set<int> inliersResult = RansacSeg.Ransac3d(cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane ransac-segment took " << elapsedTime.count() << " milliseconds" << std::endl;

    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());

    for (int i = 0; i < num_points; i++) {
        PointT pt = cloud_points[i];
        if (inliersResult.count(i)) {
            in_plane->points.push_back(pt);        //in_plane记录在平面内的点
        } else {
            out_plane->points.push_back(pt);         //out_plane记录在平面外的点
        }
    }
    return std::pair<PtCdtr<PointT>, PtCdtr<PointT>>(out_plane, in_plane);
}

// 带有姿态角信息的平面约束Ransac
template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
RansacSegmentPlaneWithPose(PtCdtr<PointT> cloud, int maxIterations, float distanceTol, Eigen::Vector3d pose_data)
{
    // Count time
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));

    int num_points = cloud->points.size();
    auto cloud_points = cloud->points;
    Ransac<PointT> RansacSeg(maxIterations, distanceTol, num_points);

    // Get inliers from RANSAC implementation
    // 随机抽样一致算法，找到最多的局内点，这些点符合某个平面，该平面就是地面
    std::unordered_set<int> inliersResult = RansacSeg.Ransac3d(cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane ransac-segment took " << elapsedTime.count() << " milliseconds" << std::endl;

    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());

    for (int i = 0; i < num_points; i++) {
        PointT pt = cloud_points[i];
        if (inliersResult.count(i)) {
            in_plane->points.push_back(pt);        //in_plane记录在平面内的点
        } else {
            out_plane->points.push_back(pt);         //out_plane记录在平面外的点
        }
    }
    return std::pair<PtCdtr<PointT>, PtCdtr<PointT>>(out_plane, in_plane);
}


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::FilterCloud(PtCdtr<PointT> cloud, float voxelsize) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object: downsample the dataset using a leaf size of .2m
    //体素网格滤波器降采样，在每个体素网格内只留一个点
    pcl::VoxelGrid<PointT> vg;
    PtCdtr<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxelsize, voxelsize, voxelsize);    //设置体素网格大小，越大则采样数越少
    vg.filter(*cloudFiltered);          //滤波

    //显示时间消耗
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    //return cloud;
    return cloudFiltered;
}

template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::EuclideanClustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize,
                                                int maxSize) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // std::vector<PtCdtr<PointT>> clusters;

    ClusterPts<PointT> clusterPoints(cloud->points.size(), clusterTolerance, minSize, maxSize);

    //执行欧式聚类，clusters保存每一类聚类中的所有点
    std::vector<PtCdtr<PointT>> clusters = clusterPoints.EuclidCluster(cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "KDTree clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}


// template<typename PointT>
// Box ProcessPointClouds<PointT>::BoundingBox(PtCdtr<PointT> cluster) {

//     // Find bounding box for one of the clusters
//     PointT minPoint, maxPoint;
//     pcl::getMinMax3D(*cluster, minPoint, maxPoint);

//     Box box;
//     box.x_min = minPoint.x;
//     box.y_min = minPoint.y;
//     box.z_min = minPoint.z;
//     box.x_max = maxPoint.x;
//     box.y_max = maxPoint.y;
//     box.z_max = maxPoint.z;

//     return box;
// }


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(PtCdtr<PointT> cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file) {

    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


//获取PCD数据文件夹下的所有PCD文件路径
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    //dataPath是目录，表示head迭代器; {}表示建立一个end迭代器
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    //对路径按名称从小到大的顺序进行排序
    sort(paths.begin(), paths.end());

    return paths;

}