//
// Created by hyin on 2020/3/25.
//

#include "pcl_seg/ransac3d.h"
using namespace lidar_obstacle_detection;

template<typename PointT>
Ransac<PointT>::~Ransac() {}

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3d(PtCdtr<PointT> cloud) {
    std::unordered_set<int> inliersResult; // unordered_set element has been unique
    // For max iterations
    while (maxIterations--) {
        std::unordered_set<int> inliers;        //unordered_set内部的元素无序，通过hash表进行快速访问
        
        //随机在点云中选取3个初始点作为局内点
        while (inliers.size() < 3) {
            inliers.insert(rand()%num_points);
        }

        // TO define plane, need 3 points
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliers.begin();     //返回从inliers第一个元素开始的迭代器itelator

        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        
        // Calulate plane coefficient
        // 求取拟合平面的系数
        float a, b, c, d, sqrt_abc;
        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);
        sqrt_abc = sqrt(a * a + b * b + c * c);

        // 求非局内点到拟合平面的距离
        for (int ind = 0; ind < num_points; ind++) {
            if (inliers.count(ind) > 0) { // count只返回1和0，若inliers内有ind则返回1,否则0
                continue;       // 跳过已存在的局内点
            }
            PointT point = cloud->points[ind];
            float x = point.x;
            float y = point.y;
            float z = point.z;
            float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc; // calculate the distance between other points and plane

            if (dist < distanceTol) {       // 小于阈值的则加入局内点
                inliers.insert(ind);
            }
            if (inliers.size() > inliersResult.size()) {       // 局内点的数量大于已存在的最好拟合数量，则更新最好的拟合局内点集
                inliersResult = inliers;

            }
        }
    }
    return inliersResult;
}

