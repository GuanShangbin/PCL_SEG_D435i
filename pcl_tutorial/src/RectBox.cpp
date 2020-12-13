#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//滤波的头文件
#include <pcl/filters/voxel_grid.h>
//申明发布器

#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


ros::Publisher pub;
//回调函数

// int Box(pcl::PointCloud<PointType>::Ptr cloud)
// {   // pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//     //pcl::io::loadPCDFile("15967bda.pcd", *cloud);

//     Eigen::Vector4f pcaCentroid;
//     pcl::compute3DCentroid(*cloud, pcaCentroid);
//     Eigen::Matrix3f covariance;
//     pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//     Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//     Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
//     eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
//     eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
//     eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

//     std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
//     std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
//     std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
//     /*
//     // 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PCA<pcl::PointXYZ> pca;
//     pca.setInputCloud(cloudSegmented);
//     pca.project(*cloudSegmented, *cloudPCAprojection);
//     std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
//     std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
//     */
//     Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//     Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
//     tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
//     tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
//     tm_inv = tm.inverse();

//     std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
//     std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

//     pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
//     pcl::transformPointCloud(*cloud, *transformedCloud, tm);

//     PointType min_p1, max_p1;   //点云的最大值与最小值点
//     Eigen::Vector3f c1, c;
//     pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
//     c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

//     std::cout << "型心c1(3x1):\n" << c1 << std::endl;

//     Eigen::Affine3f tm_inv_aff(tm_inv);
//     pcl::transformPoint(c1, c, tm_inv_aff);

//     Eigen::Vector3f whd, whd1;
//     whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
//     whd = whd1;
//     float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

//     std::cout << "width1=" << whd1(0) << endl;
//     std::cout << "heght1=" << whd1(1) << endl;
//     std::cout << "depth1=" << whd1(2) << endl;
//     std::cout << "scale1=" << sc1 << endl;

//     const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
//     const Eigen::Vector3f    bboxT1(c1);

//     const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
//     const Eigen::Vector3f    bboxT(c);


//     //变换到原点的点云主方向
//     PointType op;
//     op.x = 0.0;
//     op.y = 0.0;
//     op.z = 0.0;
//     Eigen::Vector3f px, py, pz;
//     Eigen::Affine3f tm_aff(tm);
//     pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
//     pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
//     pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
//     PointType pcaX;
//     pcaX.x = sc1 * px(0);
//     pcaX.y = sc1 * px(1);
//     pcaX.z = sc1 * px(2);
//     PointType pcaY;
//     pcaY.x = sc1 * py(0);
//     pcaY.y = sc1 * py(1);
//     pcaY.z = sc1 * py(2);
//     PointType pcaZ;
//     pcaZ.x = sc1 * pz(0);
//     pcaZ.y = sc1 * pz(1);
//     pcaZ.z = sc1 * pz(2);

//     //visualization
//     pcl::visualization::PCLVisualizer viewer;


//     pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 255, 0); //输入的初始点云相关
//     viewer.addPointCloud(cloud, color_handler, "cloud");
//     viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
//     viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
//     viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");


//     viewer.addCoordinateSystem(0.5f*sc1);
//     viewer.setBackgroundColor(0.0, 0.0, 0.0);
//     while (!viewer.wasStopped())
//     {
//         viewer.spinOnce();
//     }

//     return 0;
// }

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
// 声明存储原始数据与滤波后的数据的点云的格式
// Container for original & filtered data
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PCLPointCloud2 cloud_filtered;//存储滤波后的数据格式

// Convert to PCL data type。转化为PCL中的点云的数据格式
pcl_conversions::toPCL(*cloud_msg, *cloud);

// Perform the actual filtering进行一个滤波处理
pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建滤波对象
sor.setInputCloud (cloudPtr); //设置输入的滤波，将需要过滤的点云给滤波对象
sor.setLeafSize (0.1, 0.1, 0.1); //设置滤波时创建的体素大小为1cm立方体
sor.filter (cloud_filtered);//执行滤波处理，存储输出cloud_filtered

// Convert to ROS data type// 再将滤波后的点云的数据格式转换为ROS下的数据格式发布出去
sensor_msgs::PointCloud2 output;//声明的输出的点云的格式
pcl_conversions::moveFromPCL(cloud_filtered, output);//第一个参数是输入，后面的是输出

// Publish the data
// Box(output);
pub.publish (output);
}

int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");//声明节点的名称
ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
// 为接受点云数据创建一个订阅节点
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
//创建ROS的发布节点
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

// Spin
// 回调
ros::spin ();
}