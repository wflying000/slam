#include<fstream>
#include<vector>
#inlude<map>
using namespace std;

#include<Eigen/Core>
#include<Eigen/Geometry>

#include<opencv2/core/core.hpp>
#include<opencv2/hihgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/core/eigen.hpp>

#include<pcl/io/pcl_io.h>
#include<pcl/point_types.h>
#include<pcl/common/transforms.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct CAMERA_INTRINSIC_PARAMETERS{
  double cx, cy, fx, fy, scale;
};

struct FRAME{
  int frameID;
  cv::Mat rgb, depth;
  cv::Mat desp;
  vector<cv::KeyPoint> kp;
};

struct RESULT_OF_PNP{
  cv::Mat rvec, tvec; //旋转向量和位移向量
  int inliers;
};


PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera);  //图像转换为点云

cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);  //像素坐标转换为相机坐标

void computeKeyPointAndDesp(FRAME &frame, string detector, string descriptor);  //计算关键点和描述子

RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera);  //计算两帧之间的运动

Eigen::Isometry3d cvMat2Eigen(cv::Mat &rvec, cv::Mat &tvec);



