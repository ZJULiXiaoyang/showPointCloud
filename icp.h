#define FOCAL 575.0f
#define CX 319.5f
#define CY 239.5f
#define HEIGHT 480
#define WIDTH 640
#define depth_style 1000.0f
#include<string>
#include<vector>
using namespace std;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<opencv2/opencv.hpp>



pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2cloud( cv::Mat rgb_image, cv::Mat depth_image ) ;//get the pointcloud from depth image and rgb image

void loadRTMatrix(const char* RFile,const char* tFile,float M[][4]);

void findDepth( vector<cv::KeyPoint>& keypoint, vector<cv::Point3f>& keypoint3d, cv::Mat& depth_image, vector<cv::Point2f>& kp);//get the pointcloud of keypoint

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ,int );

void viewerOneOff (pcl::visualization::PCLVisualizer& );



