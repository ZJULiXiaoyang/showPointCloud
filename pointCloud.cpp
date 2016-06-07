
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include "icp.h"
using namespace std;
using namespace cv;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2cloud( cv::Mat rgb_image, cv::Mat depth_image ) {
    float f = 570.3;
    float cx = 320.0, cy = 240.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr( new pcl::PointCloud<pcl::PointXYZRGB> () );
    cloud_ptr->width  = rgb_image.cols;
    cloud_ptr->height = rgb_image.rows;
    cloud_ptr->is_dense = false;

    for ( int y = 0; y < rgb_image.rows; ++ y ) {
        for ( int x = 0; x < rgb_image.cols; ++ x ) {
            pcl::PointXYZRGB pt;
            pt.z = depth_image.at<unsigned short>(y, x)/1000.0;
            pt.x = (x-cx)*pt.z/f;
            pt.y = (y-cy)*pt.z/f;
            pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
            pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
            pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];

            float badPoint=std::numeric_limits<float>::quiet_NaN();
            if ( depth_image.at<unsigned short>(y, x) == 0 ) {
                pt.x = badPoint;
                pt.y = badPoint;
                pt.z = badPoint;
            }
            cloud_ptr->points.push_back( pt );

        }
    }
    return cloud_ptr;
}

void findDepth( vector<cv::KeyPoint>& keypoint, vector<cv::Point3f>& keypoint3d, cv::Mat& depth_image, vector<cv::Point2f>& kp)
{

    for (vector<cv::KeyPoint>::iterator iter = keypoint.begin(); iter < keypoint.end(); ++iter)
    {
        float u = iter->pt.x;
        float v = iter->pt.y;

        cv::Point2f p2(u,v);
        kp.push_back(p2);
        unsigned short d = depth_image.at<unsigned short>((int)v,(int)u);

        float z = d/depth_style;
        float x = (u-CX)*z/FOCAL;
        float y = (v-CY)*z/FOCAL;

        cv::Point3f p3(x,y,z);
        keypoint3d.push_back(p3);
    }
}

void loadRTMatrix(const char* RFile,const char* tFile,float M[][4])
{
        fstream fs;
        fs.open(RFile);
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                float tmp;
                fs>>tmp;
                M[i][j]=tmp;
            }
        }
        fs.close();
        fs.open(tFile);
        for(int i=0;i<3;i++)
        {
            float tmp;
            fs>>tmp;
            M[i][3]=tmp;
        }
        fs.close();
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,int num)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    int vp_1=1,vp_2=2;
    viewer->setBackgroundColor (0, 0, 0);
    if(num==2)
    {
        viewer->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
        viewer->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, rgb1, "target cloud",vp_1);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "final cloud",vp_2);
    }else
    {
        viewer->createViewPort (0.0, 0, 1.0, 1.0, vp_2);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "final cloud",vp_2);
    }
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    char k=cv::waitKey();
    if(k=='s')
        viewer.saveScreenshot("result.png");
    std::cout << "Image has been save as" << std::endl;
}
