#include "icp.h"
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include<sstream>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;

string int2str(int num)
{
    stringstream ss;
    ss<<num;
    return ss.str();
}

int main(int argc,char* argv[])
{
    string num1,num2;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    vector<Eigen::Matrix4f> TRMatrix;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGB>);
    string filePath=argv[1];
    int startNum=atoi(argv[2]);
    int dset=atoi(argv[3]);
    int endNum=atoi(argv[4]);
    for(int i=startNum;i<endNum;i=i+dset)
    {
        num1=int2str(1);
        num2=int2str(i+dset);
        string RFile=filePath+"/R1-"+num2+".txt";
        string tFile=filePath+"/T1-"+num2+".txt";

        string pathImg1=filePath+"/rgb_"+num1+".png";
        string pathImg2=filePath+"/rgb_"+num2+".png";
        string pathDepth1=filePath+"/depth_"+num1+".png";
        string pathDepth2=filePath+"/depth_"+num2+".png";

        cv::Mat depth_image1=imread(pathDepth1,CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat depth_image2=imread(pathDepth2,CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat rgb_image1=imread(pathImg1);
        cv::Mat rgb_image2=imread(pathImg2);//read the image


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1=depth2cloud(rgb_image1,depth_image1);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2=depth2cloud(rgb_image2,depth_image2);//get the pointcloud

        if(i==startNum)
            clouds.push_back(cloud1);

        Eigen::Matrix4f transformationINSERT;//define the transformation matrix



        float M[4][4];
        loadRTMatrix(RFile.c_str(),tFile.c_str(),M);
        cv::Mat transformMatrix(4,4,CV_32FC1,M);
        cv2eigen(transformMatrix,transformationINSERT);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrot(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*cloud2,*cloudrot,transformationINSERT);
        TRMatrix.push_back(transformationINSERT);
        clouds.push_back(cloudrot);

    }


    for(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it=clouds.begin();it!=clouds.end();it++)
        *cloud_final+=*(*it);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_finalDownsample(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_final);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_finalDownsample);
    pcl::io::savePCDFileASCII("final_cloud.pcd",*cloud_final);

    viewer = rgbVis(cloud_original,cloud_finalDownsample,1);
    viewer->resetCamera ();
    viewer->spin ();
    string fileName=filePath+"/result.png";
   //viewer->saveScreenshot(fileName);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}
