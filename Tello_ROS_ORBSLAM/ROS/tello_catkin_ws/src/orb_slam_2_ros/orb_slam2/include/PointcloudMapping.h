// PointcloudMapping.h
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>
#include<pcl/conversions.h>
#include<pcl/ros/conversions.h>
#include<ros/ros.h>

// #include<octomap_msgs.h>
// #include<octomap_msgs>


#include "KeyFrame.h"
#include "Converter.h"
#include"PointCloud.h"
#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H


typedef Eigen::Matrix<double, 6, 1> Vector6d;


namespace ORB_SLAM2 {

class Converter;
class KeyFrame;
// typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointCloud<PointT> mcPointCloud;
class PointCloudMapping {
public:
typedef pcl::PointXYZRGB PointT; // A point structure representing Euclidean xyz coordinates, and the RGB color.
typedef pcl::PointCloud<PointT> PointCloud;
    public:
        PointCloudMapping(double resolution,double meank_,double thresh_);
        void save();
        ~PointCloudMapping();
        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs ); // 传入的深度图像的深度值单位已经是m
        void requestFinish();
        int loopcount = 0;
        bool isFinished();
        void shutdown();
        bool cloudbusy;
        bool loopbusy;
        void updatecloud();
        void getGlobalCloudMap(PointCloud::Ptr &outputMap);  //do not use
        PointCloud::Ptr getGlobalMap();
        std::mutex mPointCloudMtx;
    private:
        void showPointCloud();
        void generatePointCloud(const cv::Mat& imRGB, const cv::Mat& imD, const cv::Mat& pose, int nId); 

        double mCx, mCy, mFx, mFy, mResolution;
        
        std::shared_ptr<std::thread>  viewerThread;
  
        std::mutex mKeyFrameMtx;
        std::condition_variable mKeyFrameUpdatedCond;
        std::queue<KeyFrame*> mvKeyFrames;
        std::queue<cv::Mat> mvColorImgs, mvDepthImgs;
  
        bool mbShutdown;
        bool mbFinish;


        PointCloud::Ptr mPointCloud;

        // filter
        pcl::VoxelGrid<PointT> voxel;
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        private:
        double mCyOffset;
public:
    void setCyOffset(double offset);
};

}
#endif

