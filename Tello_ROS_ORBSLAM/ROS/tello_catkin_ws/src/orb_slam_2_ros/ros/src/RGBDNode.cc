#include "RGBDNode.h"
#include<pcl-1.8/pcl/ros/conversions.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);
    
    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    // global_map = node.orb_slam_->mpPointCloudMapping->getGlobalMap();
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    //数据格式转换

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGB>);

    global_map = node.orb_slam_->mpPointCloudMapping->getGlobalMap();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    cout<<"-----------------------------------------------------------"<<endl;
    cout <<"ros is running "<<endl;
 

    ros::spin();
    node.stopFlag = true;
    node.pubThread->join();
    node.orb_slam_->save();
    // Stop all threads
    node.orb_slam_->Shutdown();
    node.orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();
    return 0;
}

void RGBDNode::pointCloudPublisher(){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    global_map = this->orb_slam_->mpPointCloudMapping->getGlobalMap();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Publisher pcl_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("/orbslam2/dense_pointcloud", 10);
    cout<<"-----------------------------------------------------------"<<endl;
    cout <<"point cloud publisher is running "<<endl;
    ros::Rate loop_rate(5);
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 pt;
    while (ros::ok()&&!stopFlag)
    {
    //TODO OCTOMAP添加
        {
            std::unique_lock<std::mutex> locker(this->orb_slam_->mpPointCloudMapping->mPointCloudMtx);
            pcl::toROSMsg<pcl::PointXYZRGB>(*global_map,pt);// 转换成ROS下的数据类型 最终通过topic发布
        }
        //sensor_msgs::PointCloud output;        
        pcl_conversions::moveFromPCL(pt,output);
        output.header.stamp=ros::Time::now();
       // output.header.frame_id  ="camera_rgb_frame";
        output.header.frame_id  ="map";
        pcl_pub.publish(output);
        loop_rate.sleep();
    }
}
RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
  stopFlag = false;
  pubThread = make_shared<thread>( bind(&RGBDNode::pointCloudPublisher, this ) );
  orb_slam_->setPointCloudCyOffset(point_cloud_cy_offset);
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  Update ();
}
