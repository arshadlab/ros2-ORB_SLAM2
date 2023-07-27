#include "rgbd-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{
    //rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    //depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");

    //rgb_sub_.subscribe(this, "camera/rgb");
    //depth_sub_.subscribe(this, "camera/depth");
  
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> approximate_policy;
    //message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), image_sub_, cloud_sub_);
 
 
      
    //syncApproximate(approximate_sync_policy(10), rgb_sub_, depth_sub_);
    //syncApproximate.registerCallback(&RgbdSlamNode::GrabRGBD, this);

          
        rgb_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, "/camera/image_raw");
        
      
        depth_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, "/camera/image_raw/compressedDepth");
       
        //sync_ = new message_filters::Synchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(*rgb_sub_, *depth_sub_);
        sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(5), *rgb_sub_, *depth_sub_);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        sync_->registerCallback(&RgbdSlamNode::GrabRGBD, this);
       

}

RgbdSlamNode::~RgbdSlamNode()
{
    delete rgb_sub_;
    delete depth_sub_;
    //delete sync_;
    
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    printf("Hello3");
    RCLCPP_INFO(this->get_logger(), "Image received");

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Resize the RGB image
    cv::Mat resized_rgb;
    cv::resize(cv_ptrRGB->image, resized_rgb, cv::Size(640, 480));

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Resize the depth image
    cv::Mat resized_depth;
    //cv::resize(cv_ptrD->image, resized_depth, cv::Size(640, 480));
     // Sleep for 2 seconds
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cv::Mat Tcw ;
    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec);

}
