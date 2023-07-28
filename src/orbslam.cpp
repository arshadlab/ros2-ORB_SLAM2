#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"
#include"System.h"
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>

class ORBSlamNode : public rclcpp::Node
{
public:
    ORBSlamNode(std::string ORBvoc_path, std::string settings_path, ORB_SLAM3::System::eSensor mode=ORB_SLAM3::System::RGBD, bool visualize=true, bool doRectify=false) : Node("slam_node"),
            SLAM_(ORBvoc_path, settings_path, mode=mode, visualize=visualize), doRectify_(doRectify)
    {
        // This access to any of cv routine to be required for now otherwise segfault.
        cv::Mat mat(100, 100, CV_8UC3);
        cv::Mat resized_rgb;
        cv::resize(mat, resized_rgb, cv::Size(50, 50));
        

        if (mode == ORB_SLAM3::System::MONOCULAR) {
            RCLCPP_INFO(this->get_logger(), "Setting up MONO Mode");
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10, std::bind(&ORBSlamNode::GrabImage, this, std::placeholders::_1));
        }
        else
        if (mode == ORB_SLAM3::System::RGBD) {
            RCLCPP_INFO(this->get_logger(), "Setting up RGB-D Mode");
      
            source1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
            source2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/depth_raw");
            
            sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(5), *source1_sub_, *source2_sub_);
            sync_->registerCallback(&ORBSlamNode::GrabRGBD, this);
        }
        else
        if (mode == ORB_SLAM3::System::STEREO) {
            RCLCPP_INFO(this->get_logger(), "Setting up STEREO Mode");
    

            if (doRectify){

                cv::FileStorage fsSettings(settings_path, cv::FileStorage::READ);
                if(!fsSettings.isOpened()){
                    cerr << "ERROR: Wrong path to settings" << endl;
                    assert(0);
                }

                cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
                fsSettings["LEFT.K"] >> K_l;
                fsSettings["RIGHT.K"] >> K_r;

                fsSettings["LEFT.P"] >> P_l;
                fsSettings["RIGHT.P"] >> P_r;

                fsSettings["LEFT.R"] >> R_l;
                fsSettings["RIGHT.R"] >> R_r;

                fsSettings["LEFT.D"] >> D_l;
                fsSettings["RIGHT.D"] >> D_r;

                int rows_l = fsSettings["LEFT.height"];
                int cols_l = fsSettings["LEFT.width"];
                int rows_r = fsSettings["RIGHT.height"];
                int cols_r = fsSettings["RIGHT.width"];

                if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                        rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
                    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
                    assert(0);
                }

                cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l_,M2l_);
                cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r_,M2r_);
            }

            source1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/left");
            source2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/right");
            sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(5), *source1_sub_, *source2_sub_);
            sync_->registerCallback(&ORBSlamNode::GrabStereo, this);
        }
    }

private:
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr& msgRGB, const sensor_msgs::msg::Image::SharedPtr& msgD)
    {
        RCLCPP_INFO(this->get_logger(), "Received images from both topics");
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cv_ptrSrc1_ = cv_bridge::toCvShare(msgRGB);
            cv_ptrSrc2_ = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        SLAM_.TrackRGBD(cv_ptrSrc1_->image, cv_ptrSrc2_->image, msgRGB->header.stamp.sec);

    }
     void GrabStereo(const sensor_msgs::msg::Image::SharedPtr& msgLeft, const sensor_msgs::msg::Image::SharedPtr& msgRight)
    {
        RCLCPP_INFO(this->get_logger(), "Received images from both topics");
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cv_ptrSrc1_ = cv_bridge::toCvShare(msgLeft);
            cv_ptrSrc2_ = cv_bridge::toCvShare(msgRight);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        if (doRectify_){
            cv::Mat imLeft, imRight;
            cv::remap(cv_ptrSrc1_->image,imLeft,M1l_,M2l_,cv::INTER_LINEAR);
            cv::remap(cv_ptrSrc2_->image,imRight,M1r_,M2r_,cv::INTER_LINEAR);
            SLAM_.TrackStereo(imLeft, imRight, msgLeft->header.stamp.sec);
        }
        else {        
            SLAM_.TrackStereo(cv_ptrSrc1_->image, cv_ptrSrc2_->image, msgLeft->header.stamp.sec);
        }
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image");
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cv_ptrSrc1_ = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        SLAM_.TrackMonocular(cv_ptrSrc1_->image, msg->header.stamp.sec);

    }


    ORB_SLAM3::System SLAM_;
    bool doRectify_;
    cv::Mat M1l_,M2l_,M1r_,M2r_;

    cv_bridge::CvImageConstPtr cv_ptrSrc1_;
    cv_bridge::CvImageConstPtr cv_ptrSrc2_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> source1_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> source2_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > sync_;    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ORB_SLAM3::System::eSensor sensor;
    std::string mode = argv[1];
    
    if (mode == "mono") 
         sensor = ORB_SLAM3::System::MONOCULAR;
    else
    if (mode == "rgbd") 
         sensor = ORB_SLAM3::System::RGBD;
    else
    if (mode == "stereo") 
        sensor = ORB_SLAM3::System::STEREO;

    rclcpp::spin(std::make_shared<ORBSlamNode>(argv[2], argv[3], sensor, true));
    rclcpp::shutdown();

    return 0;
}
