#include "string"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "find_my_mate/str_info.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <chrono>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros/ros.h"

namespace find_my_mate
{
class RotUPerson 
{
  public:
    RotUPerson()
    :   nh_("~"),
        cinf_sub_(nh_, "/camera/rgb/camera_info", 1),
        bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
        sync_bbx_(MySyncPolicy_bbx(10), cinf_sub_, bbx_sub_),
        positioned_(false),
        firsttick_(true),
        dir_(-1)
    {
        pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        sync_bbx_.registerCallback(boost::bind(&RotUPerson::callback_bbx, this, _1, _2));
    }

    void callback_bbx(const sensor_msgs::CameraInfoConstPtr& cinf, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
    {
        for (const auto & box : boxes->bounding_boxes) {
            if (box.Class == "person" && box.probability > 0.6){
                ROS_INFO("person detected");
                int px = (box.xmax + box.xmin) / 2;
                if (px > cinf->width/2 - cinf->width/8 && px < cinf->width/2 + cinf->width/8 ){
                    ROS_INFO("inside the params");
                    positioned_ = true;
                }
                if (px > cinf->width/2 + cinf->width/8 ){
                    ROS_INFO("right");
                    dir_ = -1;
                }
                if (px < cinf->width/2 - cinf->width/8 ){
                    ROS_INFO("left");
                    dir_ = 1;
                }
            }
        }
    }
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinf_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
    
    geometry_msgs::Twist twist;
    ros::Publisher pub_;

    bool firsttick_;
    bool positioned_;
    int dir_;

    
};
}  // namespace find_my_mate

int
main(int argc, char** argv) 
{
    ros::init(argc, argv, "RotUPersontest_node");
    find_my_mate::RotUPerson RUP;
    ros::Rate loop_rate(10);
    ros::Time initTime_ ;
    initTime_ = ros::Time::now();
    
    
    while(ros::ok){
        
        ROS_INFO("%d \n", (ros::Time::now() - initTime_).sec );

        if (RUP.positioned_){
            ROS_INFO("positioned");
            RUP.twist.angular.z=0;
            RUP.twist.linear.x=0;
            RUP.firsttick_ = true;
            RUP.positioned_ = false;
            RUP.pub_.publish(RUP.twist);
            return 0;
        } else if ((ros::Time::now() - initTime_).sec <= 10){
            ROS_INFO("rotate");
            RUP.twist.angular.z=0.45*RUP.dir_;
            RUP.twist.linear.x=0;
            RUP.pub_.publish(RUP.twist);
        } else {

            ROS_INFO("time out");
            RUP.twist.angular.z=0;
            RUP.twist.linear.x=0;
            RUP.firsttick_ = true;
            RUP.pub_.publish(RUP.twist);
            return 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
        if (RUP.firsttick_){
            initTime_ = ros::Time::now();
            ROS_INFO("AA%d", initTime_.sec);
            RUP.firsttick_ = false;
        }
    }
}