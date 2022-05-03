
#ifndef COLOR_FILTER_NODO_BOX_H
#define COLOR_FILTER_NODO_BOX_H

#include "string"
#include "color_filter/box_in_image.h"
#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ctime>
#include <chrono>
#include <sstream>

namespace color_filter
{
    class PersonBoxes
    {
    private:
        ros::Publisher pub_;
        ros::NodeHandle nh_;

        message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;

        typedef message_filters::sync_policies::ApproximateTime< darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy_bbx;
        message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;


    public:
        PersonBoxes();
        void asigncorners(color_filter::box_in_image *boximg, int xmax, int ymax, int xmin, int ymin);

        void asignimage(color_filter::box_in_image *boximg, const sensor_msgs::ImageConstPtr& rgbimg);
        void callback_bbx(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, const sensor_msgs::ImageConstPtr& rgbimg);
    };
} //namespace color_filter

#endif  // COLOR_FILTER_NODO_BOX_H