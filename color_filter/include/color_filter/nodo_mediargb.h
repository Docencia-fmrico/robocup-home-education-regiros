
#ifndef COLOR_FILTER_NODO_BOX_H
#define COLOR_FILTER_NODO_BOX_H

#include "string"
#include "color_filter/box_in_image.h"
#include "color_filter/rgb_val.h"
#include "ros/ros.h"

#include <sensor_msgs/Image.h>

namespace color_filter
{
    class MediaRGB
    {
    private:
        ros::Publisher pub_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

    public:
        MediaRGB();
        void calculatemedia(color_filter::rgb_val *rgbval, const color_filter::box_in_imageConstPtr& boximg);
        void callback_boximg(const color_filter::box_in_imageConstPtr& boximg);
    };
} //namespace color_filter

#endif  // COLOR_FILTER_NODO_BOX_H