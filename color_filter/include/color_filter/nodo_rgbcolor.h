#ifndef COLOR_FILTER_NODO_RGBCOLOR_H
#define COLOR_FILTER_NODO_RGBCOLOR_H

#include "string"
#include "std_msgs/String.h"
#include "color_filter/colorpart.h"
#include "color_filter/rgb_val.h"
#include "ros/ros.h"

#include <sensor_msgs/Image.h>

namespace color_filter
{
    class RGBColor
    {
    private:
        ros::Publisher pub_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

    public:
        RGBColor();
        void namecolor(std_msgs::String *str, const color_filter::rgb_valConstPtr& rgbval);
        void callback_rgbval(const color_filter::rgb_valConstPtr& rgbval);
    };
} //namespace color_filter

#endif  // COLOR_FILTER_NODO_RGBCOLOR_H