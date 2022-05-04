#include "ros/ros.h"
#include "color_filter/nodo_mediargb.h"
#include "color_filter/rgb_val.h"

namespace color_filter
{

    MediaRGB::MediaRGB()
    :   nh_("~")
    {
        sub_ = nh_.subscribe("/box_node/box_in_image", 1, &MediaRGB::callback_boximg, this);
        pub_ = nh_.advertise<color_filter::rgb_val>("rgb_val", 1);
    }

    void MediaRGB::calculatemedia(color_filter::rgb_val *rgbval, const color_filter::box_in_imageConstPtr& boximg)
    {
        int pos;
        int mediaval[3] = {0, 0, 0};
        int npixels = (boximg->ymax - boximg->ymin + 1)*(boximg->xmax - boximg->xmin + 1);
        ROS_INFO("n.pixels %d\n",  npixels);
        int cont = 0;
        for (int h = boximg->ymin; h <= boximg->ymax; h++ ){
            for (int w = boximg->xmin; w <= boximg->xmax; w++ ){
                pos = (boximg->image.step * h) + (3 * w);
                cont++;
                for (int i = 0; i < 3; i++){
                    mediaval[i] = mediaval[i] + boximg->image.data[pos+i];
                }
            }
        }
        ROS_INFO("cont %d\n", cont);
        for (int i = 0; i < 3; i++){
            mediaval[i] = mediaval[i] / npixels;
        }
        rgbval->boxpart = boximg->boxpart;
        rgbval->R = mediaval[0];
        rgbval->G = mediaval[1];
        rgbval->B = mediaval[2];
        return;
    }

    void MediaRGB::callback_boximg(const color_filter::box_in_imageConstPtr& boximg)
    {
        color_filter::rgb_val rgbval;
        calculatemedia(&rgbval, boximg);
        pub_.publish(rgbval);
    }
} //namespace color_filter

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mediargb_node");
    ROS_INFO("init \n");
    color_filter::MediaRGB MRGB;
    ros::spin();
    return 0;
}