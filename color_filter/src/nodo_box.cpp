#include "ros/ros.h"
#include "color_filter/box_in_image.h"
#include "color_filter/nodo_box.h"

namespace color_filter
{
    PersonBoxes::PersonBoxes()
    :   nh_("~"),
        rgb_sub_(nh_, "/camera/rgb/image_raw", 1),
        bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
        sync_bbx_(MySyncPolicy_bbx(10), bbx_sub_, rgb_sub_)

    {
        sync_bbx_.registerCallback(boost::bind(&PersonBoxes::callback_bbx, this, _1, _2));
        pub_ = nh_.advertise<color_filter::box_in_image>("box_in_image", 1);
    }

    void PersonBoxes::asigncorners(color_filter::box_in_image *boximg, int xmax, int ymax, int xmin, int ymin)
    {
        boximg->xmax = xmax;
        boximg->ymax = ymax;
        boximg->xmin = xmin;
        boximg->ymin = ymin;
        return;
    }

    void PersonBoxes::asignimage(color_filter::box_in_image *boximg, const sensor_msgs::ImageConstPtr& rgbimg)
    {
        boximg->image.data = rgbimg->data;
        boximg->image.encoding = rgbimg->encoding;
        boximg->image.header = rgbimg->header;
        boximg->image.height = rgbimg->height;
        boximg->image.is_bigendian = rgbimg->is_bigendian;
        boximg->image.step = rgbimg->step;
        boximg->image.width = rgbimg->width;
        return;
    }

    void  PersonBoxes::callback_bbx(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, const sensor_msgs::ImageConstPtr& rgbimg)
    {
        int headsize;
        color_filter::box_in_image shirt;
    
        for (const auto & box : boxes->bounding_boxes) {
            if (box.Class == "person"){
                headsize = (box.ymax-box.ymin)/6;
                asigncorners(&shirt, box.xmax, box.ymin + headsize*3,
                            box.xmin, box.ymin + headsize*1);
                asignimage(&shirt, rgbimg);
                shirt.boxpart.data = "shirt";
                pub_.publish(shirt);
            }
        }
    }
}   // namespace color_filter


int main(int argc, char** argv)
{
  ros::init(argc, argv, "box_node");
  color_filter::PersonBoxes pb;
  ros::spin();
  return 0;
}









