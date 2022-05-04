#include "string"
#include "color_filter/colorpart.h"
#include "color_filter/nodo_rgbcolor.h"
#include "ros/ros.h"

namespace color_filter
{

    RGBColor::RGBColor()
    : nh_("~")
    {
        sub_ = nh_.subscribe("/mediargb_node/rgb_val", 1, 
                                &RGBColor::callback_rgbval, this);
        pub_ = nh_.advertise<color_filter::colorpart>("colorpart", 1);
    }

    void RGBColor::namecolor(std_msgs::String *str, const color_filter::rgb_valConstPtr& rgbval)
    {
        int R = rgbval->R;
        int G = rgbval->G;
        int B = rgbval->B;

        if (R <= 100 && B<= 100 && G<= 100){
            str->data = "black";
        } else if (R >= 200 && G >= 200 && B >= 200){
            str->data = "white";
        } else if (R >= G && R >= B){
            if (B <= G+50 && B >= G-50){
                str->data = "red";
            } else if (B > G+50){
                str->data = "pink";
            } else if (G > B+50){
                str->data = "orange";
            } 
        } else if (G > R && G >= B){
            if (R <= B+50 && R >= B-50){
                str->data = "green";
            } else if (R > B+50){
                str->data = "yellow";
            } else if (B > R+50){
                str->data = "turquoise";
            } 
        } else if (B > R && B > G){
            if (G <= R+50 && G >= R-50){
                str->data = "blue";
            } else if (G > R+50){
                str->data = "light blue";
            } else if (R > G+50){
                str->data = "purple";
            }
        } 
        return ; 
    }


    void RGBColor::callback_rgbval(const color_filter::rgb_valConstPtr& rgbval)
    {
        color_filter::colorpart clrpart;
        clrpart.boxpart.data = rgbval->boxpart.data;
        namecolor(&clrpart.color, rgbval);
        ROS_INFO("color: %s\n", clrpart.color.data.c_str());
        pub_.publish(clrpart);
    }

} // namaspace color_filter

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbcolor_node");
    color_filter::RGBColor RGBClr;
    ros::spin();
    return 0;
}
