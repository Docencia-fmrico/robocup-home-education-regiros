#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "person_tf_sub");
  ros::NodeHandle n;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped bf2person_msg;
    tf2::Stamped<tf2::Transform> bf2person;

    std::string error;

    if (buffer.canTransform("base_footprint", "person/0", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2person_msg = buffer.lookupTransform("base_footprint", "person/0", ros::Time(0));

      tf2::fromMsg(bf2person_msg, bf2person);

      double dist_ = bf2person.getOrigin().length();
      //double roll, pitch, yaw;

      //tf2::Matrix3x3(bf2person.getRotation()).getRPY(roll, pitch, yaw);

      ROS_INFO("base_footprint -> person [%lf, %lf] dist = %lf                    %lf ago",
        bf2person.getOrigin().x(),
        bf2person.getOrigin().y(),
        dist_,
        (ros::Time::now() - bf2person.stamp_).toSec());
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}