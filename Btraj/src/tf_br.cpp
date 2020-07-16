#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;

    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(10, 10, 0)),
                ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        r.sleep();
    }
}
