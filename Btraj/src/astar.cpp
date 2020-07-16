#include <ros/ros.h>
#include <bits/stdc++.h>
#include "Asearch.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ROS_WARN("Astar program online!");
    Astar_search::Asearch s;
    ros::spin();
    return 0;
}