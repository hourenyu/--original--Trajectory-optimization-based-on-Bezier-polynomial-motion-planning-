#ifndef ASEARCH_H
#define ASEARCH_H

#include <ros/ros.h>
#include <bits/stdc++.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/heap/binomial_heap.hpp>
#include <visualization_msgs/MarkerArray.h>
namespace Astar_search
{
    struct Node2D
    {
        float x;
        float y;
        Node2D* parent=NULL;
        float g=0;
        float h=0;
        float distance_cost=0;//与障碍物距离的代价，如何离障碍物近则cost大
        int idx_x=-1;
        int idx_y=-1;
        bool is_Occupancy=false;
        int state=0;//0:表示未扩展 1:在openlist中 -1:在closedlist中
        Node2D(){};
        Node2D(float X,float Y):x(X),y(Y),parent(NULL){}
        Node2D(const geometry_msgs::PoseStamped::ConstPtr& initial)
        {
            x=initial->pose.position.x;
            y=initial->pose.position.y;
            parent=NULL;
        }
        Node2D(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial)
        {
            x=initial->pose.pose.position.x;
            y=initial->pose.pose.position.y;
            parent=NULL;
        }
        ~Node2D(){}

        float getC(){return (g+h+distance_cost);}
    };

    struct CompareNodes {
        bool operator()(Node2D* lhs, Node2D* rhs) const {
            return lhs->getC() > rhs->getC();
        }
    };
    
    class Asearch
    {
    private:
        Node2D* start; //类内的指针一定要用new初始化
        Node2D* goal;
        ros::NodeHandle n;
        ros::Publisher position_pub;
        ros::Publisher pathvis_pub;
        ros::Publisher path_pub;
        ros::Publisher node_pub;
        ros::Subscriber Map_sub;
        ros::Subscriber start_sub;
        ros::Subscriber goal_sub;
        nav_msgs::Path path;
        Node2D* gridmap;
        float width; //map实际大小＝map栅格格数（像素）*分辨率
        float height;
        int width_grid;//map栅格格数（像素）
        int height_grid;
        float resolution=1;
        bool get_s=false;
        bool get_g=false;
        nav_msgs::OccupancyGrid::Ptr map;
        boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> Openset;
        
    public:
        Asearch();
        ~Asearch();
        void set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
        void set_goal(const geometry_msgs::PoseStamped::ConstPtr& initial);
        void get_start();
        void get_goal();
        void setMap(const nav_msgs::OccupancyGrid::Ptr map);
        bool is_collisionfree(Node2D* node);
        void transform2idx(Node2D* node);
        void reset_map();
        float Heuristics_cost(Node2D* start,Node2D* goal);
        void trackback(Node2D* node);
        Node2D* findpath();
        geometry_msgs::Point idx_INV(const int &x,const int & y);
        float distance_cost(Node2D* node);
    };
     
}

#endif