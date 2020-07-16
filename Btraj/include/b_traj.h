#ifndef B_TRAJ
#define B_TRAJ

#include <ros/ros.h>
#include <bits/stdc++.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Eigen>
#include <qpOASES.hpp>
using namespace std;
using namespace Eigen;
using namespace qpOASES;
namespace Bezier
{
/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
struct bounding_box
{
    Vector3d center;
    Vector3i P1;
    Vector3i P2;
    Vector3i P3;
    Vector3i P4;
    bounding_box(){};
    ~bounding_box(){};
    bounding_box(const geometry_msgs::Point& pt,const int& resolution){
        P1(0)=pt.x;
        P1(1)=pt.y;
        P1(2)=0;
        P2=P1;
        P3=P1;
        P4=P1;
        center(0)=pt.x+resolution/2;
        center(1)=pt.y+resolution/2;
        center(2)=0;
    }
};

class bezier
{
private:
    float width; //map实际大小＝map栅格格数（像素）*分辨率
    float height;
    int width_grid; //map栅格格数（像素）
    int height_grid;
    float resolution = 1;
    Vector3d start;
    Vector3d goal;
    Vector3d start_v;
    Vector3d goal_v;
    Vector3d start_a;
    Vector3d goal_a;
    ros::NodeHandle n;
    ros::Subscriber Map_sub;
    ros::Subscriber inflate_path;
    ros::Subscriber start_sub;
    ros::Subscriber goal_sub;
    ros::Publisher box_pub;
    ros::Publisher traj_pub;
    ros::Publisher center_pub;
    nav_msgs::OccupancyGrid::Ptr map;
    bounding_box box_last;
    vector<bounding_box> box_list;
    visualization_msgs::MarkerArray box_list_vis;
    int max_inflate_iter=1000;
    int traj_order=6;
    vector<MatrixXd> MQM_list;
    vector<double> times;
    MatrixXd Aeq;
    MatrixXd Beq;
    MatrixXd Aieq;
    MatrixXd uBieq;
    MatrixXd lBieq;
    bool is_x=true;//true:优化ｘ轴　false:优化y轴
    double vx_max;
    double vx_min;
    double vy_max;
    double vy_min;
    double ax_max;
    double ax_min;
    double ay_max;
    double ay_min;
public:
    bezier();
    ~bezier();
    void set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
    void set_goal(const geometry_msgs::PoseStamped::ConstPtr& initial);
    void inflate(const geometry_msgs::PoseArray::Ptr& path);
    void inflate_box(bounding_box& box);
    void set_map(const nav_msgs::OccupancyGrid::Ptr Map);
    bool is_contain(const bounding_box& box_last,const bounding_box& box_now);//判断刚找到的box_now是否包含在上次找到的box_last中
    bool is_in_box(const geometry_msgs::Point& pt);//判断正要膨胀的点是否已经有box覆盖
    void visual_box(const vector<bounding_box>& box_list);
    geometry_msgs::Point trans2pt(const Vector3i& p);
    int delete_box(const bounding_box& box_last,const bounding_box& box_now,const geometry_msgs::PoseArray::Ptr& path);//return 0:不删除任何box,正常添加;　１:删除box_now;　2:删除box_last
    void simplify_box();//在求解完成所有bounding_box后可以重新遍历一遍所有的box 如果当前的box与往后的box有交集则可以删除中间的过渡box
    bool is_Overlap(const bounding_box& box_old,const bounding_box& box_now);
    void Trajectory_Generation();
    void vistraj(real_t* xopt,real_t* yopt);
    void set_MQMlist();
    void set_AeqBeq();
    void set_AieqBieq();
    MatrixXd get_M();
    void Time_allocate();
    geometry_msgs::Point getBezierPos(const int& seg,const double& t,real_t* xopt,real_t* yopt);
    double Bernstein_base(const int& i,const double& t,const double& time_i);
    double factorial(const int& n);
    void get_Overlap_center(vector<Vector3d>& pt_list);
    void vis_center(const vector<Vector3d>& pt_list);
};
}

#endif
