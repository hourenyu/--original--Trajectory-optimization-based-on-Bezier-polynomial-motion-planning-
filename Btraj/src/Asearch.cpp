#include "Asearch.h"

using namespace std;
using namespace Astar_search;

//对于栅格地图有N个像素点格数，那么想对应的索引idx应该是0~N-1

Asearch::Asearch(){
    path.header.frame_id="map";//即是以map坐标系为基(原点)进行计算和显示,
    //若这里以odom坐标系为基显示path的话（假设map与odom之间偏移为(10,10,0)）则path会偏移(10,10,0).
    //这是因为连接path的点本身是在map坐标下计算的(假设其中一个点为(5,5,0)),也就是说这点会相对坐标原点偏移(5,5,0)而不管坐标是谁.
    //因此，如果把这个点放在odom坐标系下，则这个点也会相对odom的原点偏移(5,5,0),这样的话这个点就会相对map偏移(10,10,0)+(5,5,0)=(15,15,0)
    position_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
    node_pub = n.advertise<visualization_msgs::Marker>("/pathNodes", 1);
    pathvis_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    path_pub=n.advertise<geometry_msgs::PoseArray>("/path_to_btraj", 1);

    Map_sub = n.subscribe("/map", 1, &Asearch::setMap, this);
    start_sub=n.subscribe("/initialpose", 1, &Asearch::set_start, this);
    goal_sub=n.subscribe("/move_base_simple/goal",1,&Asearch::set_goal,this);
};
Asearch::~Asearch(){};

void Asearch::set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial){
    start=new Node2D();
    start->x=initial->pose.pose.position.x;
    start->y=initial->pose.pose.position.y;
    start->parent=nullptr;
    geometry_msgs::PoseStamped start_pub;
    start_pub.header.frame_id="map";
    start_pub.header.stamp=ros::Time::now();
    start_pub.pose.position=initial->pose.pose.position;
    start_pub.pose.orientation=initial->pose.pose.orientation;
    position_pub.publish(start_pub);
    transform2idx(start);
    if(is_collisionfree(start)){
        get_start();
        get_s=true;
        if(get_s&&get_g){
            Node2D* path=findpath();
            trackback(path);
        }
    }
    else{
        ROS_WARN("Invalid start!");
    }
}

void Asearch::get_start(){
    ROS_INFO("get_start: %f, %f", start->x,start->y);
}

void Asearch::set_goal(const geometry_msgs::PoseStamped::ConstPtr& initial){
    goal=new Node2D();
    goal->x=initial->pose.position.x;
    goal->y=initial->pose.position.y;
    transform2idx(goal);
    if(is_collisionfree(goal)){
        get_goal();
        get_g=true;
        if(get_s&&get_g){
            Node2D* path=findpath();
            trackback(path);
        }
    }
    else{
        ROS_WARN("Invalid goal!");
    }

}

void Asearch::get_goal(){
    ROS_INFO("get_goal: %f, %f", goal->x,goal->y);
}

void Asearch::setMap(const nav_msgs::OccupancyGrid::Ptr Map){
    map=Map;
    width_grid=map->info.width;
    height_grid=map->info.height;
    resolution=map->info.resolution;
    width=(float)width_grid*resolution;
    height=(float)height_grid*resolution;

    gridmap=new Node2D[width_grid*height_grid];
    for(int x=0;x<width_grid;++x){
        for(int y=0;y<height_grid;++y){
            gridmap[y*width_grid+x].is_Occupancy=(map->data[y*width_grid+x]?true:false);
        }
    }
}

void Asearch::reset_map(){
    delete[] gridmap; //delete the old map
    gridmap=new Node2D[width_grid*height_grid];
    for(int x=0;x<width_grid;++x){
        for(int y=0;y<height_grid;++y){
            gridmap[y*width_grid+x].is_Occupancy=(map->data[y*width_grid+x]?true:false);
        }
    }
}

bool Asearch::is_collisionfree(Node2D* node){
    bool collisionfree=!gridmap[node->idx_y*width_grid+node->idx_x].is_Occupancy;
    bool bounding=false;
    if(node->idx_x>=0&&node->idx_x<width_grid&&node->idx_y>=0&&node->idx_y<height_grid){
        bounding=true;
    }
    return (collisionfree&&bounding);
}

void Asearch::transform2idx(Node2D* node){
    node->idx_x=node->x/resolution;
    node->idx_y=node->y/resolution;
}

float Asearch::Heuristics_cost(Node2D* start,Node2D* goal){
    float distance;
    distance=sqrt(pow((goal->idx_x-start->idx_x)*resolution,2)+pow((goal->idx_y-start->idx_y)*resolution,2));
    return distance*1.0001;//tie breaker
}

Node2D* Asearch::findpath(){
    Openset.clear();
    reset_map();
    bool vis_nodes=true;
    
    visualization_msgs::Marker pathNode;
    pathNode.header.frame_id = "map";
    pathNode.ns="visnode";
    pathNode.id=0;
    pathNode.header.stamp = ros::Time::now();
    pathNode.type = visualization_msgs::Marker::SPHERE_LIST;
    pathNode.action = visualization_msgs::Marker::ADD;
    pathNode.scale.x = 0.5;
    pathNode.scale.y = 0.5;
    pathNode.scale.z = 0.5;
    pathNode.color.a = 1.0;
    pathNode.color.r = 0.0f;
    pathNode.color.g = 1.0f;
    pathNode.color.b = 0.0f;
    pathNode.lifetime=ros::Duration();
    int max_iter=300000,iter=0;
    start->h=Heuristics_cost(start,goal);
    Openset.push(start);
    while (!Openset.empty())
    {
        Node2D* current=new Node2D();
        Node2D* succ=new Node2D();
        current=Openset.top();
        Openset.pop();
        if(current->idx_x==goal->idx_x&&current->idx_y==goal->idx_y){
            if(vis_nodes)
                node_pub.publish(pathNode);
            ROS_WARN("Find Goal!");
            return current;
        }
        if(vis_nodes){//从openset中弹出来用于扩展的点
            pathNode.points.push_back(idx_INV(current->idx_x,current->idx_y));
        }
        gridmap[current->idx_y*width_grid+current->idx_x].state=-1;
        int temp_idxx=0,temp_idxy=0;
        for(int x=-1;x<=1;++x){
            for(int y=-1;y<=1;++y){
                if(x==0&&y==0)//可以走对角
                // if(abs(x)==abs(y))//不可以走对角
                    continue;
                temp_idxx=current->idx_x+x;
                temp_idxy=current->idx_y+y;
                if(gridmap[temp_idxy*width_grid+temp_idxx].is_Occupancy||
                            gridmap[temp_idxy*width_grid+temp_idxx].state==-1)//已扩展或有障碍物
                    continue;
                succ->state=1;
                succ->idx_x=temp_idxx;
                succ->idx_y=temp_idxy;
                succ->g=current->g+sqrt(pow(x*resolution,2)+pow(y*resolution,2));
                succ->h=Heuristics_cost(succ,goal);
                succ->parent=current;
                succ->distance_cost=distance_cost(succ);
                if(gridmap[temp_idxy*width_grid+temp_idxx].state==1&&is_collisionfree(succ)){//已经在openset中
                //应该直接用gridmap记录路径链表
                    if((succ->g+succ->distance_cost)<(gridmap[temp_idxy*width_grid+temp_idxx].g+
                                                        gridmap[temp_idxy*width_grid+temp_idxx].distance_cost))
                        gridmap[temp_idxy*width_grid+temp_idxx]=*succ;
                }
                else if(is_collisionfree(succ)){ //还没扩展
                    gridmap[temp_idxy*width_grid+temp_idxx]=*succ;
                    Openset.push(&gridmap[temp_idxy*width_grid+temp_idxx]);
                    // if(vis_nodes){//所有被放入openset中的点
                    //     pathNode.points.push_back(idx_INV(temp_idxx,temp_idxy));
                    // }
                }
            }
        }
        ++iter;
        if(iter>max_iter){
            ROS_WARN("Maximum iteration has been reached !");
            return succ;
        }
        delete succ;
    }
    return NULL;
}

void Asearch::trackback(Node2D* node){
    if(!node)
        return;
    visualization_msgs::Marker Apath;
    // geometry_msgs::Point pt;
    geometry_msgs::PoseArray path_to_inflate;
    geometry_msgs::Pose temp;
    Apath.header.frame_id = "map";
    Apath.ns="visApath";
    Apath.id=0;
    Apath.header.stamp = ros::Time::now();
    Apath.type = visualization_msgs::Marker::SPHERE_LIST;
    Apath.action = visualization_msgs::Marker::ADD;
    Apath.scale.x = 0.5;
    Apath.scale.y = 0.5;
    Apath.scale.z = 0.5;
    Apath.color.a = 1.0;
    Apath.color.r = 1.0f;
    Apath.color.g = 0.0f;
    Apath.color.b = 0.0f;
    Apath.lifetime=ros::Duration();
    while(node){
        // pt=idx_INV(node->idx_x,node->idx_y);
        Apath.points.push_back(idx_INV(node->idx_x,node->idx_y));
        temp.position.x=node->idx_x;
        temp.position.y=node->idx_y;
        path_to_inflate.poses.push_back(temp);
        node=node->parent;
    }
    pathvis_pub.publish(Apath);
    path_pub.publish(path_to_inflate);
}

geometry_msgs::Point Asearch::idx_INV(const int &x,const int & y){
    geometry_msgs::Point pt;
    pt.x=x+resolution/2;
    pt.y=y+resolution/2;
    pt.z=0;
    return pt;
}

float Asearch::distance_cost(Node2D* node){
    for(int x=-1;x<=1;++x){
        for(int y=-1;y<=1;++y){
            if(x==0&&y==0)
                continue;
            if(gridmap[(node->idx_y+y)*width_grid+(node->idx_x+x)].is_Occupancy){
               return 10.0; 
            }
        }
    }
    return 0.0;
}


