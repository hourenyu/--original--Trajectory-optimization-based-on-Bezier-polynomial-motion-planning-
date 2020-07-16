#include "b_traj.h"

using namespace Bezier;

int _max_inflate_iter, _traj_order;
double _vx_max,_vx_min,_vy_max,_vy_min,_ax_max,_ax_min,_ay_max,_ay_min;

bezier::bezier(){
    max_inflate_iter=_max_inflate_iter;
    traj_order=_traj_order;
    vx_max=_vx_max;
    vx_min=_vx_min;
    vy_max=_vy_max;
    vy_min=_vy_min;
    ax_max=_ax_max;
    ax_min=_ax_min;
    ay_max=_ay_max;
    ay_min=_ay_min;
    Map_sub = n.subscribe("/map", 1, &bezier::set_map, this);
    inflate_path=n.subscribe("/path_to_btraj",1,&bezier::inflate,this);
    box_pub=n.advertise<visualization_msgs::MarkerArray>("/box_vis", 1);
    traj_pub=n.advertise<visualization_msgs::Marker>("/trajectory", 1);
    center_pub=n.advertise<visualization_msgs::Marker>("/center_pt", 1);
    start_sub=n.subscribe("/initialpose", 1, &bezier::set_start, this);
    goal_sub=n.subscribe("/move_base_simple/goal",1,&bezier::set_goal,this);

};

bezier::~bezier(){};

void bezier::set_map(const nav_msgs::OccupancyGrid::Ptr Map){
    map=Map;
    width_grid=map->info.width;
    height_grid=map->info.height;
    resolution=map->info.resolution;
    width=(float)width_grid*resolution;
    height=(float)height_grid*resolution;
}

void bezier::set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial){
    start(0)=(double)((int)initial->pose.pose.position.x)+resolution/2;
    start(1)=(double)((int)initial->pose.pose.position.y)+resolution/2;
    start(2)=0;
    start_v(0)=0;
    start_v(1)=0;
    start_v(2)=0;
    start_a(0)=0;
    start_a(1)=0;
    start_a(2)=0;
}

void bezier::set_goal(const geometry_msgs::PoseStamped::ConstPtr& initial){
    goal(0)=(double)((int)initial->pose.position.x)+resolution/2;
    goal(1)=(double)((int)initial->pose.position.y)+resolution/2;
    goal(2)=0;
    goal_v(0)=0;
    goal_v(1)=0;
    goal_v(2)=0;
    goal_a(0)=0;
    goal_a(1)=0;
    goal_a(2)=0;
}

void bezier::inflate(const geometry_msgs::PoseArray::Ptr& path){
    // cout<<123213213<<endl;
    if(map->data.empty()){
        ROS_WARN("Has not got a map!!!");
        return;
    }
    box_list.clear();
    geometry_msgs::Point pt;
    pt.x=0,pt.y=0,pt.z=0;
    box_last=bounding_box(pt,resolution);
    int n=path->poses.size();
    // cout<<"a"<<endl;
    for(int i=n-1;i>=0;--i){//路径是回溯的，所以第一个点在数组的最后
        if(is_in_box(path->poses[i].position))
            continue;
        bounding_box box_now(path->poses[i].position,resolution);
        // cout<<"b"<<endl;
        inflate_box(box_now);
        // if(is_contain(box_last,box_now))
        //     continue;
        // cout<<"c"<<endl;
        int flag=delete_box(box_last,box_now,path);//两个box的非交集部分，谁的非交集部分没有点说明它是重复的
        if(flag==1){
            //do nothing  return 1:删除box_now
        }
        else if(flag==2){
            if(!box_list.empty())
                box_list.pop_back(); //return 2:删除box_last
            
            while (!box_list.empty())//判断之前的box会不会被box_now取代
            {
                int size=box_list.size();
                box_last=box_list[size-1];
                int flag2=delete_box(box_last,box_now,path);
                if(flag2!=2)
                    break;
                box_list.pop_back();
            }

            box_list.push_back(box_now);
            box_last=box_now;
        }
        else{
            box_list.push_back(box_now);
            box_last=box_now;
        }
    }
    if(box_list.size()>1)
        simplify_box();
    visual_box(box_list);
    Trajectory_Generation();
}

void bezier::simplify_box(){
    auto temp=box_list;
    box_list.clear();
    int n=temp.size();
    int idx_old=0;
    box_list.push_back(temp[idx_old]);
    for(int i=1;i<n;++i){
        if(!is_Overlap(temp[idx_old],temp[i])){
            // cout<<i<<endl;
            box_list.push_back(temp[i-1]);
            idx_old=i-1;
            if(is_Overlap(temp[i-1],temp[n-1])){//one shot
                break;
            }
        }
    }
    box_list.push_back(temp[n-1]);
}
/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
bool bezier::is_Overlap(const bounding_box& box_old,const bounding_box& box_now){
    if(box_now.P3(0)>=(box_old.P2(0)+1)||box_now.P3(1)>=(box_old.P2(1)+1)||//这里加１是为了将box的上边界扩充完整，因为P1234用的是idx数据
       box_old.P3(0)>=(box_now.P2(0)+1)||box_old.P3(1)>=(box_now.P2(1)+1)){
       return false; 
    }
    return true;
}

void bezier::Trajectory_Generation(){
    set_MQMlist();
    // cout<<MQM_list[0]<<endl;
    // cout<<endl;
    int nums=MQM_list.size()*(traj_order+1);
    int sz=nums*nums;
    int delta_col=0,delta_row=0,cnt=0;
    real_t H[sz];
    real_t g[nums];
    memset(H,0,sizeof(real_t)*sz);
    memset(g,0,sizeof(real_t)*nums);

    for(int i=0;i<MQM_list.size();++i){
        for(int j=0;j<traj_order+1;++j){
            for(int k=0;k<traj_order+1;++k){
                H[cnt+delta_col+delta_row]=MQM_list[i](j,k);
                // H[cnt+delta_col+delta_row]=temp(j,k);
                ++cnt;
            }
            delta_row+=nums;
            cnt=0;
        }
        delta_col+=(traj_order+1);
    }

    is_x=true;//qpOASES优化x轴
    set_AeqBeq();
    set_AieqBieq();
    sz=Aeq.cols()*Aeq.rows()+Aieq.cols()*Aieq.rows();
    real_t A[sz];
    memset(A,0,sizeof(real_t)*sz);
    nums=Aeq.cols();
    for(int i=0;i<Aeq.rows();++i){
        for(int j=0;j<Aeq.cols();++j){
            A[i*nums+j]=Aeq(i,j);
        }
    }
    delta_row=Aeq.rows()*nums;
    for(int i=0;i<Aieq.rows();++i){
        for(int j=0;j<Aieq.cols();++j){
            A[delta_row+i*nums+j]=Aieq(i,j);
        }
    }
    sz=Aeq.rows()+Aieq.rows();
    real_t ub[sz],lb[sz];
    for(int i=0;i<Aeq.rows();++i){
        ub[i]=Beq(i,0);
        lb[i]=Beq(i,0);
    }
    delta_row=Aeq.rows();
    for(int i=0;i<Aieq.rows();++i){
        ub[delta_row+i]=uBieq(i,0);
        lb[delta_row+i]=lBieq(i,0);
    }

    SQProblem qp_solver(nums, sz);
    int_t nWSR = 100000;
    real_t xOpt[nums];
    qp_solver.init(H, g, A, NULL, NULL, lb, ub, nWSR);
    auto state_x=qp_solver.getPrimalSolution(xOpt);
    is_x=false;//qpOASES优化y轴
    set_AeqBeq();
    set_AieqBieq();
    for(int i=0;i<Aeq.rows();++i){//更新y轴约束
        ub[i]=Beq(i,0);
        lb[i]=Beq(i,0);
    }
    delta_row=Aeq.rows();
    for(int i=0;i<Aieq.rows();++i){
        ub[delta_row+i]=uBieq(i,0);
        lb[delta_row+i]=lBieq(i,0);
    }
    real_t yOpt[nums];
    qp_solver.hotstart(H, g, A, NULL, NULL, lb, ub, nWSR);
    auto state_y=qp_solver.getPrimalSolution(yOpt);
    // if(state_x==SUCCESSFUL_RETURN&&state_y==SUCCESSFUL_RETURN){
    //     ROS_WARN("QP solved !");
    // }
    // else{
    //     ROS_WARN("QP not solved !");
    // }
    vistraj(xOpt,yOpt);
}

void bezier::vistraj(real_t* xopt,real_t* yopt){
    visualization_msgs::Marker traj;
    traj.header.frame_id="map";
    traj.header.stamp=ros::Time::now();
    traj.ns="traj";
    traj.id=0;
    traj.type=visualization_msgs::Marker::SPHERE_LIST;
    traj.action=visualization_msgs::Marker::ADD;

    traj.scale.x=0.5;
    traj.scale.y=0.5;
    traj.scale.z=0.5;
    traj.pose.orientation.x=0.0;
    traj.pose.orientation.y=0.0;
    traj.pose.orientation.z=0.0;
    traj.pose.orientation.w=1.0;
    traj.color.a=1.0;
    traj.color.r=1.0;
    traj.color.g=0.0;
    traj.color.b=0.0;

    for(int i=0;i<box_list.size();++i){
        double delta=times[i]/500;
        for(double t=0;t<=times[i];t+=delta){
            traj.points.push_back(getBezierPos(i,t,xopt,yopt));            
        }
    }

    traj_pub.publish(traj);
}
geometry_msgs::Point bezier::getBezierPos(const int& seg,const double& t,real_t* xopt,real_t* yopt){
    geometry_msgs::Point pt;
    double x_temp=0.0,y_temp=0.0;
    int delta=(traj_order+1)*seg;
    for(int i=0;i<=traj_order;++i){
        double temp=Bernstein_base(i,t,times[seg]);
        x_temp+=(xopt[delta+i]*temp);
        y_temp+=(yopt[delta+i]*temp);
    }
    pt.x=x_temp;
    pt.y=y_temp;
    pt.z=0;
    return pt;
}

double bezier::Bernstein_base(const int& i,const double& t,const double& time_i){
    double C_i=factorial(traj_order)/(factorial(i)*factorial(traj_order-i));
    return time_i*C_i*pow(t/time_i,i)*pow(1-(t/time_i),traj_order-i);
}
double bezier::factorial(const int& n){
    double res=1.0;
    for(int i=n;i>=1;--i){
        res*=i;
    }
    return res;
}

void bezier::set_AeqBeq(){
    int box_nums=box_list.size();
    int cols=box_nums*(traj_order+1);
    Aeq=MatrixXd::Zero(3*(box_nums-1)+6,cols);//等式约束包括起点和终点的p,v,a以及中间点的连续约束
    Beq=MatrixXd::Zero(3*(box_nums-1)+6,1);
    double p_start=0.0,v_start=0.0,a_start=0.0;
    double p_end=0.0,v_end=0.0,a_end=0.0;
    if(is_x){
        p_start=start(0);
        p_end=goal(0);
        v_start=start_v(0);
        v_end=goal_v(0);
        a_start=start_a(0);
        a_end=goal_a(0);
        //v_start,a_start以实际值为准，这里先设为0,而v_end，a_end为0即可
    }
    else{
        p_start=start(1);
        p_end=goal(1);
        v_start=start_v(1);
        v_end=goal_v(1);
        a_start=start_a(1);
        a_end=goal_a(1);
        //v_start,a_start以实际值为准，这里设为0,而v_end，a_end为0即可
    }

    //起点约束p,v,a
    Aeq(0,0)=times[0]; //p
    Aeq(1,0)=-traj_order,Aeq(1,1)=traj_order;  //v
    Aeq(2,0)=(double)(traj_order*(traj_order-1))/times[0];  //a
    Aeq(2,1)=(double)(-2*traj_order*(traj_order-1))/times[0];
    Aeq(2,2)=(double)(traj_order*(traj_order-1))/times[0];
    Beq(0,0)=p_start;
    Beq(1,0)=v_start;
    Beq(2,0)=a_start;

    //终点约束p,v,a
    Aeq(3,cols-1)=times.back();  //p
    Aeq(4,cols-1)=traj_order,Aeq(4,cols-2)=-traj_order;  //v
    Aeq(5,cols-1)=(double)(traj_order*(traj_order-1))/times.back();  //a
    Aeq(5,cols-2)=(double)(-2*traj_order*(traj_order-1))/times.back();
    Aeq(5,cols-3)=(double)(traj_order*(traj_order-1))/times.back();
    Beq(3,0)=p_end;
    Beq(4,0)=v_end;
    Beq(5,0)=a_end;

    int row=6,delta=0;
    //连续约束p,v,a
    for(int i=0;i<box_nums-1;++i){
        Aeq(row,delta+traj_order)=times[i],Aeq(row,delta+traj_order+1)=-times[i+1];//p
        ++row;
        Aeq(row,delta+traj_order-1)=-traj_order;//v
        Aeq(row,delta+traj_order)=traj_order;
        Aeq(row,delta+traj_order+1)=traj_order;
        Aeq(row,delta+traj_order+2)=-traj_order;
        ++row;
        Aeq(row,delta+traj_order-2)=(double)(traj_order*(traj_order-1))/times[i];
        Aeq(row,delta+traj_order-1)=(double)(-2*traj_order*(traj_order-1))/times[i];
        Aeq(row,delta+traj_order)=(double)(traj_order*(traj_order-1))/times[i];
        Aeq(row,delta+traj_order+1)=(double)(-traj_order*(traj_order-1))/times[i+1];
        Aeq(row,delta+traj_order+2)=(double)(2*traj_order*(traj_order-1))/times[i+1];
        Aeq(row,delta+traj_order+3)=(double)(-traj_order*(traj_order-1))/times[i+1];
        ++row;
        delta+=(traj_order+1);
    }
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/

void bezier::set_AieqBieq(){
    int box_nums=box_list.size();
    int rows=(box_nums*(traj_order+1)+box_nums*traj_order+box_nums*(traj_order-1))*1;
    int cols=box_nums*(traj_order+1);
    Aieq=MatrixXd::Zero(rows,cols);
    uBieq=MatrixXd::Zero(rows,1);
    lBieq=MatrixXd::Zero(rows,1);
    int row=0,delta=0;
    for(int i=0;i<box_nums;++i){
        for(int j=0;j<=traj_order;++j){
            Aieq(row,j+delta)=times[i];
            if(is_x){
                uBieq(row,0)=(double)box_list[i].P2(0)*resolution+resolution-resolution/2;//加或减resolution/2
                lBieq(row,0)=(double)box_list[i].P1(0)*resolution+resolution/2;  //是为了让bounding_box对位置的约束远离一点障碍物
            }                                                                    //但有时候如果将点设的很靠近box的边界，qp可能会无解
            else{                                                                //因为实际的起始点或终点被画在了box外，无法满足端点在box内的约束，除非将端点转到栅格的中心.
                uBieq(row,0)=(double)box_list[i].P2(1)*resolution+resolution-resolution/2;
                lBieq(row,0)=(double)box_list[i].P4(1)*resolution+resolution/2;
            }
            ++row;
        }
        delta+=(traj_order+1);
    }
    delta=0;
    for(int i=0;i<box_nums;++i){
        for(int j=0;j<=(traj_order-1);++j){
            Aieq(row,j+delta)=-traj_order;
            Aieq(row,j+delta+1)=traj_order;
            if(is_x){
                uBieq(row,0)=vx_max;
                lBieq(row,0)=vx_min;
            }  
            else{
                uBieq(row,0)=vy_max;
                lBieq(row,0)=vy_min;
            }   
            ++row;
        }
        delta+=(traj_order+1);
    }

    delta=0;
    for(int i=0;i<box_nums;++i){
        for(int j=0;j<=(traj_order-2);++j){
            Aieq(row,j+delta)=(double)(traj_order*(traj_order-1))/times[i];
            Aieq(row,j+delta+1)=(double)(-2*traj_order*(traj_order-1))/times[i];
            Aieq(row,j+delta+2)=(double)(traj_order*(traj_order-1))/times[i];
            if(is_x){
                uBieq(row,0)=ax_max;
                lBieq(row,0)=ax_min;
            }
            else{
                uBieq(row,0)=ay_max;
                lBieq(row,0)=ay_min;
            }  
            ++row;
        }
        delta+=(traj_order+1);
    }
}

void bezier::set_MQMlist(){
    if(traj_order<4||traj_order>12){
        ROS_WARN("traj_order ranges from 4 to 12 !");//minimum jerk 3阶导
        return;
    }
    MQM_list.clear();
    int box_nums=box_list.size();
    MatrixXd Q=MatrixXd::Zero(traj_order+1,traj_order+1);
    MatrixXd M=MatrixXd::Zero(traj_order+1,traj_order+1);
    MatrixXd MQM=MatrixXd::Zero(traj_order+1,traj_order+1);
    Time_allocate();
    //get Q
    for(int i=3;i<=traj_order;++i){
        for(int j=3;j<=traj_order;++j){
            Q(i,j)=(double)(i*(i-1)*(i-2)*j*(j-1)*(j-2))/(double)(i+j-5);
        }
    }
    M=get_M();
    MQM=M.transpose()*Q*M;
    for(int i=0;i<box_nums;++i){
        MatrixXd temp=MQM/pow(times[i],3);//3=2*k-3,k为几阶导，这里是minimum jerk 3阶导
        MQM_list.push_back(temp);
    }
}

void bezier::Time_allocate(){
    times=vector<double>(box_list.size(),200);
    vector<Vector3d> pt_list;
    pt_list.push_back(start);
    get_Overlap_center(pt_list);
    pt_list.push_back(goal);
    vis_center(pt_list);

/**
 * 方法一：简单的按最大速度进行分配
**/
    // double V_max=sqrt(pow(vx_max,2)+pow(vy_max,2))*0.08;
    // for(int i=0;i<pt_list.size()-1;++i){
    //     Vector3d dis=(pt_list[i+1]-pt_list[i])*resolution;
    //     double Distance= dis.norm();
    //     double t=Distance/V_max;
    //     times[i]=t;
    //     cout<<t<<endl;
    // }
/**
 * 方法二：按每段轨迹都加速到最大速度再减速到0进行分配 
**/
    double V_max=sqrt(pow(vx_max,2)+pow(vy_max,2));
    double A_max=sqrt(pow(ax_max,2)+pow(ay_max,2));
    double A_min=sqrt(pow(ax_min,2)+pow(ay_min,2));
    double V0=sqrt(pow(start_v(0),2)+pow(start_v(1),2));
    double Vend=sqrt(pow(goal_v(0),2)+pow(goal_v(1),2));

    // cout<<V_max<<" "<<A_max<<" "<<endl;
    // cout<<A_min<<" "<<V0<<" "<<Vend<<endl;
    for(int i=0;i<pt_list.size()-1;++i){
        Vector3d dis=(pt_list[i+1]-pt_list[i])*resolution;
        double Distance=dis.norm();
        double t=0.0;
        double acct=V_max/A_max;
        double dcct=V_max/A_min;
        double dis_acc_dcc=0.0;
        if(i==0&&V0!=0){
            acct=(V_max-V0)/A_max;
            dis_acc_dcc=(V_max+V0)*acct*0.5+V_max*dcct*0.5;
        }
        else if((i==pt_list.size()-2)&&Vend!=0){
            dcct=(V_max-Vend)/A_min;
            dis_acc_dcc=V_max*acct*0.5+(V_max+Vend)*dcct*0.5;
        }
        else{
            dis_acc_dcc=(acct+dcct)*V_max*0.5;
        }

        if(dis_acc_dcc<=Distance){
            double unif_t=(Distance-dis_acc_dcc)/V_max;
            t=acct+dcct+unif_t;
        }
        else{
            t=acct+dcct;
        }
        times[i]=t*6;
        // cout<<times[i]<<endl;
        // cout<<Distance<<endl;
    }
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/

void bezier::get_Overlap_center(vector<Vector3d>& pt_list){
    for(int i=0;i<box_list.size()-1;++i){
        Vector3d pt(0,0,0);
        double x_low=max(box_list[i].P1(0),box_list[i+1].P1(0));
        double x_up =min(box_list[i].P2(0),box_list[i+1].P2(0))+1;
        double y_low=max(box_list[i].P4(1),box_list[i+1].P4(1));
        double y_up =min(box_list[i].P2(1),box_list[i+1].P2(1))+1;
        pt(0)=(x_low+x_up)/2;
        pt(1)=(y_low+y_up)/2;
        pt(2)=0;
        pt_list.push_back(pt);
    }
}

void bezier::vis_center(const vector<Vector3d>& pt_list){
    visualization_msgs::Marker center;
    center.header.frame_id="map";
    center.header.stamp=ros::Time::now();
    center.ns="center";
    center.id=0;
    center.type=visualization_msgs::Marker::SPHERE_LIST;
    center.action=visualization_msgs::Marker::ADD;

    center.scale.x=1;
    center.scale.y=1;
    center.scale.z=1;
    center.pose.orientation.x=0.0;
    center.pose.orientation.y=0.0;
    center.pose.orientation.z=0.0;
    center.pose.orientation.w=1.0;
    center.color.a=1.0;
    center.color.r=0;
    center.color.g=1.0;
    center.color.b=1.0;    

    for(int i=0;i<pt_list.size();++i){
        geometry_msgs::Point pt;
        pt.x=pt_list[i](0);
        pt.y=pt_list[i](1);
        pt.z=pt_list[i](2);
        center.points.push_back(pt);
    }

    center_pub.publish(center);
}

MatrixXd bezier::get_M(){
    MatrixXd M=MatrixXd::Zero(traj_order+1,traj_order+1);
	switch(traj_order)
	{	
		case 0: 
		{
			M << 1;
			break;
		}
		case 1: 
		{
			M << -1,  0,
			     -1,  1;
			break;
		}
		case 2:
		{
			M << -1,  0,  0,
				 -2,  2,  0,
				  1, -2,  1;
			break;
		}
		case 3: 
		{
			M << -1,  0,  0,  0,
				 -3,  3,  0,  0,
				  3, -6,  3,  0,
				 -1,  3, -3,  1;	
			break;
		}
		case 4:
		{
			M <<  1,   0,   0,   0,  0,
				 -4,   4,   0,   0,  0,
				  6, -12,   6,   0,  0,
				 -4,  12, -12,   4,  0,
				  1,  -4,   6,  -4,  1;
			break;
		}
		case 5:
		{
			M << 1,   0,   0,   0,  0,  0,
				-5,   5,   0,   0,  0,  0,
				10, -20,  10,   0,  0,  0,
			   -10,  30, -30,  10,  0,  0,
			     5, -20,  30, -20,  5,  0,
			    -1,   5, -10,  10, -5,  1;
			break;
		}
		case 6:
		{	
			M << 1,   0,   0,   0,   0,  0,  0,
				-6,   6,   0,   0,   0,  0,  0,
				15, -30,  15,   0,   0,  0,  0,
			   -20,  60, -60,  20,   0,  0,  0,
			    15, -60,  90, -60,  15,  0,  0,
			    -6,  30, -60,  60, -30,  6,  0,
			     1,  -6,  15, -20,  15, -6,  1;
			break;
		}
		case 7:
		{
			M << 1,    0,    0,    0,    0,   0,   0,   0,
			    -7,    7,    0,    0,    0,   0,   0,   0,
			    21,   42,   21,    0,    0,   0,   0,   0,
			   -35,  105, -105,   35,    0,   0,   0,   0, 
			    35, -140,  210, -140,   35,   0,   0,   0,
			   -21,  105, -210,  210, -105,  21,   0,   0,
			     7,  -42,  105, -140,  105, -42,   7,   0,
			    -1,    7,  -21,   35,  -35,  21,  -7,   1;
			break;
		}
		case 8:
		{
			M << 1,    0,    0,    0,    0,    0,   0,   0,   0,
			    -8,    8,    0,    0,    0,    0,   0,   0,   0,
			    28,  -56,   28,    0,    0,    0,   0,   0,   0,
			   -56,  168, -168,   56,    0,    0,   0,   0,   0, 
			    70, -280,  420, -280,   70,    0,   0,   0,   0,
			   -56,  280, -560,  560, -280,   56,   0,   0,   0,
			    28, -168,  420, -560,  420, -168,  28,   0,   0,
			    -8,   56, -168,  280, -280,  168, -56,   8,   0,
			     1,   -8,   28,  -56,   70,  -56,  28,  -8,   1;
			break;
		}
		case 9:
		{
			M << 1,    0,     0,     0,     0,    0,    0,     0,     0,    0,
			    -9,    9,     0,     0,     0,    0,    0,     0,     0,    0, 
			    36,  -72,    36,     0,     0,    0,    0,     0,     0,    0, 
			   -84,  252,  -252,    84,     0,    0,    0,     0,     0,    0, 
			   126, -504,   756,  -504,   126,    0,    0,     0,     0,    0,
			  -126,  630, -1260,  1260,  -630,  126,    0,     0,     0,    0,
			    84, -504,  1260, -1680,  1260, -504,   84,     0,     0,    0,
			   -36,  252,  -756,  1260, -1260,  756, -252,    36,     0,    0,
			     9,  -72,   252,  -504,   630, -504,  252,   -72,     9,    0,
			    -1,    9,   -36,    84,  -126,  126,  -84,    36,    -9,    1;
			break;
		}
		case 10:
		{
			M <<  1,     0,     0,     0,      0,     0,    0,     0,     0,    0,   0,
			    -10,    10,     0,     0,      0,     0,    0,     0,     0,    0,   0,
			     45,   -90,    45,     0,      0,     0,    0,     0,     0,    0,   0,
			   -120,   360,  -360,   120,      0,     0,    0,     0,     0,    0,   0,
			    210,  -840,  1260,  -840,    210,     0,    0,     0,     0,    0,   0,
			   -252,  1260, -2520,  2520,  -1260,   252,    0,     0,     0,    0,   0,
			    210, -1260,  3150, -4200,   3150, -1260,  210,     0,     0,    0,   0,
			   -120,  840,  -2520,  4200,  -4200,  2520, -840,   120,     0,    0,   0,
			     45, -360,   1260, -2520,   3150, -2520, 1260,  -360,    45,    0,   0,
			    -10,   90,   -360,   840,  -1260,  1260, -840,   360,   -90,   10,   0,
			      1,  -10,     45,  -120,    210,  -252,  210,  -120,    45,  -10,   1;
			break;
		}
		case 11:
		{
			M <<  1,     0,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
			    -11,    11,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
			     55,  -110,   55,      0,      0,      0,     0,     0,     0,    0,   0,  0,
			   -165,   495, -495,    165,      0,      0,     0,     0,     0,    0,   0,  0,
			    330, -1320, 1980,  -1320,    330,      0,     0,     0,     0,    0,   0,  0,
			   -462,  2310, -4620,  4620,  -2310,    462,     0,     0,     0,    0,   0,  0,
			    462, -2772,  6930, -9240,   6930,  -2772,   462,     0,     0,    0,   0,  0,
			   -330,  2310, -6930, 11550, -11550,   6930, -2310,   330,     0,    0,   0,  0,
			    165, -1320,  4620, -9240,  11550,  -9240,  4620, -1320,   165,    0,   0,  0,
			    -55,   495, -1980,  4620,  -6930,   6930, -4620,  1980,  -495,   55,   0,  0,
			     11,  -110,   495, -1320,   2310,  -2772,  2310, -1320,   495, -110,  11,  0,
			     -1,    11,   -55,   165,   -330,    462,  -462,   330,  -165,   55, -11,  1;
			break;
		}
		case 12:
		{
			M <<  1,     0,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
			    -12,    12,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
			     66,  -132,    66,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
			   -220,   660,  -660,    220,      0,      0,     0,     0,     0,    0,    0,   0,   0,
			    495, -1980,  2970,  -1980,    495,      0,     0,     0,     0,    0,    0,   0,   0, 
			   -792,  3960, -7920,   7920,  -3960,    792,     0,     0,     0,    0,    0,   0,   0,
			    924, -5544, 13860, -18480,  13860,  -5544,   924,     0,     0,    0,    0,   0,   0,
			   -792,  5544,-16632,  27720, -27720,  16632, -5544,   792,     0,    0,    0,   0,   0,
			    495, -3960, 13860, -27720,  34650, -27720, 13860, -3960,   495,    0,    0,   0,   0,
			   -220,  1980, -7920,  18480, -27720,  27720,-18480,  7920, -1980,  220,    0,   0,   0,
			     66,  -660,  2970,  -7920,  13860, -16632, 13860, -7920,  2970, -660,   66,   0,   0,
			    -12,   132,  -660,   1980,  -3960,   5544, -5544,  3960, -1980,  660, -132,  12,   0,
			      1,   -12,    66,   -220,    495,   -792,   924,  -792,   495, -220,   66, -12,   1;
			break;
		}
        default:
        {
            ROS_WARN("traj_order out of range of M !");
            break;
        }
	}
    return M;
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
int bezier::delete_box(const bounding_box& box_last,const bounding_box& box_now,const geometry_msgs::PoseArray::Ptr& path){
    int n=path->poses.size();
    bool has_box_last=false,has_box_now=false;
    for(int i=n-1;i>=0;--i){
        int Px=path->poses[i].position.x,Py=path->poses[i].position.y;
        if(!has_box_last){
            if(Px>=box_last.P1(0)&&Px<=box_last.P2(0)&&Py>=box_last.P4(1)&&Py<=box_last.P2(1)&&
                            (Px<box_now.P1(0)||Px>box_now.P2(0)||Py<box_now.P4(1)||Py>box_now.P2(1)))
            {
                has_box_last=true;//有自己独一二的点
            }
        }
        if(!has_box_now){
            if(Px>=box_now.P1(0)&&Px<=box_now.P2(0)&&Py>=box_now.P4(1)&&Py<=box_now.P2(1)&&
                            (Px<box_last.P1(0)||Px>box_last.P2(0)||Py<box_last.P4(1)||Py>box_last.P2(1)))
            {
                has_box_now=true;//有自己独一二的点
            } 
        }
        if(has_box_last&&has_box_now){
            return 0;
        }      
    }
    if(has_box_last&&!has_box_now)
        return 1;
    
    return 2;
}

bool bezier::is_in_box(const geometry_msgs::Point& pt){
    if(pt.x>=(double)box_last.P1(0)&&pt.x<=(double)box_last.P2(0)&&pt.y>=(double)box_last.P4(1)&&pt.y<=(double)box_last.P2(1))
        return true;
    return false;
}

geometry_msgs::Point bezier::trans2pt(const Vector3i& p){
    geometry_msgs::Point pt;
    pt.x=p(0);
    pt.y=p(1);
    pt.z=p(2);
    return pt;
}

void bezier::visual_box(const vector<bounding_box>& box_list){
    for(int i=0;i<box_list_vis.markers.size();++i)
        box_list_vis.markers[i].action = visualization_msgs::Marker::DELETE;
    
    box_pub.publish(box_list_vis);
    box_list_vis.markers.clear();

    visualization_msgs::Marker box;
    box.header.frame_id="map";
    box.ns="box";
    box.type=visualization_msgs::Marker::CUBE;
    box.action=visualization_msgs::Marker::ADD;
    box.header.stamp = ros::Time::now();

    box.pose.orientation.x=0.0;
    box.pose.orientation.y=0.0;
    box.pose.orientation.z=0.0;
    box.pose.orientation.w=1.0;
    
    box.color.a = 0.3;
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 1.0;

    int idx=0;
    for(int i=0;i<box_list.size();++i){
        box.id=idx;

        box.pose.position.x=(box_list[i].P2(0)+box_list[i].P1(0))*resolution/2+resolution/2;
        box.pose.position.y=(box_list[i].P2(1)+box_list[i].P4(1))*resolution/2+resolution/2;
        box.pose.position.z=0;

        box.scale.x=(box_list[i].P2(0)-box_list[i].P1(0)+1)*resolution;
        box.scale.y=(box_list[i].P2(1)-box_list[i].P4(1)+1)*resolution;
        box.scale.z=0.5;
        box_list_vis.markers.push_back(box);
        ++idx;
    }

    box_pub.publish(box_list_vis);
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
void bezier::inflate_box(bounding_box& box){
    if(map->data[box.P1(1)*width_grid+box.P1(0)]){
        ROS_WARN("The initial point is occupied!");
        return;
    }
    bool has_P2_x=false,has_P2_y=false,has_P3_x=false,has_P3_y=false,stop=false;
    Vector3i temp_P2=box.P2;
    Vector3i temp_P3=box.P3;
    // cout<<1<<endl;
    for(int i=0;i<max_inflate_iter;++i){
        // cout<<i<<endl;
        // cout<<temp_P2(0)<<" "<<temp_P2(1)<<" "<<temp_P3(0)<<" "<<temp_P3(1)<<endl;
        if(!has_P2_x||!has_P2_y){
            if(!has_P2_x){
                temp_P2(1)+=1;
                for(int j=temp_P2(0);j>=temp_P3(0);--j){
                    if(map->data[temp_P2(1)*width_grid+j]||temp_P2(1)>=height_grid){
                        has_P2_x=true;
                        break;
                    }
                }
                if(has_P2_x){
                    temp_P2(1)-=1;
                }
            }

            if(!has_P2_y){
                temp_P2(0)+=1;
                for(int j=temp_P2(1);j>=temp_P3(1);--j){
                    if(map->data[j*width_grid+temp_P2(0)]||temp_P2(0)>=width_grid){
                        has_P2_y=true;
                        break;
                    }
                }
                if(has_P2_y){
                    temp_P2(0)-=1;
                }
            }            
        }
        if(!has_P3_x||!has_P3_y){
            if(!has_P3_x){
                temp_P3(1)-=1;
                for(int j=temp_P3(0);j<=temp_P2(0);++j){
                    if(temp_P3(1)<0){
                        has_P3_x=true;
                        break;
                    }
                    if(map->data[temp_P3(1)*width_grid+j]){
                        has_P3_x=true;
                        break;
                    }
                }
                if(has_P3_x){
                    temp_P3(1)+=1;
                }
            }
            if(!has_P3_y){
                temp_P3(0)-=1;
                for(int j=temp_P3(1);j<=temp_P2(1);++j){
                    if(temp_P3(0)<0){
                        has_P3_y=true;
                        break;
                    }
                    if(map->data[j*width_grid+temp_P3(0)]||temp_P3(0)<0){
                        has_P3_y=true;
                        break;
                    }
                }
                if(has_P3_y){
                    temp_P3(0)+=1;
                }
            }
        }

        if(has_P2_x&&has_P2_y&&has_P3_x&&has_P3_y)
            break;
    }

    box.P2(1)=temp_P2(1);
    box.P1(1)=temp_P2(1);

    box.P2(0)=temp_P2(0);
    box.P4(0)=temp_P2(0);

    box.P3(1)=temp_P3(1);
    box.P4(1)=temp_P3(1);

    box.P3(0)=temp_P3(0);
    box.P1(0)=temp_P3(0);
    // cout<<2<<endl;
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
bool bezier::is_contain(const bounding_box& box_last,const bounding_box& box_now){
    if(box_last.P1(0)<=box_now.P1(0)&&box_last.P1(1)>=box_now.P1(1)&&
        box_last.P2(0)>=box_now.P2(0)&&box_last.P2(1)>=box_now.P2(1)&&
        box_last.P3(0)<=box_now.P3(0)&&box_last.P3(1)<=box_now.P3(1)&&
        box_last.P4(0)>=box_now.P4(0)&&box_last.P4(1)<=box_now.P4(1))
        return true;
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "B_traj");
    ros::NodeHandle nh("~");
    nh.param("max_inflate_iter", _max_inflate_iter, 50);
    nh.param("B_traj/order",     _traj_order, 4);
    nh.param("B_traj/vx_max", _vx_max, 30.0);
    nh.param("B_traj/vx_min", _vx_min, -30.0);
    nh.param("B_traj/vy_max", _vy_max, 30.0);
    nh.param("B_traj/vy_min", _vy_min, -30.0);
    nh.param("B_traj/ax_max", _ax_max, 30.0);
    nh.param("B_traj/ax_min", _ax_min, -30.0);
    nh.param("B_traj/ay_max", _ay_max, 30.0);
    nh.param("B_traj/ay_min", _ay_min, -30.0);

    ROS_WARN("B_traj program online!");
    bezier b_traj;
    ros::spin();
    return 0;
}