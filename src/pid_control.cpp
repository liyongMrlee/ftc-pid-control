#include "pid_control/pid_control.h"

PidControl::PidControl()
    : goal_reach(true),
      new_path_flag(false),
      control_thread_(NULL),
      control_period(0.05)
{
    path_sub_ = node_.subscribe<nav_msgs::Path>("global_path", 1, boost::bind(&PidControl::pathCallBack, this, _1));
    cmd_pub_ = node_.advertise<std_msgs::String>("control_status",1,true);
    vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    trajectory_pub_ = node_.advertise<nav_msgs::Path>("trajectory", 1, true);
    local_path_pub_ = node_.advertise<nav_msgs::Path>("local_path", 1, true);
    smooth_path_pub_ = node_.advertise<nav_msgs::Path>("smooth_path", 1, true);
}

PidControl::~PidControl()
{
}

void PidControl::pathCallBack(const nav_msgs::Path::ConstPtr &path)
{
    ROS_INFO("I have recived path");
    global_path = *path;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    vel_pub_.publish(cmd_vel); // 发布速度
    // global_path.poses.clear();
    ROS_INFO("delete old fuzzy pid modle");
    // pathSmoother(path,global_path);
    ROS_INFO("path size = %d,global path size = %d", path->poses.size(), global_path.poses.size());
    goal_reach = false;
    new_path_flag = true;

    if(control_thread_ != NULL)
    {
        control_thread_->join();
        delete control_thread_;
        control_thread_ = NULL;
    }
    control_thread_ = new boost::thread(boost::bind(&PidControl::FuzzyPidControl, this, control_period));
}

void PidControl::pathSmoother(const nav_msgs::Path::ConstPtr &original_path,nav_msgs::Path& smooth_path)
{
    ROBOT_POSE pose;
    getRobotPose(pose);
    // ROS_INFO("run to here 0");
    nav_msgs::Path path ;
    path= *original_path;
    // pose.x = path.poses[0].pose.position.x;
    // pose.y = path.poses[0].pose.position.y;
    pose.yaw = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,path.poses[2].pose.position.x - path.poses[1].pose.position.x);
    std::vector<Eigen::Vector2f> trajectory_vec;
    int point_index = 0;
    nav_msgs::Path local_path;
    //  ROS_INFO("run to here 1");
    while(!isReachGoalVirtual(path, pose))
    {
        CroppingPath(path, local_path, pose, point_index);
        geometry_msgs::Twist cmd_vel;
        computeVelocity(local_path, pose,cmd_vel);
        // ROS_INFO("pose x = %f,y = %f,cmd_x = %f,cmd_z = %f",pose.x,pose.y,pose.yaw,cmd_vel.linear.x,cmd_vel.angular.z);
        float vel_line = cmd_vel.linear.x;
        float vel_angle = cmd_vel.angular.z;
        float new_pose_x = pose.x + vel_line*cos(pose.yaw)*control_period;
        float new_pose_y = pose.y + vel_line*sin(pose.yaw)*control_period;
        float new_node_yaw = range_angle_PI(pose.yaw + vel_angle*control_period);
        trajectory_vec.push_back(Eigen::Vector2f(new_pose_x,new_pose_y));
        pose.x = new_pose_x;
        pose.y = new_pose_y;
        pose.yaw = new_node_yaw;

        /*local_path.header.stamp = ros::Time::now();
        local_path.header.frame_id = "map";
        local_path_pub_.publish(local_path); // 发布机器人局部路径

        smooth_path.header.stamp = ros::Time::now();
        smooth_path.header.frame_id = "map";
        geometry_msgs::PoseStamped smooth_path_node;
        smooth_path_node.pose.position.x = pose.x;
        smooth_path_node.pose.position.y = pose.y;
        smooth_path.poses.push_back(smooth_path_node); // 添加平滑路径的第一个点
        smooth_path_pub_.publish(smooth_path);
        // usleep(50*1000);*/
    }
    ROS_INFO("trajectory vec size = %d",trajectory_vec.size());
    smooth_path.poses.clear();
    geometry_msgs::PoseStamped smooth_path_node_start;
    smooth_path_node_start.pose.position.x = trajectory_vec[0](0);
    smooth_path_node_start.pose.position.y = trajectory_vec[0](1);
    smooth_path.poses.push_back(smooth_path_node_start); // 添加平滑路径的第一个点
    int smooth_path_index = 0;
    for(int i = 1;i<trajectory_vec.size();i++)
    {
        float node_x = smooth_path.poses.back().pose.position.x;
        float node_y = smooth_path.poses.back().pose.position.y;
        float diff_x = trajectory_vec[i](0) - node_x;
        float diff_y = trajectory_vec[i](1) - node_y;
        float distance  = sqrt(diff_x * diff_x + diff_y * diff_y);
        if(distance > 0.03)
        {
            geometry_msgs::PoseStamped smooth_path_node;
            smooth_path_node.pose.position.x = trajectory_vec[i](0);
            smooth_path_node.pose.position.y = trajectory_vec[i](1);
            // ROS_INFO("smooth path node x = %f, y = %f,index = %d",node_x,node_y,smooth_path_index++);
            smooth_path.poses.push_back(smooth_path_node); // 添加平滑路径的中间点
        }
    }
    geometry_msgs::PoseStamped smooth_path_node_end;
    smooth_path_node_end.pose.position.x = trajectory_vec.back()(0);
    smooth_path_node_end.pose.position.y = trajectory_vec.back()(1);
    smooth_path.poses.push_back(smooth_path_node_end); // 添加平滑路径的最后一个点

    smooth_path.header.stamp = ros::Time::now();
    smooth_path.header.frame_id = "map";
    smooth_path_pub_.publish(smooth_path);
}

void PidControl::computeVelocity(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist &cmd_vel)
{
    CorrectFoundation _correctValue;
    _correctValue = getCorrectFoundation(path, pose);
    float diff_angle = range_angle_PI(_correctValue.angle_robot2path - pose.yaw);
    /*if(fabs(diff_angle) > 1.2)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = diff_angle > 0 ? -VEL_ANGLE_ROTATION : VEL_ANGLE_ROTATION;
    }
    else*/
    {
        cmd_vel.linear.x = MAX_VEL_LINE;
        cmd_vel.angular.z = diff_angle / 0.5;
        // if(fabs(_correctValue.diff_angle) < 0.01)    cmd_vel.angular.z = 0;
    }
    // ROS_INFO("vel line = %f,vel angle = %f",cmd_vel.linear.x,cmd_vel.angular.z);
}

void PidControl::FuzzyPidControl(float control_period)
{
    ros::Rate r(1.0 / control_period);
    // trajectory_path.poses.clear();
    int point_index = 0;
    rotate_to_global_path = false;
    new_path_flag = false;
    nav_msgs::Path local_path;
    cmd_vel_last.linear.x = 0;
    cmd_vel_last.angular.z = 0;
    ROS_INFO("Preapre to pub vel!");
    FuzzyPidModle* fuzzyPidmodle = new FuzzyPidModle(1.0 / control_period);
    while (!goal_reach)
    {
        ROBOT_POSE pose;
        if (getRobotPose(pose))
        {
            geometry_msgs::PoseStamped trajectory_node;
            trajectory_node.pose.position.x = pose.x;
            trajectory_node.pose.position.y = pose.y;
            trajectory_path.poses.push_back(trajectory_node); // 添加机器人运动实时轨迹

            CroppingPath(global_path, local_path, pose, point_index);
            geometry_msgs::Twist cmd_vel;
            robotNavigation(local_path, pose, cmd_vel,fuzzyPidmodle);

            vel_pub_.publish(cmd_vel); // 发布速度
            trajectory_path.header.stamp = ros::Time::now();
            trajectory_path.header.frame_id = "map";
            trajectory_pub_.publish(trajectory_path); // 发布机器人实时轨迹
            local_path.header.stamp = ros::Time::now();
            local_path.header.frame_id = "map";
            local_path_pub_.publish(local_path); // 发布机器人局部路径
            r.sleep();
        }
        if(new_path_flag)     break;
    }
    if(fuzzyPidmodle)   delete fuzzyPidmodle;
    //通知规划器，已经到达终点
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    vel_pub_.publish(cmd_vel); // 发布速度
    ROS_INFO("goal reach!!!");
    std_msgs::String control_status;
    control_status.data = "goalreach";
    cmd_pub_.publish(control_status);
    r.sleep();
}

bool PidControl::getRobotPose(ROBOT_POSE &pose)
{
    try
    {
        location_listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(2.0));
        location_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), location_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_INFO("can not get base_footprint to map!");
        return false;
    }
    pose.x = static_cast<float>(location_transform.getOrigin().x());
    pose.y = static_cast<float>(location_transform.getOrigin().y());
    pose.yaw = tf::getYaw(location_transform.getRotation());

    return true;
}

void PidControl::CroppingPath(nav_msgs::Path global_path, nav_msgs::Path &local_path, ROBOT_POSE pose, int &last_start_index)
{
    int end_index = 0;
    int new_start_index = 1000;
    local_path.poses.clear();
    // ROS_INFO("run to here 0");
    getMinDistanceIndex(global_path,pose,last_start_index,new_start_index);         //找距离机器人最近的点作为局部路径的最新起点
    // ROS_INFO("run to here 1,new start index = %d",new_start_index);
    getMaxDistanceIndex(global_path,new_start_index,end_index);                     //找局部路径的最远点
    // ROS_INFO("run to here 2,end_index = %d",end_index);
    local_path.poses.insert(local_path.poses.end(), global_path.poses.begin() + new_start_index, global_path.poses.begin() + end_index);
    last_start_index = new_start_index;
}

void PidControl::getMinDistanceIndex(nav_msgs::Path path,ROBOT_POSE pose,int start_index, int& min_distance_index)
{
    float min_distance = 1000;
    // ROS_INFO("----------path pose size = %d",path.poses.size());
    // ROS_INFO("robot pose x = %f,y = %f",pose.x,pose.y);
    for (int i = start_index; i < path.poses.size(); i++)
    {
        float diff_x = pose.x - path.poses[i].pose.position.x;
        float diff_y = pose.y - path.poses[i].pose.position.y;
        float distance = sqrt(diff_x * diff_x + diff_y * diff_y);
        // ROS_INFO("path node index = %d,x = %f,y = %f,distance = %f",i,path.poses[i].pose.position.x,path.poses[i].pose.position.y,distance);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_distance_index = i;
            // ROS_INFO("************min distance = %f,index = %d",min_distance,min_distance_index);
        }
    }
}

void PidControl::getMaxDistanceIndex(nav_msgs::Path global_path,int start_index, int &max_distance_index)
{
    float path_lenght = 0;
    for (int i = start_index + 1; i < global_path.poses.size(); i++)
    {
        float diff_x = global_path.poses[i].pose.position.x - global_path.poses[i - 1].pose.position.x;
        float diff_y = global_path.poses[i].pose.position.y - global_path.poses[i - 1].pose.position.y;
        float distance = sqrt(diff_x * diff_x + diff_y * diff_y);
        path_lenght += distance;
        if (path_lenght > MAX_DISTANCE)
        {
            max_distance_index = i - 1;
            return;
        }
    }
    max_distance_index = global_path.poses.size() - 1;
}

float PidControl::getAngleRobot2path(nav_msgs::Path path, ROBOT_POSE pose)
{
    Eigen::Vector2f vector_sum(0,0);
    for (int i = 0; i < path.poses.size(); i++)
    {
        float path_node_x = path.poses[i].pose.position.x;
        float path_node_y = path.poses[i].pose.position.y;
        Eigen::Vector2f robot2node_vector(path_node_x - pose.x,path_node_y - pose.y);
        vector_sum += robot2node_vector;
    }
    float angle_robot2path = atan2(vector_sum(1),vector_sum(0));
    return angle_robot2path;

}

float PidControl::getPathAngle(nav_msgs::Path path)
{
    float node_first_x = path.poses[0].pose.position.x;
    float node_first_y = path.poses[0].pose.position.y;
    Eigen::Vector2f vector_sum(0,0);
    for (int i = 1; i < path.poses.size(); i++)
    {
        float path_node_x = path.poses[i].pose.position.x;
        float path_node_y = path.poses[i].pose.position.y;
        Eigen::Vector2f pathangle2node_vector(path_node_x - node_first_x,path_node_y - node_first_y);
        vector_sum += pathangle2node_vector;
    }
    return atan2(vector_sum(1),vector_sum(0));
}

bool PidControl::rotateToGlobalPath(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist &cmd_vel)
{
    float angle_path = getPathAngle(path);
    float diff_angle = range_angle_PI(angle_path - pose.yaw);
    if (diff_angle > -ERROR_ANGLE && diff_angle < ERROR_ANGLE)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return true;
    }
    else
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = diff_angle > 0 ? VEL_ANGLE_ROTATION : -VEL_ANGLE_ROTATION;
        return false;
    }
}

void PidControl::robotNavigation(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist &cmd_vel,FuzzyPidModle* fuzzyPidmodle)
{
    if (!rotate_to_global_path)      // 初始化，机器人转向局部路径方向
    {
        rotate_to_global_path = rotateToGlobalPath(path, pose, cmd_vel);
        if (rotate_to_global_path)
            ROS_INFO("rotate reach!!");
    }
    else
    {
        // return;
        // ros::Time current_time = ros::Time::now();
        // goal_reach = driveToward(path, pose, cmd_vel,fuzzyPidmodle);
        goal_reach = driveTowardByFtc(path, pose, cmd_vel,fuzzyPidmodle);
        // ROS_INFO("pid compute cost time = %f\n",(ros::Time::now() - current_time).toSec());
    }
}

bool PidControl::isReachGoal(nav_msgs::Path path, ROBOT_POSE pose)
{
    //  ROS_INFO("run to here 2");
    float goal_x = path.poses.back().pose.position.x;
    float goal_y = path.poses.back().pose.position.y;
    //  ROS_INFO("run to here 3");
    if (fabs(goal_x - pose.x) < ERROR_GOAL && fabs(goal_y - pose.y) < ERROR_GOAL)
        return true;
    return false;
}

bool PidControl::isReachGoalVirtual(nav_msgs::Path path, ROBOT_POSE pose)
{
    //  ROS_INFO("run to here 2");
    float goal_x = path.poses.back().pose.position.x;
    float goal_y = path.poses.back().pose.position.y;
    //  ROS_INFO("run to here 3");
    if (fabs(goal_x - pose.x) < 2*ERROR_GOAL && fabs(goal_y - pose.y) < 2*ERROR_GOAL)
        return true;
    return false;
}

bool PidControl::driveToward(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist &cmd_vel,FuzzyPidModle* fuzzyPidmodle)
{
    if (isReachGoal(path, pose))
    {
        ROS_INFO("goal reach!!");
        return true;
    } // 到达目标点，则返回成功
    CorrectFoundation _correctValue;
    ROBOT_POSE check_pose = {pose.x + FRONT_CHECK_DIS * cos(pose.yaw), pose.y + FRONT_CHECK_DIS * sin(pose.yaw), pose.yaw};
    if (path.poses.size()< 8)
    {   
        _correctValue = getCorrectFoundation(path, pose); // 靠近终点时，使用机器人位姿
        //快到终点时候，采用ftc的方法的横向控制
        float diff_angle = range_angle_PI(_correctValue.angle_robot2path - pose.yaw);
        cmd_vel.linear.x = VEL_LINE_HALF;
        cmd_vel.angular.z = diff_angle / 1.0;
        if(fabs(_correctValue.diff_angle) < 0.05)    cmd_vel.angular.z = 0;
    }
    else
    {
        int start_index = 0;
        getMinDistanceIndex(path,check_pose,0,start_index);
        nav_msgs::Path new_path;
        new_path.poses.insert(new_path.poses.end(),path.poses.begin() + start_index,path.poses.end());
        _correctValue = getCorrectFoundation(new_path, check_pose); // 还未靠近终点时，使用前置探测点
        if (fabs(_correctValue.diff_angle) > 1.6)
        {
            rotate_to_global_path = false;
            return false;
        }
        else
        {
            // ROS_INFO("diff_angle  = %f,diff_distance = %f,path angle = %f",
            //         _correctValue.diff_angle, _correctValue.diffDistace_to_line, _correctValue.angle_path);
            // ROS_INFO("robot pose x = %f,y = %f,yaw = %f",pose.x,pose.y,pose.yaw);
            // TODO：调用模糊PID模块，进行纠偏
            if (fabs(_correctValue.diff_angle) < 0.05 && fabs(_correctValue.diffDistace_to_line) < 0.03)
                _correctValue.diffDistace_to_line = 0;

            cmd_vel.linear.x = MAX_VEL_LINE;
            float vel_y = MAX_VEL_LINE * sin(_correctValue.diff_angle); // 速度关于路径垂直方向的分量
            // float delta_vy = fuzzyPidmodle->fuzzyPidOutputComput(_correctValue.diffDistace_to_line);
            float delta_vy = fuzzyPidmodle->PidOutputComput(_correctValue.diffDistace_to_line);
            ROS_INFO("vy = %f,delta_vy = %f", vel_y, delta_vy);
            vel_y -= delta_vy;
            // ROS_INFO("new vy0 = %f", vel_y);
            if (fabs(vel_y) > MAX_VEL_LINE)
            {
                vel_y = MAX_VEL_LINE * fabs(vel_y) / vel_y * sin(1.4);
                cmd_vel.linear.x = VEL_LINE_HALF;
            }
            // ROS_INFO("new vy1 = %f", vel_y);
            // ROS_INFO("asin th = %f", asin(vel_y / MAX_VEL_LINE)); // 纠正后的横向速度分量
            float new_theta = asin(vel_y / MAX_VEL_LINE);
            // if(fabs(new_theta) > 1)   new_theta = new_theta / fabs(new_theta);
            float expect_angle = new_theta + _correctValue.angle_path;
            float delta_th = range_angle_PI(expect_angle - pose.yaw); // 当前机器人方向与期望机器人方向的夹角，需要机器人快速到达该方向
            // ROS_INFO("current angle = %f,increm_th = %f,expect angle = %f,diff_current_and expect = %f", pose.yaw, new_theta, expect_angle, delta_th);
            if (fabs(delta_th) < MAX_VEL_ANGLE)
            {
                cmd_vel.angular.z = delta_th / 0.5; // 期望0.5s到达该期望方向，将角度转化为角速度
            }
            else
            {
                cmd_vel.angular.z = delta_th > 0 ? MAX_VEL_ANGLE : -MAX_VEL_ANGLE;
            }
        }
    }
    float vel_left  = cmd_vel.linear.x - cmd_vel.angular.z*0.27 / 2.0;
    float vel_right = cmd_vel.linear.x + cmd_vel.angular.z*0.27 / 2.0;
    ROS_INFO("vel line = %f,vel angle = %f,vel left = %f,vel right = %f\n", 
              cmd_vel.linear.x, cmd_vel.angular.z,vel_left,vel_right);
    return false;
}

float PidControl::getLocalPathDistance(nav_msgs::Path path)
{
    float total_distance = 0;
    for(int i = 0; i < path.poses.size()-1;i++)
    {
        float diff_x = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        float diff_y = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        total_distance += sqrt(diff_x * diff_x + diff_y * diff_y);
    }
    return total_distance;
}

float PidControl::getLineVel(float local_path_len)
{
    double vel = local_path_len / SIM_TIME;             //ftc方法获得线速度
    if(vel > cmd_vel_last.linear.x)     
       vel = getMidValue(cmd_vel_last.linear.x + ACC_LINE*control_period,MAX_VEL_LINE,MIN_VEL_LINE);
    else if(vel < cmd_vel_last.linear.x + ACC_LINE*control_period)
        vel = getMidValue(cmd_vel_last.linear.x - ACC_LINE*control_period,MAX_VEL_LINE,MIN_VEL_LINE);
    return vel;
}

void PidControl::getSmoothVelocity(geometry_msgs::Twist &cmd_vel)
{
    float vel_left  = cmd_vel.linear.x - cmd_vel.angular.z*0.27 / 2.0;
    float vel_right = cmd_vel.linear.x + cmd_vel.angular.z*0.27 / 2.0;
    ROS_INFO("origin vel line = %f,vel angle = %f,vel left = %f,vel right = %f", 
              cmd_vel.linear.x, cmd_vel.angular.z,vel_left,vel_right);
    //判断左右轮速是否大于最大轮速
    if(vel_left > MAX_WHEEL_SPEED)
    {
        vel_left = MAX_WHEEL_SPEED;
        vel_right = vel_left + cmd_vel.angular.z * ODOM_L;
    }
    else if(vel_right > MAX_WHEEL_SPEED)
    {
        vel_right = MAX_WHEEL_SPEED;
        vel_left = vel_right - cmd_vel.angular.z * ODOM_L;
    }
    cmd_vel.linear.x  = (vel_right + vel_left) / 2.0;
    cmd_vel.angular.z = (vel_right - vel_left) / ODOM_L;
    ROS_INFO("new vel line = %f,vel angle = %f,vel left = %f,vel right = %f\n", 
                cmd_vel.linear.x, cmd_vel.angular.z,vel_left,vel_right);
}


bool PidControl::driveTowardByFtc(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist &cmd_vel,FuzzyPidModle* fuzzyPidmodle)
{
    if (isReachGoal(path, pose))
    {
        ROS_INFO("goal reach!!");
        return true;
    } // 到达目标点，则返回成功
    CorrectFoundation _correctValue;
    ROBOT_POSE check_pose = {pose.x + FRONT_CHECK_DIS * cos(pose.yaw), pose.y + FRONT_CHECK_DIS * sin(pose.yaw), pose.yaw};
    float path_length = getLocalPathDistance(path);
    cmd_vel.linear.x = getLineVel(path_length);


    if (path.poses.size()< 8)
    {   
        _correctValue = getCorrectFoundation(path, pose); // 靠近终点时，使用机器人位姿
        //快到终点时候，采用ftc的方法的横向控制
        float diff_angle = range_angle_PI(_correctValue.angle_robot2path - pose.yaw);
        // cmd_vel.linear.x = VEL_LINE_HALF;
        cmd_vel.angular.z = diff_angle / 0.5;
        if(fabs(_correctValue.diff_angle) < 0.05)    cmd_vel.angular.z = 0;
    }
    else
    {
        int start_index = 0;
        getMinDistanceIndex(path,check_pose,0,start_index);
        nav_msgs::Path new_path;
        new_path.poses.insert(new_path.poses.end(),path.poses.begin() + start_index,path.poses.end());
        _correctValue = getCorrectFoundation(new_path, check_pose); // 还未靠近终点时，使用前置探测点
        if (fabs(_correctValue.diff_angle) > 1.6)
        {
            rotate_to_global_path = false;
            return false;
        }
        else
        {
            // ROS_INFO("diff_angle  = %f,diff_distance = %f,path angle = %f",
            //         _correctValue.diff_angle, _correctValue.diffDistace_to_line, _correctValue.angle_path);
            // ROS_INFO("robot pose x = %f,y = %f,yaw = %f",pose.x,pose.y,pose.yaw);
            // TODO：调用模糊PID模块，进行纠偏
            if (fabs(_correctValue.diff_angle) < 0.05 && fabs(_correctValue.diffDistace_to_line) < 0.03)
                _correctValue.diffDistace_to_line = 0;

            cmd_vel.linear.x = MAX_VEL_LINE;
            float vel_y = MAX_VEL_LINE * sin(_correctValue.diff_angle); // 速度关于路径垂直方向的分量
            // float delta_vy = fuzzyPidmodle->fuzzyPidOutputComput(_correctValue.diffDistace_to_line);
            float delta_vy = fuzzyPidmodle->PidOutputComput(_correctValue.diffDistace_to_line);
            ROS_INFO("vy = %f,delta_vy = %f", vel_y, delta_vy);
            vel_y -= delta_vy;
            ROS_INFO("new vy0 = %f", vel_y);
            if(fabs(vel_y) > MAX_VEL_Y)     vel_y = vel_y > 0 ? MAX_VEL_Y : -MAX_VEL_Y;    //vel_y若大于最大法向速度分量，则取值为MAX_VEL_Y

            // ROS_INFO("new vy1 = %f", vel_y);
            // ROS_INFO("asin th = %f", asin(vel_y / MAX_VEL_LINE)); // 纠正后的横向速度分量
            float new_theta = asin(vel_y / MAX_VEL_LINE);
            // if(fabs(new_theta) > 1)   new_theta = new_theta / fabs(new_theta);
            float expect_angle = new_theta + _correctValue.angle_path;
            float delta_th = range_angle_PI(expect_angle - pose.yaw); // 当前机器人方向与期望机器人方向的夹角，需要机器人快速到达该方向
            // ROS_INFO("current angle = %f,increm_th = %f,expect angle = %f,diff_current_and expect = %f", pose.yaw, new_theta, expect_angle, delta_th);
            cmd_vel.angular.z = delta_th / 0.5; // 期望1.0s到达该期望方向，将角度转化为角速度
            if (fabs(cmd_vel.angular.z) > MAX_VEL_ANGLE)    cmd_vel.angular.z = cmd_vel.angular.z > 0 ? MAX_VEL_ANGLE : -MAX_VEL_ANGLE;

        }
    }
    getSmoothVelocity(cmd_vel);
    cmd_vel_last = cmd_vel;
    return false;
}

CorrectFoundation PidControl::getCorrectFoundation(nav_msgs::Path path, ROBOT_POSE pose)
{
    CorrectFoundation _correctValue;
    _correctValue.angle_robot2path = getAngleRobot2path(path,pose);
    _correctValue.angle_path = getPathAngle(path);
    _correctValue.diff_angle = range_angle_PI(pose.yaw - _correctValue.angle_path);                                      // 机器人当前角度与路径的夹角误差
    _correctValue.isleft = range_angle_PI(_correctValue.angle_robot2path - _correctValue.angle_path) < 0 ? true : false; // 机器人与局部路径的平均夹角 - 路径的角度 >0:右  <0:左
    /*if (_correctValue.isleft)
    {
        ROS_INFO("left!!");
    }
    else
    {
        ROS_INFO("right!!");
    }*/
    float diff_distance = getDistanceError(path, pose);
    _correctValue.diffDistace_to_line = _correctValue.isleft ? diff_distance : -diff_distance;
    return _correctValue;
}

float PidControl::getDistanceError(nav_msgs::Path path, ROBOT_POSE pose)
{
    float diff_distance = 0;
    Point p0, p1, p2; // p0:机器人坐标     p1：局部路径起点     p2：局部路径中间点
    p0.x = pose.x;
    p0.y = pose.y;

    p1.x = path.poses[0].pose.position.x;
    p1.y = path.poses[0].pose.position.y;

    for (int i = 1; i < path.poses.size(); i++)
    {
        p2.x = path.poses[i].pose.position.x;
        p2.y = path.poses[i].pose.position.y;
        diff_distance += diffDistace_point_to_line(p0, p1, p2);
    }
    diff_distance /= (path.poses.size() - 1);
    return diff_distance;
}

float PidControl::diffDistace_point_to_line(const Point p0, const Point p1, const Point p2)
{
    // Ax +By + C =0   直线方程的一般式，A、B、C分别为系数
    float A = p2.y - p1.y;
    float B = p1.x - p2.x;
    float C = p2.x * p1.y - p1.x * p2.y;
    float dist = fabs(A * p0.x + B * p0.y + C) / sqrt(A * A + B * B);
    return dist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid control");
    PidControl pc;
    ros::spin();
}