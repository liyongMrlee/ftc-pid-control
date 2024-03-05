#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>
#include "tf/transform_listener.h"
#include "pid_control/fuzzyPid_modle.h"


#define MAX_DISTANCE 0.6                        //局部路径最大距离(m)

#define VEL_ANGLE_ROTATION  1                   //原地旋转的角速度
#define MAX_VEL_ANGLE 1                         //最大角速度（rad/s）
#define MIN_VEL_ANGLE 0.2                       //最小角速度（rad/s）
#define MAX_VEL_LINE  0.25                      //最大线速度（m/s）
#define MIN_VEL_LINE  0.05                      //最小线速度（m/s）
#define MAX_VEL_Y     MAX_VEL_LINE * sin(1.2)   //法向速度分量的最大值，用于限制过快的角速度
#define MAX_WHEEL_SPEED 0.3                     //最大轮速。角速度过快时，降低线速度，以尽量满足角速度
#define VEL_LINE_HALF 0.15                      //在转急弯纠偏无效时，减小速度为该速度

#define ACC_ANGLE 0.5                           //角加速度（rad/m*m）
#define ACC_LINE  0.5                           //线加速度（m/s*s）
#define ERROR_ANGLE 0.05                        //角度误差(rad)
#define ERROR_DISTANCE 0.02                     //距离误差(rad)
#define FRONT_CHECK_DIS 0.2                     //前置探测点距离
#define ERROR_GOAL     0.05                     //到达目标点检测
#define SIM_TIME        1.0                     //ftc仿真时间
#define ODOM_L          0.27

typedef struct robot_pose
{
   float x;
   float y;
   float yaw; 
}ROBOT_POSE;

//纠偏所需要的参数信息
typedef struct _CorrectFoundation
{     
    bool isleft;    // 0:顺时针    1：逆时针
    float diffDistace_to_line;
    float diff_angle;
    float angle_robot2path;
    float angle_path;
}CorrectFoundation;

struct Point
{
    float x;
    float y;
};


class PidControl
{
public:
    PidControl();
    ~PidControl();

    void pathCallBack(const nav_msgs::Path::ConstPtr& path);
    bool getRobotPose(ROBOT_POSE& pose);
    void FuzzyPidControl(float control_period);
    void pathSmoother(const nav_msgs::Path::ConstPtr &path,nav_msgs::Path& smooth_path);
    void computeVelocity(nav_msgs::Path path, ROBOT_POSE pose, geometry_msgs::Twist& cmd_vel);
    void CroppingPath(nav_msgs::Path global_path,nav_msgs::Path& local_path,ROBOT_POSE pose,int& point_index);
    void robotNavigation(nav_msgs::Path path,ROBOT_POSE pose,geometry_msgs::Twist& cmd_vel,FuzzyPidModle* fuzzyPidmodle);
    void getMaxDistanceIndex(nav_msgs::Path global_path,int start_index, int &max_distance_index);
    void getMinDistanceIndex(nav_msgs::Path path,ROBOT_POSE pose,int start_index, int &end_index);
    float getRobot2PathAngle(nav_msgs::Path path,ROBOT_POSE pose);
    float getAngleRobot2path(nav_msgs::Path path, ROBOT_POSE pose);
    float getPathAngle(nav_msgs::Path path);
    bool rotateToGlobalPath(nav_msgs::Path path,ROBOT_POSE pose,geometry_msgs::Twist& cmd_vel);
    bool driveToward(nav_msgs::Path path,ROBOT_POSE pose,geometry_msgs::Twist& cmd_vel,FuzzyPidModle* fuzzyPidmodle);
    bool driveTowardByFtc(nav_msgs::Path path,ROBOT_POSE pose,geometry_msgs::Twist& cmd_vel,FuzzyPidModle* fuzzyPidmodle);
    CorrectFoundation getCorrectFoundation(nav_msgs::Path path,ROBOT_POSE pose);
    float getDistanceError(nav_msgs::Path path,ROBOT_POSE pose);
    float diffDistace_point_to_line(const Point p0,const Point p1,const Point p2);
    float getLocalPathDistance(nav_msgs::Path path);
    float getLineVel(float local_path_len);
    void getSmoothVelocity(geometry_msgs::Twist &cmd_vel);
    bool isReachGoal(nav_msgs::Path path,ROBOT_POSE pose);
    bool isReachGoalVirtual(nav_msgs::Path path, ROBOT_POSE pose);


    float range_angle_PI(float angle)
    {
        if (angle > M_PI)
		    angle -= 2 * M_PI;
	    else if (angle < -M_PI)
		    angle += 2 * M_PI;
	    return angle;
    }

    float getMidValue(float value,float max_value,float min_value)      //取最大值和最小值中间的值
    {
        float result;
        result = std::min(value,max_value);             //取当前alue和最大值之间小的那一个
        result = std::max(value,min_value);             //取当前alue和最小值之间大的那一个
        return result;
    }

private:
    ros::NodeHandle node_;
    /*ros topic*/
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher  vel_pub_;
    ros::Publisher  trajectory_pub_;
    ros::Publisher  local_path_pub_;
    ros::Publisher  smooth_path_pub_;
    boost::thread* control_thread_;
    tf::StampedTransform location_transform;
    tf::TransformListener location_listener;

    bool goal_reach,rotate_to_global_path,new_path_flag;
    nav_msgs::Path global_path,trajectory_path;
    float control_period;
    geometry_msgs::Twist cmd_vel_last;

};