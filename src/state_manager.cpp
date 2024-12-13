#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath>
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>

class StateManager {
public:
    StateManager(){  // プライベートノードハンドルを初期化
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
        waypoint_num_pub_ = nh_.advertise<std_msgs::Int16>("waypoint_num", 1);
        goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
        robot_speed_pub_ = nh_.advertise<std_msgs::Float32>("speed", 1000, this);

        main_waypoint_sub_ = nh_.subscribe("main_waypoint", 1000, &StateManager::mainWaypointCallback, this);
        sub_waypoint_sub_ = nh_.subscribe("sub_waypoint", 1000, &StateManager::subWaypointCallback, this);
        collision_state_sub_ = nh_.subscribe("collision_state", 1000, &StateManager::collisionStateCallback, this);
        goal_judge_sub_ = nh_.subscribe("goal_judge", 1000, &StateManager::goalJudgeCallback, this);
        sub_waypoint_state_sub_ = nh_.subscribe("sub_waypoint_state", 1000, &StateManager::subWaypointStateCallback, this);
        timer_callback_ = nh_.createTimer(ros::Duration(1.0), &StateManager::timerCallback, this);

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        waypoint_num_msg_.data = 0;
        waypoint_num_flag = 0;
        sub_waypoint_state_ = 0;
        is_timer_start = false;

        // パラメータの取得
        pnh_.getParam("current_location", current_location_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;  // プライベートノードハンドル

    ros::Subscriber waypoint_sub_, waypoint_num_sub_, collision_state_sub_, goal_judge_sub_, main_waypoint_sub_, sub_waypoint_sub_, sub_waypoint_state_sub_;
    ros::Publisher marker_pub_, waypoint_pub_, waypoint_num_pub_, goal_reached_pub_, robot_speed_pub_;
    ros::Timer timer_callback_;
    ros::Time timer_start_;
    ros::Time timer_now_;
    std_msgs::Int16 waypoint_num_msg_;
    geometry_msgs::PoseStamped waypoint_;
    geometry_msgs::Point p_;
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    geometry_msgs::PoseStamped goal_;  // 目標地点goal_judge 
    geometry_msgs::PoseStamped main_waypoint_;
    geometry_msgs::PoseStamped sub_waypoint_;
    std_msgs::Float32 robot_speed_msg;
    std_msgs::Bool goal_reached_msg_;

    ros::Time start_;
    ros::Time now_;


    double roll_, pitch_, yaw_;
    double theta_;
    double angle_rad_, angle_deg_;
    double distance_, distance_judge_;
    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;
    bool goal_reached_;
    int collision_state_;
    int goal_judge_;
    int waypoint_num_flag;
    int sub_waypoint_state_;
    bool is_timer_start;
    std::string current_location_;  // 取得したパラメータを格納

    void timerCallback(const ros::TimerEvent&);
    void robotSpeedPub();
    void waypointManager();
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mainWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subWaypointStateCallback(const std_msgs::Int16::ConstPtr& msg);
    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);
    void collisionStateCallback(const std_msgs::Int16::ConstPtr& msg);
    void goalJudgeCallback(const std_msgs::Int16::ConstPtr& msg);
    
};

// タイマコールバックの実装（内容は省略）
void StateManager::timerCallback(const ros::TimerEvent&) {
    // タイマ処理のロジックを記述
    robotSpeedPub();
    waypointManager();

}

void StateManager::waypointManager(){
    //3秒停止するtimer
    if(collision_state_ == 2 && goal_judge_ == 0){//goal_judge_ == 0はいらないかも

        if(is_timer_start == false){
            start_ = ros::Time::now();
            ROS_INFO("TIMER START");
            is_timer_start = true;
        }

        now_ = ros::Time::now();
        if (now_ - start_ > ros::Duration(3.0))
        {
            ROS_INFO("3sec PASSED");
            goal_judge_ = 0; //いらないかも
            main_waypoint_.pose.position.x = sub_waypoint_.pose.position.x;
            main_waypoint_.pose.position.y = sub_waypoint_.pose.position.y;
            main_waypoint_.pose.position.z = sub_waypoint_.pose.position.z;
            // waypoint_pub_.publish(sub_waypoint_);
            ROS_INFO("PUBLISH SUB WAYPOINT: (%f, %f)", main_waypoint_.pose.position.x, main_waypoint_.pose.position.y);

        }
    }
    else{
        is_timer_start = false;
    }

        std::cout << "goal_judge_" << goal_judge_ << std::endl;
        std::cout << "waypoint_num_flag" << waypoint_num_flag << std::endl;

    if(goal_judge_ == 1 && waypoint_num_flag == 0){
    // if(goal_judge_ == 1){
        waypoint_num_msg_.data += 1;
        std::cout << "waypoint_num_msg_.data" << waypoint_num_msg_.data << std::endl;
        std::cout << "waypoint_num_flag" << waypoint_num_flag << std::endl;
        std::cout << "goal_judge_" << goal_judge_ << std::endl;

        waypoint_num_flag = 1;//ずっとこのif文内に入るのを防ぐ　いらないかも　waypointが更新されるので、goal_judgeがfalseになる

        goal_reached_msg_.data = true;
        std::cout << "bbbbb" << std::endl;
        // goal_reached_pub_.publish(goal_reached_msg_);
        // waypoint_num_pub_.publish(waypoint_num_msg_);
        // waypoint_pub_.publish(main_waypoint_);
    }
    if(goal_judge_ == 0){
        goal_reached_msg_.data = false;
        std::cout << "ccccc" << std::endl;

    }
    if(goal_judge_ == 0){
        waypoint_num_flag = 0;
    }
        goal_reached_pub_.publish(goal_reached_msg_);
        waypoint_num_pub_.publish(waypoint_num_msg_);
        waypoint_pub_.publish(main_waypoint_);
    
    // if(sub_waypoint_state_ == 1){
    //     goal_judge_ = 0; //いらないかも
    //     waypoint_pub_.publish(sub_waypoint_);
    //     ROS_INFO("PUBLISH SUB WAYPOINT");
    // }else{
    //     if(goal_judge_ == 1){
    //         waypoint_pub_.publish(goal_);
    //         waypoint_num_msg_.data += 1;
    //         goal_judge_ = 0;
    //         goal_reached_pub_.publish(goal_reached_msg_);
    //         waypoint_num_pub_.publish(waypoint_num_msg_);
    //     }
    //     else{
    //         // waypoint_pub_.publish(waypoint_);
    //         continue;
    //     }
    // }


}

void StateManager::robotSpeedPub(){
    std::cout << "collision_state_ : " << collision_state_ << std::endl;
    if(collision_state_ == 0){
        robot_speed_msg.data = 1.0;
    }
    else if(collision_state_ == 1){
        robot_speed_msg.data = 0.1;
    }
    else if(collision_state_ == 2){
        robot_speed_msg.data = 0.0;
    }
    else{
        ROS_INFO("UNKNOWN COLLISION STATE");
    }
    robot_speed_pub_.publish(robot_speed_msg);
}

void StateManager::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.pose.position.x = msg->pose.position.x;
    goal_.pose.position.y = msg->pose.position.y;
    goal_.pose.position.z = msg->pose.position.z;
}

void StateManager::mainWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    main_waypoint_.pose.position.x = msg->pose.position.x;
    main_waypoint_.pose.position.y = msg->pose.position.y;
    main_waypoint_.pose.position.z = msg->pose.position.z;
    std::cout << "main_waypoint.x" << main_waypoint_.pose.position.x << std::endl;
}

void StateManager::subWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sub_waypoint_.pose.position.x = msg->pose.position.x;
    sub_waypoint_.pose.position.y = msg->pose.position.y;
    sub_waypoint_.pose.position.z = msg->pose.position.z;
}

void StateManager::subWaypointStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    sub_waypoint_state_ = msg->data;
    // std::cout << "sub_waypoint_state_ : " << sub_waypoint_state_ << std::endl;
}

void StateManager::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
    goal_reached_ = msg->data;
}

void StateManager::collisionStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    collision_state_ = msg->data;
}

void StateManager::goalJudgeCallback(const std_msgs::Int16::ConstPtr& msg) {
    goal_judge_ = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_manager");

    StateManager cm;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
