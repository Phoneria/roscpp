#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h> // Arming 
#include <geometry_msgs/PoseStamped.h> // Local Positioning
#include <mavros_msgs/SetMode.h> // Flight Mode
#include <mavros_msgs/State.h> // State
#include <nav_msgs/Odometry.h> 
#include <thread>
#include <math.h>

mavros_msgs::CommandBool com;
geometry_msgs::PoseStamped pose;
mavros_msgs::SetMode mode;
mavros_msgs::State state;
nav_msgs::Odometry odom;

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
    odom=*msg;
}


void state_cb(const mavros_msgs::State::ConstPtr &msg){
    state=*msg;
}

ros::Publisher pose_pub ;

int is_reached(){
    float x_desired, y_desired, z_desired;
    x_desired = pose.pose.position.x;
    y_desired = pose.pose.position.y;
    z_desired = pose.pose.position.z;

    float x_diff = x_desired - pose.pose.position.x;
    float y_diff = y_desired - pose.pose.position.y;
    float z_diff = z_desired - pose.pose.position.z;


    if (sqrt(pow(x_diff,2) + pow(y_diff,2) +  pow(z_diff,2)) < 1.2){
        return 1;
    }
    else{
        return 0;
    }

}

void setpoint(int x){

    ros::Rate looprate(10);
    while(ros::ok()){
        ROS_INFO("x:%f, y:%f, z:%f,mode:%s, reached:%i", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z, state.mode.c_str(), is_reached());
        pose_pub.publish(pose);
        ros::spinOnce();
        looprate.sleep();
        ros::Duration(1).sleep();

    }
}

void circle(){
    
    ros::Rate looprate(10);
    while(ros::ok()){
        for (float i = 0.0;i<6.28; i = i + 0.01){
            pose.pose.position.x = cos(i)*5;
            pose.pose.position.y = sin(i)*5;
            pose.pose.position.z = 5;
            ros::Duration(0.01).sleep();

        }
    }
}

int main(int argc, char** argv){

    ros::init(argc,argv, "offboard_node");
    ros::NodeHandle nh;

    ros::ServiceClient com_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10 , odom_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);



    ros::Rate looprate(10);

    while (ros::ok() && !state.connected){
        ROS_INFO("Connecting ...");
        ros::spinOnce();
        looprate.sleep();
    }

    ROS_INFO("CONNECTED TO DRONE");
    
    std::thread thr_setpoint(setpoint, 100);
    
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for ( int i = 100; ros::ok() && i > 0 ; --i){
        pose_pub.publish(pose);
        ros::spinOnce();
        looprate.sleep();
    }

    mode.request.custom_mode = "OFFBOARD";


    ROS_INFO("OFFBOARD MODE");

    if (mode_client.call(mode)==true){

        if(mode.response.mode_sent == true){
            ROS_INFO("Offboard is enabled");
        }

        com.request.value = true;


        if(com_client.call(com)==true){
            if(com.response.result==0){
                ROS_INFO("Arming is done");
            
            }
        }
        
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 5;


        while(ros::ok()){
            while (is_reached() != 1){
                ROS_INFO("\nWaiting to reach");
            }
            ROS_INFO("\nREACHED!!!!");
            if (is_reached()==1){
                circle();
            }
        }

        thr_setpoint.join();


    }
    
    else{
        ROS_INFO("Mode doesn't match");
    }



















}











