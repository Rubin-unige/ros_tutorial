
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg);

int main(int argc, char **argv){

    ros::init(argc, argv, "turtlebot_publisher");
    ros::NodeHandle nh;
    

    pub = nh.advertise<geometry_msgs::Twist> ("turtle1/cmd_vel", 1);
    ros::spin();

    return 0;

}

void turtleCallback(const turtlesim::Pose::ConstPtr& msg){

    geometry_msgs::Twist my_vel;
    my_vel.linear.x = 1.0;
    my_vel.angular.z = 1.0;
    pub.publish(my_vel);
}