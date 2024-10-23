
#include "ros/ros.h"
#include "turtlesim/Pose.h"

void turtleCallback(const turtlesim::Pose::ConstPtr& msg);

int main(int argc, char **argv){

    ros::init(argc, argv, "turtlebot_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);
    ros::spin();

    return 0;

}

void turtleCallback(const turtlesim::Pose::ConstPtr& msg){

    ROS_INFO("Turtle subscriber@[%f,%f,%f]", msg->x, msg->y, msg->theta);
}