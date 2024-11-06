
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "turtlebot_publisher");
    ros::NodeHandle nh;

    
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("turtle1/cmd_vel", 10);
    ros::ServiceClient client1 = nh.serviceClient <turtlesim::Spawn> ("/spawn");
    turtlesim::Spawn srv1;

    srv1.request.x = 1.0;
    srv1.request.y = 5.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "new_turtle";

    client1.call(srv1);

    ros::Rate loop_rate(2);

    geometry_msgs::Twist my_vel;    

    while (ros::ok)
    {
       my_vel.linear.x = 2.0;
       my_vel.angular.z = 1.0;
       pub.publish(my_vel);
       ros::spinOnce();
       loop_rate.sleep();
    }

    return 0;
}
