#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"  
#include "turtlesim/Kill.h"              

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("Turtle position: [%f, %f, %f]", msg->x, msg->y, msg->theta);

    geometry_msgs::Twist my_vel;
    
    // Check if the turtle has reached the right or left boundary
    if (msg->x >= 9.0)  // Right boundary
    {
        my_vel.linear.x = 0.5;   // Move forward
        my_vel.angular.z = 0.5;  // Turn left (positive rotation)
    }
    else if (msg->x <= 2.0)  // Left boundary
    {
        my_vel.linear.x = 0.5;    // Move forward
        my_vel.angular.z = -0.5;  // Turn right (negative rotation)
    }
    else
    {
        // In between boundaries, move straight along the x-axis
        my_vel.linear.x = 0.5;
        my_vel.angular.z = 0.0;
    }
    
    pub.publish(my_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_subscriber");
    ros::NodeHandle nh;
    
    pub = nh.advertise<geometry_msgs::Twist>("rpr_turtle/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("rpr_turtle/pose", 1, turtleCallback);

    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    ros::ServiceClient client2 = nh.serviceClient<turtlesim::Kill>("/kill");

    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";
    if (client2.call(kill_srv))
    {
        ROS_INFO("Successfully killed turtle1.");
    }
    else
    {
        ROS_ERROR("Failed to kill turtle1.");
    }

    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 2.0;
    spawn_srv.request.y = 1.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "rpr_turtle";

    if (client1.call(spawn_srv))
    {
        ROS_INFO("Successfully spawned rpr_turtle at (2.0, 1.0, 0.0).");
    }
    else
    {
        ROS_ERROR("Failed to spawn rpr_turtle.");
    }

    ros::spin();

    return 0;
}
