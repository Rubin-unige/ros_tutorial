
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <unistd.h>  

float linear_x, angular_z;

int main( int argc, char **argv){

    // Initialise ros 
    ros::init(argc, argv, "turtle_user_interface");
    ros::NodeHandle nh;

    // Initialise service client
    ros::ServiceClient client_spawn = nh.serviceClient <turtlesim::Spawn> ("/spawn");
    // Spawn turtle 2
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
    client_spawn.call(spawn_srv);

    // Initialise publishers for turtle1 and turtle2
    ros::Publisher pub_turtle1 = nh.advertise <geometry_msgs::Twist> ("turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise <geometry_msgs::Twist> ("turtle2/cmd_vel", 10);

    std::string turtle_name;

    while (ros::ok())
    {
        // User input
        std::cout << "Enter the turtle you want to control(turtle1 or turtle2): ";
        std::cin >> turtle_name;
        // Ask user to enter the velocities
        std::cout << "Enter the linear velocity x: ";
        std::cin >> linear_x;
        std::cout << "Enter the angular velocity z: ";
        std::cin >> angular_z;

        geometry_msgs::Twist turtle_vel;
        turtle_name.linear.x = linear_x;
        turtle_name.angular.z = angular_z;

        // Turtle1
        if (turtle_name == "turtle1")
        {
            pub_turtle1.publish(turtle_vel);
            sleep(1);
            turtle_vel.linear.x = 0.0;
            turtle_vel.angular.z = 0.0;
            pub_turtle1.publish(turtle_vel);
        }
        // Turtle2
        else if (robot_name == "turtle2") {
            pub_turtle2.publish(move_cmd);  
            sleep();    
            move_cmd.linear.x = 0;         
            move_cmd.angular.z = 0;
            pub_turtle2.publish(move_cmd); 
        } 
        // Invalid input
        else {
            ROS_WARN("Invalid turtle name. Please choose 'turtle1' or 'turtle2'.");
        }

        ros::spinOnce(); 

    }
    
    return 0;
    
}
