#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>

ros::Publisher pub;

bool rotating = false;
bool moving_back = false;
bool moving_up = false;
bool moving_down = false;
bool moving_left = false;
ros::Time rotation_start_time;

// Boundary values for turtlesim world
const double X_MIN = 0.0;
const double X_MAX = 11.0;
const double Y_MIN = 0.0;
const double Y_MAX = 11.0;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
    geometry_msgs::Twist my_vel;

    // Phase 1: Move right until reaching x > 10.5
    if (!moving_back && !rotating && !moving_up && !moving_down && !moving_left && msg->x < 10.5)
    {
        my_vel.linear.x = 1.0;  // Move forward (right)
        my_vel.angular.z = 0.0;
    }
    // Phase 2: Move back to x = 5.5
    else if (!rotating && !moving_up && !moving_down && !moving_left && msg->x > 5.5)
    {
        my_vel.linear.x = -1.0;  // Move back (left)
        my_vel.angular.z = 0.0;
        moving_back = true;  // Set flag to indicate the turtle is moving back
    }
    else if (moving_back && msg->x <= 5.5)
    {
        moving_back = false;  // Stop moving back
        rotating = true;      // Start rotating
        rotation_start_time = ros::Time::now();  // Record start time for rotation
    }
    // Phase 3: Rotate 90 degrees
    else if (rotating)
    {
        my_vel.linear.x = 0.0;  // Stop moving forward
        my_vel.angular.z = 1.57;  // Rotate 90 degrees (counterclockwise)

        // Wait for 1 second to complete the rotation
        if ((ros::Time::now() - rotation_start_time).toSec() > 1.0)  // 1 second rotation time
        {
            rotating = false;  // Stop rotating
        }
    }
    // Phase 4: Move up until reaching y = 0.5 (since moving up means decreasing y)
    else if (!moving_down && !moving_left && msg->y > 1.0)  // Prevent going beyond the top (y = 0)
    {
        // Rotate the turtle to face upwards (90 degrees counterclockwise)
        my_vel.linear.x = 1.0;  // Move forward (along x, but effectively moves up)
        my_vel.angular.z = 1.57; // Rotate to face up (counterclockwise)
    }
    // Phase 5: Move back down towards the center (y = 5.5)
    else if (!moving_left && msg->y < 5.5)  // Prevent going below y = 11
    {
        // Rotate the turtle to face downwards (90 degrees clockwise)
        my_vel.linear.x = -1.0;  // Move backward (along x, but effectively moves down)
        my_vel.angular.z = 0.0; // Rotate to face down (clockwise)
        moving_down = true;
    }
    else if (moving_down && msg->y >= Y_MAX - 0.5)
    {
        moving_down = false;  // Stop moving down
        rotating = true;      // Start rotating to face left
        rotation_start_time = ros::Time::now();  // Record start time for rotation
    }
    // Phase 6: Rotate 90 degrees to face left
    else if (rotating)
    {
        my_vel.linear.x = 0.0;  // Stop moving forward
        my_vel.angular.z = -1.57;  // Rotate -90 degrees (clockwise, to face left)

        // Wait for 1 second to complete the rotation
        if ((ros::Time::now() - rotation_start_time).toSec() > 1.0)  // 1 second rotation time
        {
            rotating = false;  // Stop rotating
        }
    }
    // Phase 7: Move left until reaching x < 2.0
    else if (!moving_left && msg->x > X_MIN + 0.5)  // Prevent going beyond the left boundary (x = 0)
    {
        my_vel.linear.x = -1.0;  // Move left (along x-axis)
        my_vel.angular.z = 0.0;
        moving_left = true;
    }
    else if (moving_left && msg->x <= X_MIN + 0.5)
    {
        moving_left = false;  // Stop moving left
    }

    // Publish the velocity command to the turtle
    pub.publish(my_vel);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "turtlebot_exercise2");
    ros::NodeHandle nh;

    // Publisher to send velocity commands to "turtle1"
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // Subscriber to "turtle1" pose to get position updates
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);

    // Spin to keep the program running and processing callbacks
    ros::spin();

    return 0;
}
