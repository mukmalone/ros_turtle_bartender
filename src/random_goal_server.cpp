//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This node is a service server giving the Party Turtles
// some random dance moves

#include <ros/ros.h>
#include <ros_turtle_bartender/NextGoal.h>

bool calc_next_goal(ros_turtle_bartender::NextGoal::Request &req,
                    ros_turtle_bartender::NextGoal::Response &res)
{
    res.x = 1 + 9.0 * rand() / (float)RAND_MAX;
    res.y = 1 + 9.0 * rand() / (float)RAND_MAX;
    res.complete=false;
    
    ROS_INFO("new_x: %f new_y:%f", res.x, res.y);
    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "next_goal_server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("/next_goal", 
        calc_next_goal);
    
    ros::spin();
}