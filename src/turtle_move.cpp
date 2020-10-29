#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>
#include <ros_hideandseek/NextGoal.h>
#include <cmath>

float x, y, theta;
bool complete,enable_search;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
}

void statusCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "FOUND"){
        complete=true;
    }
}

void searchCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data=="START_SEARCH"){
    enable_search=true;
  }
}

int main(int argc, char **argv)
{
    //we need to wait until board is setup before starting the search pattern
    enable_search=false;
    //flag to signal we are end of search pattern or we found circle
    complete = false;
    
    ros::init(argc, argv, "turtle_move");
 
    ros::NodeHandle n;
    //pose for turtle position in Go to Goal algorithm
    ros::Subscriber sub_pose = n.subscribe("turtle1/pose", 1000, poseCallback);
    //monitoring if color sensor found circle
    ros::Subscriber sub_status = n.subscribe("status", 1000, statusCallback);
    //sending velocity commands to drive turtlesim
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    //topic which will monitor everything is setup
    ros::Subscriber sub_search = n.subscribe("/start_search", 1000, searchCallback); 
    //service to get the next goal when we reach current goal
    ros::ServiceClient goalClient = n.serviceClient<ros_hideandseek::NextGoal>("/next_goal");
    ros_hideandseek::NextGoal nextGoal;

    float search_step_size;
    n.getParam("/search_step_size", search_step_size);

    ros::Rate loop_rate(20);

    //when board gets setup we exit loop
    while(!enable_search){
        ros::spinOnce();
        loop_rate.sleep();
    }

    const float pi = 3.14159265358979323846;
    geometry_msgs::Twist control_command;
    control_command.linear.y = 0.0;
    control_command.linear.z = 0.0;
    control_command.angular.x = 0.0;
    control_command.angular.y = 0.0;

    int goal_num = 0;
    bool goal_complete = false;
    while (ros::ok() && !complete)
    {                 
        if(goal_num==0){
            //we are just starting and want to get our first goal
            goal_complete=true;            
        }  

        if(goal_complete){
            if(goal_num==0){
                nextGoal.request.x=search_step_size;
                nextGoal.request.y=search_step_size;
                nextGoal.request.theta=theta;
                goal_num++;
            } else {
                nextGoal.request.x=nextGoal.response.x;
                nextGoal.request.y=nextGoal.response.y;
                nextGoal.request.theta=theta;
            }
            goalClient.call(nextGoal);
            goal_complete=false;
        }

        float goal_x=nextGoal.response.x;
        float goal_y=nextGoal.response.y;

        float angle = atan2(goal_y-y, goal_x-x) - theta;
        //ensure angle is between -pi and pi
        if (angle<-pi){
            angle+=2*pi;
        }
        else if (angle>pi){
            angle-=2*pi;
        }
        //angular velocity
        control_command.angular.z = 5 * (angle); 
        float adaptive_control=1;
        //control to enable sharp turns and staighter paths
        if(abs(control_command.angular.z)<.5){
            adaptive_control=.5;
        } else {
            adaptive_control=abs(control_command.angular.z);
        }
        //linear velocity
        control_command.linear.x = .5/adaptive_control* sqrt( pow(goal_x-x,2) + pow(goal_y-y,2) );

        if(complete){
            //slam on brakes if we found it
            control_command.linear.x=0;
            control_command.linear.z=0;
        }

        control_pub.publish(control_command);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Turtle cmd: x [%f], ang.z [%f]",control_command.linear.x, control_command.angular.z);

        //check if we are at the goal within tolerance
        float dx, dy, tolerance=0.1;
        dx = abs(x-goal_x);
        dy = abs(y-goal_y);

        if(dx<=tolerance && dy<=tolerance){
            //if we are at the goal move to next goal
            goal_complete=true;
        }

        if(nextGoal.response.complete){
            //if we have cycled through all goals stop
            //eventually this is where we will look for the color to change
            complete=true;
        }
    }

    if(complete){
        control_command.linear.x=0;
        control_command.linear.z=0;
        control_pub.publish(control_command);
        ros::spinOnce();
    }

    return 0;
}