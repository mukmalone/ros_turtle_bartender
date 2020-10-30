#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>
#include <ros_turtle_bartender/NextGoal.h>
#include <cmath>
#include <turtlesim/SetPen.h>

float x, y, theta;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "turtle_dance", ros::init_options::AnonymousName);
 
    ros::NodeHandle n;

    //wait for pen service and turn off pen
    ros::service::waitForService("/turtle1/set_pen");
    turtlesim::SetPen pen_state;    
    pen_state.request.off = 1;    
    ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    pen.call(pen_state);

    //pose for turtle position in Go to Goal algorithm
    ros::Subscriber sub_pose = n.subscribe("turtle1/pose", 1000, poseCallback);

    //sending velocity commands to drive turtlesim
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    
    //service to get the next goal when we reach current goal
    ros::ServiceClient goalClient = n.serviceClient<ros_turtle_bartender::NextGoal>("/next_goal");
    ros_turtle_bartender::NextGoal nextGoal;    

    ros::Rate loop_rate(20);

    const float pi = 3.14159265358979323846;
    geometry_msgs::Twist control_command;
    control_command.linear.y = 0.0;
    control_command.linear.z = 0.0;
    control_command.angular.x = 0.0;
    control_command.angular.y = 0.0;

    bool goal_complete = false;
    while (ros::ok())
    {                 

        if(goal_complete){
            nextGoal.request.x=nextGoal.response.x;
            nextGoal.request.y=nextGoal.response.y;
            nextGoal.request.theta=theta;
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
    }

    return 0;
}