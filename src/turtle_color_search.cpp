#include "ros/ros.h"
#include "turtlesim/Color.h"
#include "std_msgs/String.h"

float red,green,blue;
bool enable_search;

void colorCallback(const turtlesim::Color::ConstPtr& msg)
{
  ROS_INFO("I heard: Red [%u], Green [%u], Blue [%u]", msg->r, msg->g, msg->b);
  red=msg->r;
  green=msg->g;
  blue=msg->b;
}

void searchCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data=="START_SEARCH"){
    enable_search=true;
  }
}

int main(int argc, char **argv)
{
  enable_search=false;
  ros::init(argc, argv, "turtle_color_search");

  ros::NodeHandle n;

  ros::Rate loop_rate(40);

  ros::Subscriber sub_color = n.subscribe("turtle1/color_sensor", 1000, colorCallback);
  ros::Subscriber sub_search = n.subscribe("/start_search", 1000, searchCallback);

  ros::Publisher control_pub = n.advertise<std_msgs::String>("status", 10);
  std_msgs::String status;
  status.data="NOTHING";
  while (ros::ok())
  {    
    //when we see white in the color sensor we want to publish we found something
    if(red>200 && green>200 && blue>200 && enable_search){
      status.data="FOUND";
    } 
    control_pub.publish(status);      
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}