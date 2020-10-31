#include <ros/ros.h>
#include <iostream>
#include <string>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <geometry_msgs/Pose.h>

using namespace std;

class Robot_Class
{
public:
	int id;
	int no_wheels;
	string robot_name;
	geometry_msgs::Pose robot_pose;

	//pose for turtle position in Go to Goal algorithm
	ros::NodeHandle nh;
    ros::Subscriber sub_pose = nh.subscribe(robot_name + "/pose", 1000, poseCallback);

	void move_robot();

	void stop_robot();

	void spawn_robot(ros::NodeHandle n);

	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

};

void Robot_Class::move_robot()
{

	cout<<"Moving Robot"<<endl;

}

void Robot_Class::stop_robot()
{

	cout<<"Stopping Robot"<<endl;
}

void Robot_Class::spawn_robot(ros::NodeHandle n)
{
	//This class will spawn the turtle and turn-off the pen
	turtlesim::Spawn turtle;    
    turtle.request.x = 11.0 * rand() / (float)RAND_MAX;
	turtle.request.y = 11.0 * rand() / (float)RAND_MAX;
	turtle.request.theta = 3.14 * rand() / (float)RAND_MAX;
	turtle.request.name = robot_name;
    ros::ServiceClient spawn_turtle = n.serviceClient<turtlesim::Spawn>("/spawn");
    spawn_turtle.call(turtle);

	turtlesim::SetPen pen_state;    
    pen_state.request.off = 1;    
    ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/" + robot_name + "/set_pen");
    pen.call(pen_state);

	cout<<"Robot spawned "<<robot_name<<endl;
}

void Robot_Class::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	//float x = msg->x;
	cout<<"x: "<<endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_dance", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //wait for pen service and turn off pen
    ros::service::waitForService("/spawn");
	
	Robot_Class robot_2;

	robot_2.id = 3;
	robot_2.robot_name = "Humanoid_robot";
	robot_2.spawn_robot(n);
	

	cout<<"ID="<<robot_2.id<<"\t"<<"Robot Name"<<robot_2.robot_name<<endl;

	robot_2.move_robot();
	robot_2.stop_robot();

	return 0;


}