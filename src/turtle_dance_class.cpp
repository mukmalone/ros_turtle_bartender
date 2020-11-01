#include <ros/ros.h>
#include <iostream>
#include <string>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Pose.h>

using namespace std;
float x,y,theta;

class Robot_Class {
	public:	
		Robot_Class() {
			cout<<"Hello World"<<endl;

			//pose for each Pary_Turtle position in Go to Goal algorithm
			//ros::Subscriber sub_pose = n.subscribe<turtlesim::Pose>(robot_name + "/pose", 5, &Robot_Class::poseCallback, this);		
		}
		string robot_name;

		void move_robot();

		void stop_robot();

		ros::NodeHandle n;

		ros::Subscriber sub_pose;

		void spawn_robot(ros::NodeHandle n);

		void poseCallback(const turtlesim::Pose::ConstPtr& msg);

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

	sub_pose = n.subscribe<turtlesim::Pose>("/" + robot_name + "/pose", 5, &Robot_Class::poseCallback, this);		
	cout<<"Robot spawned "<<robot_name<<endl;
}

void Robot_Class::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
	cout<<"hello this is pose callback for: "<<robot_name<<endl;
	//cout<<"x: "<<x<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_dance", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //wait for spawn service so we know turtlesim is up
    ros::service::waitForService("/spawn");
	
	//spawn all the party robots
	int num_robots = 4;
	Robot_Class robot[num_robots];

	for(int i=0; i<num_robots; i++){
		robot[i].robot_name = "Party_Turtle_" + to_string(i);
		robot[i].spawn_robot(n);
		//robot[i].move_robot();
		//robot[i].stop_robot();
	}		
	 
	ros::Rate loop_rate(20);

	while (ros::ok())
    {   
		ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;
}