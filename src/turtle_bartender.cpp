#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <string>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros_turtle_bartender/NextGoal.h>
#include <turtlesim/Kill.h>

using namespace std;

const float pi = 3.14159265358979323846;

class Robot_Class {
	public:	
		string robot_name;
		ros::NodeHandle n;
		ros::Subscriber subscriber_pose;
		turtlesim::Pose pose;
    	ros::Publisher cmd_vel;    
		geometry_msgs::Twist control_command;		
		float goal_x;
		float goal_y;

		void move_robot();

		void spawn_robot();

		void poseCallback(const turtlesim::Pose::ConstPtr& msg);

		void get_goal();

		bool robot_at_goal();

};

void Robot_Class::move_robot()
{
	float angle = atan2(goal_y-pose.y, goal_x-pose.x) - pose.theta;
    //ensure angle is between -pi and pi
    if (angle<-pi){
        angle+=2*pi;
    }
    else if (angle>pi){
        angle-=2*pi;
    }

	//angular velocity
	control_command.angular.z = 5 * (angle); 
	float adaptive_control=1.0;

	//control to enable sharp turns and staighter paths
	if(abs(control_command.angular.z)<.5){
		adaptive_control=.5;
	} else {
		adaptive_control=abs(control_command.angular.z);
	}

	//linear velocity
	control_command.linear.x = 2/adaptive_control* sqrt( pow(goal_x-pose.x,2) + pow(goal_y-pose.y,2) );

	cmd_vel.publish(control_command);
}

void Robot_Class::spawn_robot()
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

	subscriber_pose = n.subscribe<turtlesim::Pose>("/" + robot_name + "/pose", 5, &Robot_Class::poseCallback, this);
	cmd_vel = n.advertise<geometry_msgs::Twist>(robot_name+"/cmd_vel", 10);

	//initiate the values of the control command to zero where needed
	control_command.linear.y = 0.0;
    control_command.linear.z = 0.0;
    control_command.angular.x = 0.0;
    control_command.angular.y = 0.0;
}

void Robot_Class::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	//populate pose for movement
    pose.x=msg->x;
	pose.y=msg->y;
	pose.theta=msg->theta;
    
    //broadcast TF
    static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = robot_name;
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

void Robot_Class::get_goal()
{
	ros::ServiceClient goalClient = n.serviceClient<ros_turtle_bartender::NextGoal>("/next_goal");
    ros_turtle_bartender::NextGoal nextGoal;  
	nextGoal.request.x=pose.x;
    nextGoal.request.y=pose.y;
    nextGoal.request.theta=pose.theta;
    goalClient.call(nextGoal);
	goal_x=nextGoal.response.x;
	goal_y=nextGoal.response.y;
    
}

bool Robot_Class::robot_at_goal()
{
	//check if we are at the goal within tolerance
    float dx, dy, tolerance=0.1;
    dx = abs(pose.x-goal_x);
    dy = abs(pose.y-goal_y);
	if(dx<=tolerance && dy<=tolerance){
        //if we are at the goal move to next goal
        return true;
    } else {
		return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_dance_party", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //wait for spawn service so we know turtlesim is up
    ros::service::waitForService("/spawn");
	
	//spawn all the party robots
	Robot_Class robot;
	
	robot.robot_name = "turtle_bartender";	
    robot.spawn_robot();
	//robot.get_goal();

    //kill turtle1 the default turtle
    turtlesim::Kill turtle_to_kill;    
    turtle_to_kill.request.name = "turtle1";    
    ros::ServiceClient kill = n.serviceClient<turtlesim::Kill>("/kill");
    kill.call(turtle_to_kill);
    
    //setup TF listening
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
			
	ros::Rate loop_rate(20);
    bool first_run = true;

	while (ros::ok())
    {   

		//cycle through each party turtle.  if it is at the goal, get a new one
		//if it is not, keep moving towards the goal
		ros::spinOnce();
        loop_rate.sleep();

        if(first_run){
            robot.get_goal();
            first_run = false;
        }
		
        if(!robot.robot_at_goal()){
            robot.move_robot();
        } else {
            robot.get_goal();
        }
		
        string robot_customer = "Party_Turtle_0";
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform(robot.robot_name, robot_customer,
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            //continue;
        }

	}

	return 0;
}