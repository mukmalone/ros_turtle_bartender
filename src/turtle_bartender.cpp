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
#include <ros_turtle_bartender/DrinkOrder.h>
#include <std_msgs/Int64.h>

using namespace std;

const float pi = 3.14159265358979323846;
int num_cust;

void numCustCallback(const std_msgs::Int64::ConstPtr& msg)
{
    num_cust = msg->data;
}

class Robot_Class {
	public:	
		string robot_name;
		ros::NodeHandle n;
		ros::Subscriber subscriber_pose;
		turtlesim::Pose pose;
    	ros::Publisher cmd_vel;    
		geometry_msgs::Twist control_command;		
		string customer;
		string drink;

		void spawn_robot();

		void poseCallback(const turtlesim::Pose::ConstPtr& msg);

		void get_order(int num_customers);

};

void Robot_Class::spawn_robot()
{
	//This class will spawn the turtle and turn-off the pen
	turtlesim::Spawn turtle;    
    turtle.request.x = 9.0 * rand() / (float)RAND_MAX;
	turtle.request.y = 9.0 * rand() / (float)RAND_MAX;
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

void Robot_Class::get_order(int num_customers)
{
	ros::ServiceClient orderClient = n.serviceClient<ros_turtle_bartender::DrinkOrder>("/next_order");
    ros_turtle_bartender::DrinkOrder nextOrder;  
	nextOrder.request.num_customers=num_customers;    
    orderClient.call(nextOrder);
	customer=nextOrder.response.customer;
	drink=nextOrder.response.drink;
    
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

    //kill turtle1 the default turtle
    turtlesim::Kill turtle_to_kill;    
    turtle_to_kill.request.name = "turtle1";    
    ros::ServiceClient kill = n.serviceClient<turtlesim::Kill>("/kill");
    kill.call(turtle_to_kill);
    
    //setup TF listening
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Publisher turtle_vel = 
		n.advertise<geometry_msgs::Twist>(robot.robot_name+"/cmd_vel",10);
	
	ros::Subscriber num_customers = n.subscribe("num_party_turtles", 100, numCustCallback);
			
	ros::Rate loop_rate(20);

	bool drink_picked = false;
	string robot_destination;
	bool subscribed_TF = false;
	bool first_run = true;

	while (ros::ok())
    {   
		if(num_cust>0 && !first_run) {

			if(drink_picked) {
				//we need to bring to the customer
				robot_destination = robot.customer;
			} else {
				//we need to get the drink from proper drink counter
				robot_destination = robot.drink;
			}
			
			geometry_msgs::TransformStamped transformStamped;
			try{
				transformStamped = tfBuffer.lookupTransform(robot.robot_name,robot_destination,
									ros::Time(0));
				subscribed_TF=true;
			}
			catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				subscribed_TF=false;
			}

			geometry_msgs::Twist vel_msg;

			float dist_customer = sqrt(pow(transformStamped.transform.translation.y, 2) +
									pow(transformStamped.transform.translation.x, 2));
			
			if(dist_customer > 0.1) {
				float adaptive_control=1.0;
				//ensure angle is between -pi and pi
				float angle = atan2(transformStamped.transform.translation.y,
												transformStamped.transform.translation.x);
				if (angle<-pi){
					angle+=2*pi;
				}
				else if (angle>pi){
					angle-=2*pi;
				}
				vel_msg.angular.z = 5.0 * angle;
				//control to enable sharp turns and staighter paths
				if(abs(vel_msg.angular.z)<.5){
					adaptive_control=.5;
				} else {
					adaptive_control=abs(vel_msg.angular.z);
				}
				
				vel_msg.linear.x = 1/adaptive_control * sqrt(pow(transformStamped.transform.translation.x,2) +
											pow(transformStamped.transform.translation.y,2));
				turtle_vel.publish(vel_msg);

			} else {

				vel_msg.angular.z = 0;
				vel_msg.linear.x = 0;
				turtle_vel.publish(vel_msg);
				if(!drink_picked && subscribed_TF) {
					//signalling we have the drink
					drink_picked = true;
				} else {
					//signalling we delivered the drink
					robot.get_order(num_cust);
					drink_picked = false;
				}
				
			} 
		} else if(num_cust>0 && first_run) {
			robot.get_order(num_cust);
			first_run=false;
		}
		ros::spinOnce();
        loop_rate.sleep();
		
	}

	return 0;
}