//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This node broadcasts the position of the drink stands

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
using namespace std;
const float pi = 3.14159265358979323846;

int main(int argc, char** argv){
    ros::init(argc, argv, "drink_location_broadcaster");    
    ros::NodeHandle n;
    static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
    ros::Rate loop_rate(20);

    int cnt = 0;
    string drink_name;
    float drink_x, drink_y, theta;

    while (ros::ok())
    { 
        //there will be 4 drink types defined and broadcasted here.  
        //One in each corner
        switch (cnt)
        {
            case (int)0:
                drink_name = "Red Wine";
                drink_x = 1.0;
                drink_y = 1.0;
                theta = -pi*3/4;
                break;
            case (int)1:
                drink_name = "White Wine";
                drink_x = 10.0;
                drink_y = 1.0;
                theta = -pi/4;
                break;
            case (int)2:
                drink_name = "Vodka";
                drink_x = 1.0;
                drink_y = 10.0;
                theta = pi*3/4;
                break;
            case (int)3:
                drink_name = "Jamison";
                drink_x = 10.0;
                drink_y = 10.0;
                theta = pi/4;
                break;
            default:
                break;
        }
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = drink_name;
        transformStamped.transform.translation.x = drink_x;
        transformStamped.transform.translation.y = drink_y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
        ros::spinOnce();
        loop_rate.sleep();
        cnt++;
        if(cnt>3){cnt=0;}
    }

  
    ros::spin();
    return 0;
};