#include <ros/ros.h>
#include <ros_hideandseek/NextGoal.h>

bool calc_next_goal(ros_hideandseek::NextGoal::Request &req,
                    ros_hideandseek::NextGoal::Response &res)
{
    ros::NodeHandle n;
    float search_step_size;
    n.getParam("/search_step_size", search_step_size);

    float limit_x=11.0;
    float limit_y=11.0;
    const float pi = 3.14159265358979323846;
    float new_y = req.y + search_step_size;

    res.complete=false;
    ROS_INFO("Theta: %f", req.theta);
    if(req.theta > 1.4 && req.theta < 2 ){
        //turtle is pointing up and we need to move right or left
        if(abs(req.x-search_step_size)<.5){
            //we are on the left and need to move right
            res.x=limit_x-search_step_size;
            res.y=req.y;
        } else {
            //we are on the right and need to go left
            res.x=search_step_size;
            res.y=req.y;
        }    
    } 
    else {
        res.x=req.x;
        res.y=new_y;
    }

    //check if answer choosen is outside the limits. If it is we are done searching
    if(res.y>=limit_y){
        res.complete=true;
        res.x=0;
        res.y=0;
    }
    
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