#include <ros/ros.h>
#include <ros_turtle_bartender/DrinkOrder.h>
using namespace std;

bool get_next_order(ros_turtle_bartender::DrinkOrder::Request &req,
                    ros_turtle_bartender::DrinkOrder::Response &res)
{
    res.customer = "Party_Turtle_1";
    res.drink = "Red Wine";
    
    cout<<"Customer: "<<res.customer<<"Drink: "<<res.drink<<endl;
    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "next_order_server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("/next_order", 
        get_next_order);
    
    ros::spin();
}