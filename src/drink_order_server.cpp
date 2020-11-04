#include <ros/ros.h>
#include <ros_turtle_bartender/DrinkOrder.h>
#include <stdio.h>
#include <curl/curl.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

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

    // HTTP response body (not headers) will be sent directly to this stringstream
    stringstream response;

    curlpp::Easy foo;
    foo.setOpt( new curlpp::options::Url( "http://localhost:3000/" ) );
    foo.setOpt( new curlpp::options::UserPwd( "blah:passwd" ) );
    foo.setOpt( new curlpp::options::WriteStream( &response ) );

    // send our request to the web server
    foo.perform();
    
    ros::spin();

}

