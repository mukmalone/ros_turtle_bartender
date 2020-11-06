#include <ros/ros.h>
#include <ros_turtle_bartender/DrinkOrder.h>
#include <stdio.h>
#include <curl/curl.h>
#include <iostream>
#include <string>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

using namespace std;

bool get_next_order(ros_turtle_bartender::DrinkOrder::Request &req,
                    ros_turtle_bartender::DrinkOrder::Response &res)
{
    CURL *curl;
    CURLcode res_curl;
    string readBuffer;
    string delimiter = ",";
    string strArguments = "name=daniel&project=curl";
    const char *data = "data";;
    
    curl = curl_easy_init();
    
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, 12L);
        
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
        curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:3000");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res_curl = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        res.customer = readBuffer.substr(0, readBuffer.find(delimiter));
        res.drink = readBuffer.substr(readBuffer.find(delimiter), readBuffer.length());
    }
    
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

