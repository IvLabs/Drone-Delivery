/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <thread> 
using namespace std;

mavros_msgs::State current_state;//Object for storing current state
sensor_msgs::NavSatFix pt;//Object for storing GPS position

//Function to get current state 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Function to get GPS position
void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& loc){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	pt= *loc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
   //Initializing subscribers and publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber location_sub=nh.subscribe("/mavros/global_position/global",1,&locationCallback);    
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    	ros::ServiceClient client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //thread th1(foo); 
    //th1.detach();
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //Get latitude and longitude of the destination from this file
	ifstream f("/home/anish/latlong.txt");
	string a;
	//Clearing all previous wapypoints
system("rosrun mavros mavwp clear");
    //Creating Objects to store waypoints
	mavros_msgs::WaypointPush srv2;
	mavros_msgs::Waypoint wp;
	//Starting position (if you want to directly enter)
	pt.latitude=21.124245;
	pt.longitude=79.051432;
for(int i = 100; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ",pt.latitude,pt.longitude);
        ros::spinOnce();
        rate.sleep();
    }

//Giving parameters to the waypoint object for the starting waypont
wp.frame =mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
wp.is_current = true;
wp.autocontinue = true; 
//Getting Current latitude and Longitude from GPS
wp.x_lat  =pt.latitude;   
wp.y_long =pt.longitude;   
wp.z_alt  =8.0;

//Pushing the starting waypoint to the drone
srv2.request.waypoints.push_back(wp);

if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}

//Storing the destination altitude in a variable
double x,y;
f>>a;
cout<<a;
x = stold(a);
f>>a;
cout<<a;
y=stold(a);
cout.precision(14);
//x=47.3978118;
//y=8.5460740;
cout<<"\nX:"<<x<<" Y:"<<y;
/*
mavros_msgs::SetMode offb_set_mode;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::CommandTOL srv_land;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

*/
//Giving parameters to the waypoint object for the second waypont(Destination at an altitude of 8m)
wp.frame =mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
wp.is_current = false;
wp.autocontinue = true; 
wp.x_lat  =x;   
wp.y_long =y;   
wp.z_alt  =8.0;
srv2.request.waypoints.push_back(wp);
if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}

//Giving parameters to the waypoint object for the second waypont(Loiter at the Destination at an altitude of 8m)
wp.frame =mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
wp.command = mavros_msgs::CommandCode::NAV_LOITER_TIME;
wp.is_current = false;
wp.autocontinue = true; 
wp.x_lat  =x;   
wp.y_long =y;   
wp.z_alt  =4.0;
wp.param1=15;
srv2.request.waypoints.push_back(wp);
if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}

//Giving parameters to the waypoint object for the second waypont(land at the destinatin)
wp.frame =mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
wp.command = mavros_msgs::CommandCode::NAV_LAND;
wp.is_current = false;
wp.autocontinue = true; 
wp.x_lat  =x;   
wp.y_long =y;   
wp.z_alt  =0;
srv2.request.waypoints.push_back(wp);
if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}
////////////////////////////////////////////////////////MISSIon Created///////////////////////////////////////////////
/*
ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
sleep(2);
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "AUTO.MISSION";
if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Auto enabled");
            }
*/
/* 
wp.frame =mavros_msgs::Waypoint::FRAME_GLOBAL;
wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
wp.is_current = false;
wp.autocontinue = true; 
wp.x_lat  =x;   
wp.y_long =y;   
wp.z_alt  =pt.altitude+3.5;


srv2.request.waypoints.push_back(wp);

if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}


wp.frame =mavros_msgs::Waypoint::FRAME_MISSION;
wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
wp.is_current = false;
wp.autocontinue = true; 
wp.x_lat  =pt.latitude;   
wp.y_long =pt.longitude;   
wp.z_alt  =pt.altitude;
srv2.request.waypoints.push_back(wp);
if (client.call(srv2)) {
  ROS_INFO("wp nay send ok %d", srv2.response.success);
}
else {
  ROS_ERROR("Failed");
}*/
    return 0;
}
