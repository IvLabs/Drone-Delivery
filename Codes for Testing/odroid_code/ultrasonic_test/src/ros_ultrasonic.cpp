#include "ros/ros.h"
#include "wiringPi.h"
#include <iostream>
#include <unistd.h>
#include <ctime>
#include "std_msgs/Float64.h"

#define ECHO 1
#define TRIG 0

float get_ultrasonic_reading(void);

int main(int argc, char **argv)
{
	wiringPiSetup();

	// TRIG pin 
	pinMode(TRIG, OUTPUT);
	// ECHO pin
	pinMode(ECHO, INPUT);
	
	// initialize node
	ros :: init(argc, argv, "ultrasonic_readings");
	
	// create NodeHandle
	ros :: NodeHandle n;
	
	// create publisher
	ros :: Publisher ultra_pub = n.advertise<std_msgs :: Float64>("distance", 1000);
	
	ros :: Rate loop_rate(10);
	
	while (ros :: ok())
	{
		float dist = get_ultrasonic_reading();
		
		std_msgs :: Float64 msg;
		msg.data = dist;
		
		ultra_pub.publish(msg);
		
		loop_rate.sleep();
	}
	
	return 0;
}

float get_ultrasonic_reading(void)
{
	// clear trigger pin
	digitalWrite(TRIG, 0);
	delayMicroseconds(3);
	
	// 10us HIGH trigger signal
	digitalWrite(TRIG, 1);
	delayMicroseconds(12);
	digitalWrite(TRIG, 0);
	
	clock_t begin_time;
	clock_t end_time;
	// wait for echo start
	while (digitalRead(ECHO) == 0)
	{
		begin_time = clock();
	}
	
	// wait for echo end
	while (digitalRead(ECHO) == 1)
	{
		end_time = clock();
	}
	
	// find difference of time in secs.
	float time_diff = float(end_time - begin_time) / CLOCKS_PER_SEC;
	
	// find distance in cm
	float dist = time_diff * 17150;
	
	return(dist);
}
