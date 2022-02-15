#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>
float x[10],y[10],z[10];
int counter =0;
int min =1.5, flag =0, pos=0;
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 5;
	nextWayPoint.y = -4;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = -3.25;
	nextWayPoint.y = 1.5;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = -9;
	nextWayPoint.y = -2.5;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = -6;
	nextWayPoint.y = -6;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;
  	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;
	for(int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		float d0 = 1.5; 
		float k = .5;
		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .25)
		{
			float x = cos(current_2D_scan.angle_increment*i);
			float y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
			if(min == current_2D_scan.ranges[i])
			{
				
				pos =i;
			}
			avoid = true;
			
		}
	}
	ROS_INFO("minimum value %d",min);
	int vector_y =(-2)*cos(pos*current_2D_scan.angle_increment)*min;
	ROS_INFO("vector_y %d",vector_y);
	int vector_x =2*sin(pos*current_2D_scan.angle_increment)*min;
	ROS_INFO("vector_x %d",vector_x);

	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad );
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	if(avoid)
	{
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 1.5)
		{
			avoidance_vector_x = 1.5* (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 1.5* (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		ROS_INFO("avoiding");

		set_destination((avoidance_vector_x*0.15) + current_pos.x + vector_x, (avoidance_vector_y*0.15) + current_pos.y + vector_y, 2, 0);
		
	}
	else 
	{
		if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
					{
						set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;	
					}
					else{
						//land after all waypoints are reached
						land();
					}	
			}
	}
	

}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);
	//set_speed(2);
	set_destination(0,0,2,0);
	

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
	
	}

	return 0;
}



