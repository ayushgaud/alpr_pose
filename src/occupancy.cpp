#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

ros::Publisher marker_pub;
visualization_msgs::Marker occupancy_marker;
tf::StampedTransform init_transform;
const double resolution = 0.2f;
const int num_markers = 250;
const double log_free = 0.8;
const double log_occ = 1.5;
const double max_lim = 5;
const double min_lim = -5;
bool init = false;
struct marker{
	double odds;
	visualization_msgs::Marker occupancy_marker;
};

std::vector<marker> occ_markers;


void carPoseCb(const geometry_msgs::Pose& msg)
{
	tf::StampedTransform car_transform;
	tf::poseMsgToTF(msg, car_transform);
	if (init){
		tf::Pose curr_pose = car_transform.inverseTimes(init_transform);
		std::cout << "Pose: x:" << curr_pose.getOrigin().x() << " y: " << curr_pose.getOrigin().y() << " z: " << curr_pose.getOrigin().z() << std::endl;
		int pos_id = curr_pose.getOrigin().y()/resolution;
		if (pos_id < num_markers && pos_id >=0)
		{	
			std::vector<marker>::iterator nth = occ_markers.begin() + pos_id;
			nth->odds += log_occ;
			nth->odds = std::min(nth->odds,max_lim);
			std::cout << "Id: " << nth->occupancy_marker.id << "Inc odds: " << nth->odds << std::endl;
			nth->occupancy_marker.color.r = (1 - (nth->odds +  5)/10.0f);
			nth->occupancy_marker.color.g = (1 - (nth->odds +  5)/10.0f);
			nth->occupancy_marker.color.b = (1 - (nth->odds +  5)/10.0f);
			marker_pub.publish(nth->occupancy_marker);
		}

	}
	else
	{
			//Marker initialization 
		ROS_INFO("Got initial transform!");
		init_transform = car_transform;
		for(int i = 0; i < num_markers; i++)
		{
			occupancy_marker.id = i;
			tf::Vector3 t_off(occupancy_marker.scale.x/2 -2, -i*resolution ,0);
			tf::Pose marker_offset;
			marker_offset.setOrigin(t_off);
			tf::Pose marker_pose = init_transform * marker_offset;
			marker_pose.setRotation(tf::createQuaternionFromRPY(0, 0, tf::getYaw(init_transform.getRotation())));
			occupancy_marker.pose.position.x = marker_pose.getOrigin().x();
			occupancy_marker.pose.position.y = marker_pose.getOrigin().y();
			occupancy_marker.pose.position.z = 0;
			occupancy_marker.pose.orientation.x = marker_pose.getRotation().x();
			occupancy_marker.pose.orientation.y = marker_pose.getRotation().y();
			occupancy_marker.pose.orientation.z = marker_pose.getRotation().z();
			occupancy_marker.pose.orientation.w = marker_pose.getRotation().w();
			marker temp;
			temp.occupancy_marker = occupancy_marker;
			temp.odds = 0.0f;
			occ_markers.push_back(temp);
			marker_pub.publish(occupancy_marker);

		}
		init = true;

	}
}


void PoseCb(const nav_msgs::Odometry& msg)
{
	//ROS_INFO("PoseCb");
	tf::StampedTransform transform;
	tf::poseMsgToTF(msg.pose.pose, transform);
	if(init)
	{
		//tf::TransformListener listener;
		// tf::StampedTransform transform;
		//try{
			// listener.waitForTransform("/odom", "/camera_base_link", ros::Time(0), ros::Duration(1.0) );
			// listener.lookupTransform("/odom", "/camera_base_link", ros::Time(0), transform);
		tf::Pose curr_pose = transform.inverseTimes(init_transform);
		std::cout << "Empty Pose: x:" << curr_pose.getOrigin().x() << " y: " << curr_pose.getOrigin().y() << " z: " << curr_pose.getOrigin().z() << std::endl;
		int pos_id = curr_pose.getOrigin().y()/resolution;
		if (pos_id >= 3)
			pos_id -= 3;
		std::vector<marker>::iterator nth = occ_markers.begin() + pos_id;
		for (int iter = 0; iter < 6; iter++)
		{
			pos_id += iter;
			if (pos_id < num_markers && pos_id >= 0)
			{	
				if (nth->odds < 1)
					nth->odds -= log_free;
				nth->odds = std::max(nth->odds,min_lim);
				std::cout << "Id: " << nth->occupancy_marker.id << " odds: " << nth->odds << std::endl;
				nth->occupancy_marker.color.r = (1 - (nth->odds +  5)/10.0f);
				nth->occupancy_marker.color.g = (1 - (nth->odds +  5)/10.0f);
				nth->occupancy_marker.color.b = (1 - (nth->odds +  5)/10.0f);
				marker_pub.publish(nth->occupancy_marker);
			}
			nth++;
		}

		// }
		// catch (tf::TransformException &ex) {
		// 	ROS_ERROR("%s",ex.what());
		// 	//ros::Duration(1.0).sleep();

		// }
	}
}

// void carCallback(const std_msgs::String::ConstPtr& msg)
// {

// 	// tf::StampedTransform transform;
// 	// tf::TransformListener listener;
// 	// try 
// 	// {
// 		// listener.waitForTransform("odom", "car_pose", ros::Time(0), ros::Duration(3.0) );
// 		// listener.lookupTransform("odom", "car_pose", ros::Time(0), transform);


// 	// }
// 	// catch (tf::TransformException ex) 
// 	// {
// 	// 	ROS_ERROR("%s",ex.what());
// 	// }
// }

// void transformCallback(const ros::TimerEvent&)
// {

// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy");
	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("license_plate", 10, carCallback);
	ros::Subscriber pos_sub = nh.subscribe("/bebop/odom", 10, PoseCb);
	ros::Subscriber car_sub = nh.subscribe("alpr_pose", 10, carPoseCb);
	marker_pub = nh.advertise<visualization_msgs::Marker>("occupancy_map", 10);
	//Init occupancy marker
	occupancy_marker.header.frame_id = "/odom";
	occupancy_marker.header.stamp = ros::Time::now();
	occupancy_marker.ns = "occupancy";
	occupancy_marker.action = visualization_msgs::Marker::ADD;
	occupancy_marker.id = 0;
	occupancy_marker.type = visualization_msgs::Marker::CUBE;
	occupancy_marker.scale.x = 10.0;
	occupancy_marker.scale.y = resolution;
	occupancy_marker.scale.z = 0.1;
	occupancy_marker.lifetime = ros::Duration(0);
	occupancy_marker.color.b = 0.5f;
	occupancy_marker.color.g = 0.5f;
	occupancy_marker.color.r = 0.5f;
	occupancy_marker.color.a = 1;


	//Transform callback for updation of markers
	//ros::Timer timer = nh.createTimer(ros::Duration(0.05), transformCallback);

	ros::spin();
	return 0;
}