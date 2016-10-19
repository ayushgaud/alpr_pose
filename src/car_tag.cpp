
#include <cstdio>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <string>
#include <cstdlib>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

struct car{
	int id;
	double confidence;
	std::string plate;
	ros::Time last_appear;
	int tot_seen;
	visualization_msgs::Marker car_marker;
};
int count = 0;
std::list<car> cars;
std::list<car>::iterator it;
ros::Duration thresh_dur(5.0);
int thresh_seen = 2;
ros::Publisher marker_pub;

int LevenshteinDistance(std::string s, std::string t)
{
    // degenerate cases
    if (s == t) return 0;
    if (s.length() == 0) return t.length();
    if (t.length() == 0) return s.length();

    // create two work lists of integer distances
    int v0[t.length() + 1];
    int v1[t.length() + 1];

    // initialize v0 (the previous row of distances)
    // this row is A[0][i]: edit distance for an empty s
    // the distance is just the number of characters to delete from t
    for (int i = 0; i < t.length() +1 ; i++)
        v0[i] = i;

    for (int i = 0; i < s.length(); i++)
    {
        // calculate v1 (current row distances) from the previous row v0

        // first element of v1 is A[i+1][0]
        //   edit distance is delete (i+1) chars from s to match empty t
        v1[0] = i + 1;

        // use formula to fill in the rest of the row
        for (int j = 0; j < t.length(); j++)
        {
            bool cost = (s[i] == t[j]) ? 0 : 1;
            v1[j + 1] = std::min(v1[j] + 1, std::min(v0[j + 1] + 1, v0[j] + cost));
            v1[j + 1] = std::max(v1[j + 1],0);
        }

        // copy v1 (current row) to v0 (previous row) for next iteration
        for (int j = 0; j < t.length(); j++)
            v0[j] = v1[j];
    }

    return v1[t.length()];
}

void Callback(const std_msgs::String::ConstPtr& msg)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string raw = msg->data.c_str();
  std::size_t pos_s = raw.find(":");
  std::string plate = raw.substr(0,pos_s);
  std::size_t pos_e = raw.find("B");
  std::string conf = raw.substr (pos_s + 2, pos_e - pos_s -4);
  pos_s = raw.find("G");
  float blue = std::atof(raw.substr (pos_e + 1, pos_s - pos_e -2).c_str());
  pos_e = raw.find("R");
  float green = std::atof(raw.substr (pos_s + 1, pos_e - pos_s -2).c_str());
  float red = std::atof(raw.substr (pos_e + 1).c_str());
  //std::cout << "Plate "<<plate <<" conf "<<conf<<" B "<<blue<<" G " << green <<" R "<< red << std::endl;
  bool flag = false;
  if(!cars.size())
  {
  	cars.push_front(car());
  	cars.front().id = 0;
  	cars.front().confidence = std::atof(conf.c_str());
  	cars.front().plate = plate;
  	cars.front().last_appear = ros::Time::now();
  	cars.front().tot_seen = 1;
  	cars.front().car_marker.header.frame_id = "/odom";
  	cars.front().car_marker.header.stamp = ros::Time::now();
  	cars.front().car_marker.ns = "cars";
  	cars.front().car_marker.action = visualization_msgs::Marker::ADD;
  	cars.front().car_marker.id = 0;
	//cars.front().car_marker.type = visualization_msgs::Marker::CUBE;
	cars.front().car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	cars.front().car_marker.mesh_resource = "package://alpr_pose/mesh/volks.dae";
	cars.front().car_marker.scale.x = 0.7;
  	cars.front().car_marker.scale.y = 0.7;
  	cars.front().car_marker.scale.z = 0.7;
  	cars.front().car_marker.lifetime = ros::Duration(0);
  	cars.front().car_marker.color.b = blue/255.0f;
  	cars.front().car_marker.color.g = green/255.0f;
  	cars.front().car_marker.color.r = red/255.0f;
  	cars.front().car_marker.color.a = 1;
  	count++;
	std::cout << "Added new car: " << count - 1 << " license plate: "<< plate << std::endl;
  	try {
		    listener.waitForTransform("odom", "car_pose", ros::Time(0), ros::Duration(3.0) );
		    listener.lookupTransform("odom", "car_pose", ros::Time(0), transform);
		    cars.front().car_marker.pose.position.x = transform.getOrigin().x() - 2;
		    cars.front().car_marker.pose.position.y = transform.getOrigin().y() - 2;
		    cars.front().car_marker.pose.position.z = transform.getOrigin().z();
		    transform.setRotation(transform.getRotation() * tf::createQuaternionFromYaw(M_PI/2) );
		    cars.front().car_marker.pose.orientation.x = transform.getRotation().x();
		    cars.front().car_marker.pose.orientation.y = transform.getRotation().y();
		    cars.front().car_marker.pose.orientation.z = transform.getRotation().z();
		    cars.front().car_marker.pose.orientation.w = transform.getRotation().w();

		    marker_pub.publish(cars.front().car_marker);
		}
	catch (tf::TransformException ex) 
		{
    		ROS_ERROR("%s",ex.what());
		}
  }
  else
  {
  	for (it = cars.begin(); it != cars.end(); ++it)
  	{
  		if(LevenshteinDistance(plate,it->plate) < 3)
  		{
  			std::cout << "Found car: " << it->id << " license plate: "<< it->plate << std::endl;
  			marker_pub.publish(it->car_marker);
  			flag = true;
  			it->tot_seen++;
  			it->last_appear = ros::Time::now();
  			if(it->confidence  < std::atof(conf.c_str()) - 1)
  			{
  				it->confidence = std::atof(conf.c_str());
  				it->plate = plate;
  				std::cout << "License plate updated to: "<< it->plate << std::endl;
  			}
  			break;
  		}
  		else
  		{
  			if((ros::Time::now() - it->last_appear) > thresh_dur && it->tot_seen < thresh_seen)
  			{
  				std::cout << "Erroneous license plate removed: "<< it->plate << std::endl;
  				it = cars.erase(it);
  			}

  		}
  		//std::cout << it->id << it->confidence<< it->plate << std::endl;
  	}
  	if(flag == false)
	{	
		cars.push_front(car());
		cars.front().id = count;
		cars.front().confidence = std::atof(conf.c_str());
		cars.front().plate = plate;
		cars.front().last_appear = ros::Time::now();
  		cars.front().tot_seen = 1;
  		cars.front().car_marker.header.frame_id = "/odom";
	  	cars.front().car_marker.header.stamp = ros::Time::now();
	  	cars.front().car_marker.ns = "cars";
	  	cars.front().car_marker.action = visualization_msgs::Marker::ADD;
	  	cars.front().car_marker.id = count;
	  	//cars.front().car_marker.type = visualization_msgs::Marker::CUBE;
	  	cars.front().car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  		cars.front().car_marker.mesh_resource = "package://alpr_pose/mesh/volks.dae";
  		cars.front().car_marker.scale.x = 0.7;
  		cars.front().car_marker.scale.y = 0.7;
  		cars.front().car_marker.scale.z = 0.7;
	  	cars.front().car_marker.lifetime = ros::Duration(0);
	  	cars.front().car_marker.color.b = blue/255.0f;
	  	cars.front().car_marker.color.g = green/255.0f;
	  	cars.front().car_marker.color.r = red/255.0f;
	  	cars.front().car_marker.color.a = 1;
	  	count++;
	  	std::cout << "Added new car: " << count - 1 << " license plate: "<< plate << " Total cars: " << cars.size() << std::endl;
  	try {
		    listener.waitForTransform("odom", "car_pose", ros::Time(0), ros::Duration(3.0) );
		    listener.lookupTransform("odom", "car_pose", ros::Time(0), transform);
		    cars.front().car_marker.pose.position.x = transform.getOrigin().x() - 2; //Compensation for rotation
		    cars.front().car_marker.pose.position.y = transform.getOrigin().y() - 2;
		    cars.front().car_marker.pose.position.z = transform.getOrigin().z();
		    transform.setRotation(transform.getRotation() * tf::createQuaternionFromYaw(M_PI/2) );
		    cars.front().car_marker.pose.orientation.x = transform.getRotation().x();
		    cars.front().car_marker.pose.orientation.y = transform.getRotation().y();
		    cars.front().car_marker.pose.orientation.z = transform.getRotation().z();
		    cars.front().car_marker.pose.orientation.w = transform.getRotation().w();

		    marker_pub.publish(cars.front().car_marker);
		}
	catch (tf::TransformException ex) 
		{
    		ROS_ERROR("%s",ex.what());
		}
	}
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "car_tagger");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("license_plate", 10, Callback);
	marker_pub = nh.advertise<visualization_msgs::Marker>("car_marker", 10);
	ros::spin();
	return 0;
}