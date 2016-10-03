
#include <cstdio>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <string>
#include <cstdlib>

#include <ros/ros.h>
#include "std_msgs/String.h"

struct car{
	int id;
	double confidence;
	std::string plate;

};
int count = 0;
std::vector<car> cars;
std::vector<car>::iterator it;
int LevenshteinDistance(std::string s, std::string t)
{
    // degenerate cases
    if (s == t) return 0;
    if (s.length() == 0) return t.length();
    if (t.length() == 0) return s.length();

    // create two work vectors of integer distances
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
  std::string raw = msg->data.c_str();
  std::size_t pos = raw.find(":");
  std::string conf = raw.substr (pos + 2);
  raw.resize(pos);
  bool flag = false;
  if(!cars.size())
  {
  	cars.push_back(car());
  	cars[0].id = 0;
  	cars[0].confidence = std::atof(conf.c_str());
  	cars[0].plate = raw;
  	std::cout << cars[0].id << cars[0].confidence<< cars[0].plate << std::endl;
  	count++;
  }
  else
  {
  	for (it = cars.begin(); it != cars.end(); ++it)
  	{
  		if(LevenshteinDistance(raw,it->plate) < 4)
  		{
  			std::cout << "Found car: " << it->id << " license plate: "<< it->plate << std::endl;
  			flag = true;
  			if(it->confidence < std::atof(conf.c_str()))
  			{
  				it->confidence = std::atof(conf.c_str());
  				it->plate = raw;
  				std::cout << "License plate updated to: "<< it->plate << std::endl;
  			}
  			break;
  		}
  		//std::cout << it->id << it->confidence<< it->plate << std::endl;
  	}
  	if(flag == false)
	{	
		cars.push_back(car());
		cars[count].id = count;
		cars[count].confidence = std::atof(conf.c_str());
		cars[count].plate = raw;
		std::cout << "Added new car: " << count << " license plate: "<< raw << std::endl;
		count++;
	}
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "car_tagger");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("license_plate", 10, Callback);
	ros::spin();
	return 0;
}