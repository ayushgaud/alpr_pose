
#include <cstdio>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <algorithm>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/version.hpp"

#include "alpr.h"

#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>



const std::string MAIN_WINDOW_NAME = "ALPR main window";


/** Function Headers */
bool detectandshow(alpr::Alpr* openalpr, cv::Mat frame);

bool measureProcessingTime = false;


std::string path = ros::package::getPath("alpr_pose");
alpr::Alpr openalpr("gb", path + "/config/openalpr.conf");
ros::Publisher pose_pub; 
ros::Publisher plate_pub;
image_transport::Publisher pub;
std_msgs::String msg;

std::vector<double> f_data (5, 0);
std::vector<double> f_sort (5, 0);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    detectandshow(&openalpr, cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main( int argc, char** argv )
{
 // You can use other countries/regions as well (for example: "eu", "au", or "kr")
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
  std::cout << "CV2" << std::endl;
#elif CV_MAJOR_VERSION == 3
  std::cout << "CV3" << std::endl;
// do opencv 3 code
#endif 
 // Optionally specify the top N possible plates to return (with confidences).  Default is 10
 openalpr.setTopN(5);
 // Optionally, provide the library with a region for pattern matching.  This improves accuracy by
 // comparing the plate text with the regional pattern.
 openalpr.setDefaultRegion("base");

 // Make sure the library loaded before continuing.
 // For example, it could fail if the config/runtime_data is not found
 if (openalpr.isLoaded() == false)
 {
     std::cerr << "Error loading OpenALPR" << std::endl;
     return 1;
 }

 ros::init(argc, argv, "alpr_node");
 ros::NodeHandle nh;
 //cv::namedWindow("view");
 //cv::startWindowThread();
 image_transport::ImageTransport it(nh);
 image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10, imageCallback);
 pub = it.advertise("/alpr_image", 1);
 plate_pub = nh.advertise<std_msgs::String>("license_plate", 10);
 pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/alpr_pose",10);
 ros::spin();
 //cv::destroyWindow("view");

}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
 
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
          
}

bool detectandshow( alpr::Alpr* openalpr, cv::Mat frame )
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
  //timespec startTime;
  //getTimeMonotonic(&startTime);

  std::vector<alpr::AlprRegionOfInterest> regionsOfInterest;
  
  regionsOfInterest.push_back(alpr::AlprRegionOfInterest(0, 0, frame.cols, frame.rows));
  alpr::AlprResults results;
  if (regionsOfInterest.size()>0) results = openalpr->recognize(frame.data, frame.elemSize(), frame.cols, frame.rows, regionsOfInterest);

  //timespec endTime;
  //getTimeMonotonic(&endTime);
  //double totalProcessingTime = diffclock(startTime, endTime);
  if (measureProcessingTime);
    //std::cout << "Total Time to process image: " << totalProcessingTime << "ms." << std::endl;
  //cv::imshow("view", frame);
  //cv::waitKey(30);
  if(results.plates.size() > 0) 
    {
      cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
      bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
                                        // initial approximations of the rotation and translation vectors

      // RANSAC parameters
      int iterationsCount = 500;        // number of Ransac iterations.
      float reprojectionError = 2.0;    // maximum allowed distance to consider it an inlier.
      float confidence = 0.95;          // ransac successful confidence.
      cv::Mat inliers, _A_matrix, _R_matrix, _P_matrix, _t_matrix;
      cv::Mat img = frame;
      double f = 55;                           // focal length in mm
      double sx = 22.3, sy = 14.9;             // sensor size
      double width = 640, height = 368;        // image size
      double params[] = { width*f/sx,   // fx
                          height*f/sy,  // fy
                          width/2,      // cx
                          height/2};    // cy
      _A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
      _A_matrix.at<double>(0, 0) = 396.17782 ;//params[0];       //      [ fx   0  cx ]
      _A_matrix.at<double>(1, 1) = 399.798333;//params[1];       //      [  0  fy  cy ]
      _A_matrix.at<double>(0, 2) = 322.453185;//params[2];       //      [  0   0   1 ]
      _A_matrix.at<double>(1, 2) = 174.243174;//params[3];
      _A_matrix.at<double>(2, 2) = 1;
      _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
      _t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
      _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix

      std::vector<cv::Point2f> list_points2d;
      std::vector<cv::Point3f> list_points3d;
      double length = 0;
      
      length = 53.33*(results.plates[0].bestPlate.character_details.size()) - 10;
      list_points3d.push_back(cv::Point3f(0,65,0));
      list_points3d.push_back(cv::Point3f(length,65,0));
      list_points3d.push_back(cv::Point3f(length,0,0));
      list_points3d.push_back(cv::Point3f(0,0,0));

      //Draw green bounding box around each detected character    
      for (int i = 0; i < results.plates.size(); i++)
      {
        float height = 0;
        int q;
       for (q = 0; q < results.plates[i].bestPlate.character_details.size(); q++)
          {
            alpr::AlprChar details = results.plates[i].bestPlate.character_details[q];
            height = height + std::sqrt(std::pow(details.corners[2].y - details.corners[1].y,2) + std::pow(details.corners[2].x - details.corners[1].x,2));
            line(img, cv::Point(details.corners[0].x, details.corners[0].y), cv::Point(details.corners[1].x, details.corners[1].y), cv::Scalar(0,255,0), 1);
            line(img, cv::Point(details.corners[1].x, details.corners[1].y), cv::Point(details.corners[2].x, details.corners[2].y), cv::Scalar(0,255,0), 1);
            line(img, cv::Point(details.corners[2].x, details.corners[2].y), cv::Point(details.corners[3].x, details.corners[3].y), cv::Scalar(0,255,0), 1);
            line(img, cv::Point(details.corners[3].x, details.corners[3].y), cv::Point(details.corners[0].x, details.corners[0].y), cv::Scalar(0,255,0), 1);
            //std::cout<<"i: "<<i << " q: "<<q<<std::endl;
          }
        //height = height/q;
        //std::cout <<"Avg Height: "<< height << std::endl;
        
      }
      
      //std::cout << "debug: " << results.plates[0].bestPlate.character_details.size();    
      alpr::AlprChar First_details = results.plates[0].bestPlate.character_details[0];
      alpr::AlprChar Last_details = results.plates[0].bestPlate.character_details[results.plates[0].bestPlate.character_details.size()-1];

      list_points2d.push_back(cv::Point2f(First_details.corners[0].x,First_details.corners[0].y));
      list_points2d.push_back(cv::Point2f(Last_details.corners[1].x,Last_details.corners[1].y));
      list_points2d.push_back(cv::Point2f(Last_details.corners[2].x,Last_details.corners[2].y));
      list_points2d.push_back(cv::Point2f(First_details.corners[3].x,First_details.corners[3].y));
      //std::cout << "Debug info : " << list_points2d << std::endl;
      //std::cout << "Debug info : " << list_points3d << std::endl;

      //solvePnP(list_points3d, list_points2d,_A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess, CV_ITERATIVE);
      solvePnPRansac(list_points3d, list_points2d,_A_matrix, distCoeffs, rvec, tvec,
                          useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers, CV_ITERATIVE);

      Rodrigues(rvec,_R_matrix);                   // converts Rotation Vector to Matrix
      _t_matrix = tvec;                            // set translation matrix
      
      //std::cout << "Translation is: " << _t_matrix << std::endl << "Rotation is: " << rotationMatrixToEulerAngles(_R_matrix) << std::endl;
      //Draw axis
      //
      /*
      std::vector<cv::Point3f> axis;
      axis.push_back(cv::Point3f(100,0,0));
      axis.push_back(cv::Point3f(0,100,0));
      axis.push_back(cv::Point3f(0,0,20));
      //cv::Mat axis = (cv::Mat_<double>(3,3) << 80, 0, 0, 0, 80, 0, 0, 0, -80);//Axis in world frame
      std::vector<cv::Point2f> imagePoints;               //Projection in image frame
      std::vector<cv::Point2f> reproj;
      projectPoints(axis, rvec, tvec, _A_matrix, distCoeffs, imagePoints);//Compute projection

      line(img, list_points2d[3], imagePoints[0], cv::Scalar(255,0,0), 3, CV_AA);
      line(img, list_points2d[3], imagePoints[1], cv::Scalar(0,255,0), 3, CV_AA);
      line(img, list_points2d[3], imagePoints[2], cv::Scalar(0,0,255), 3, CV_AA);
      */

      //Draw a bounding box around the detected characters
      line(img, list_points2d[3], list_points2d[2], cv::Scalar(0,0,255), 1, CV_AA);
      line(img, list_points2d[2], list_points2d[1], cv::Scalar(0,0,255), 1, CV_AA);
      line(img, list_points2d[1], list_points2d[0], cv::Scalar(0,0,255), 1, CV_AA);
      line(img, list_points2d[0], list_points2d[3], cv::Scalar(0,0,255), 1, CV_AA);
      
      //Section to applly median filtering
      f_data.erase(f_data.begin());
      f_data.push_back(tvec.at<double>(2)/1000);
      f_sort = f_data;
      std::sort(f_sort.begin(), f_sort.end());

      double median = f_sort[f_sort.size()/2];

      //Pose publisher
      geometry_msgs::PoseStamped _pose;
      _pose.pose.position.x = tvec.at<double>(0)/1000;
      _pose.pose.position.y = tvec.at<double>(1)/1000;
      _pose.pose.position.z = median;
      
      _pose.header.stamp = ros::Time::now();
      _pose.header.frame_id = "camera";
      pose_pub.publish(_pose);


      //TF broadcaster and frame conversion to ENU

	    /*tf::Matrix3x3 cameraRotation_rh(_R_matrix.at<float>(0,0),   _R_matrix.at<float>(0,1),   _R_matrix.at<float>(0,2),
	                                    _R_matrix.at<float>(1,0),   _R_matrix.at<float>(1,1),   _R_matrix.at<float>(1,2),
	                                    _R_matrix.at<float>(2,0),   _R_matrix.at<float>(2,1),   _R_matrix.at<float>(2,2));
			*/

	    tf::Vector3 globalTranslation_rh( median, -tvec.at<double>(0)/1000, -tvec.at<double>(1)/1000);
	    cv::Vec3f rotation = rotationMatrixToEulerAngles(_R_matrix);
	    //tf::Vector3 euler(rotation[2], -rotation[0], -rotation[1]);
	    tf::Quaternion global_rotation;
	    global_rotation.setRPY(0,0,0);//(rotation[2], -rotation[0], -rotation[1])

      transform = tf::Transform(global_rotation, globalTranslation_rh);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base_link", "car_pose"));

      //For dubug purposes, prints the contents in the window of median filter 
      
      /*std::cout << "f_sort contains:";
      for (std::vector<double>::iterator it = f_sort.begin() ; it != f_sort.end(); ++it)
      std::cout << ' ' << *it;
      std::cout << '\n';
		  */
		  cv::Scalar color;
		  try
		  {
		  	color = cv::mean(img(cv::Rect(First_details.corners[0].x + 5, std::max(First_details.corners[0].y - 60, 0), 60, 15)));//Car average color
      	//cv::rectangle(img, cv::Rect(First_details.corners[0].x, std::max(First_details.corners[0].y - 60, 0), 60, 15), cv::Scalar(0,0,255));
		  }
		  catch(cv::Exception& e)
		  {
		  	const char* err_msg = e.what();
    		std::cout << "ROI exception caught: " << err_msg << std::endl;
		  	color = cv::Scalar(0, 0, 255);
		  }
      //cv::Scalar color = cv::Scalar(0, 0, 255);
      std::ostringstream strs;
      strs << std::setprecision(3) << median << "m";
      std::string str = strs.str();
      putText(img, str, cv::Point(frame.cols - 150, 30), 2, 1, cv::Scalar(0,255,0), 1, 8, false);

      //Show image with distance printed
      //cv::imshow("view", img);
      //cv::waitKey(30);
			sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    	pub.publish(img_msg);
			alpr::AlprPlateResult plate = results.plates[0];
			//std::cout << "plate" << ": " << plate.topNPlates.size() << " results" << std::endl;
			for (int k = 0; k < plate.topNPlates.size(); k++)
			{
			 alpr::AlprPlate candidate = plate.topNPlates[k];
			 //std::cout << "    - " << candidate.characters << "\t confidence: " << candidate.overall_confidence;
			 //std::cout << "\t pattern_match: " << candidate.matches_template << std::endl;
			 if(candidate.overall_confidence > 70)
				{
					std::ostringstream strs;
      		strs << candidate.overall_confidence << " B"<< color.val[0] << " G"<< color.val[1] << " R"<< color.val[2];
      		std::string str = strs.str();
			 		msg.data = candidate.characters + ": " + str;
			 		plate_pub.publish(msg);
			 	}
			}

    }
    else
    {
    	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    	pub.publish(img_msg);
    } 
  


  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base_link", "car_pose"));
  return results.plates.size() > 0;
}