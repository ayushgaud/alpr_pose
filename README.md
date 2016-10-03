# Alpr Pose (ROS package)

A ROS node to extract pose information form the license plate using openalpr library (http://www.openalpr.com/). It uses simple perspective and point technique to estimate the poses form the standard license plates used in India.

# Usage

Just clone the repository and build it using catkin_make command. Also modify the path of runtime_data in the openalpr.conf file in the config directory to point it to your runtime_data folder.
