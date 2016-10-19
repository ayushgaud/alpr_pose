# Alpr Pose (ROS package)

A ROS node to extract pose information form the license plate using openalpr library (http://www.openalpr.com/) and tag cars in a map. It uses simple perspective and point technique to estimate the poses form the standard license plates used in India.

# Usage

Just clone the repository and build it using catkin_make command. Also modify the path of runtime_data in the openalpr.conf file in the config directory to point it to your runtime_data folder.

To run use the demo.launch file:  
```roslaunch alpr_pose demo.launch```

UPDATE: Intigration of map requires rviz satellite plugin. Repository URL https://github.com/gareth-cross/rviz_satellite

For any queries contact me directly at my Email ayush.gaud[at]gmail.com

![rviz_screenshot](https://cloud.githubusercontent.com/assets/4923897/19534873/3219b4dc-9663-11e6-86b5-8e471b30a509.png)
