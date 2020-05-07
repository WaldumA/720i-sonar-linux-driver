#include <ros/ros.h>
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Char.h>
#include <sonar_msgs/sonar_raw_data.h>

// Quality of life
using namespace Eigen;
using namespace std;

class sonarDriver {
private:
    // ROS handler
    ros::NodeHandle nh;

    // Scan variables
    int gain, sos, range;
    string pub_topic;

    
    // ROS parameters
    ros::Publisher sonar_data_pub;
    


public:
    sonarDriver() {
        // Reading ROS parameters
        nh.getParam("sonar_driver_gain",gain);
        nh.getParam("sonar_driver_sos",sos);
        nh.getParam("sonar_driver_range",range);
        nh.getParam("sonar_driver_pub_topic",pub_topic);
        

        // Initiate publisher
        sonar_data_pub = nh.advertise<sonar_msgs::sonar_raw_data>(pub_topic,1000);

        // Run driver
        run();
    }

    ~sonarDriver() {

    }

void run();
};