#include <ros/ros.h>
#include <sonar_msgs/sonar_raw_data.h>
#include <sonar_msgs/sonar_processed_data.h>
#include <vector>
#include <Eigen/Dense>

// OPENCV for debugging NODE
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// Quality of life
using namespace std;
using namespace Eigen;
using namespace cv;

// Declaring structures
struct binAndIntensity {
    vector<int> num_bin;
    vector<int> intensity;
};

struct rangeAndAngle {
    vector<float> range;
    vector<float> angle;
};

class sonarProcessor{
    private:
    // ROS handler
    ros::NodeHandle nh;

    // ROS parameters
    ros::Publisher sonar_data_pub;
    ros::Subscriber sonar_raw_data_sub;

    // ROS params
    string pub_topic, sub_topic;

    public:
    sonarProcessor() {
        // Reading ROS parameters
        nh.getParam("sonar_processor_sub_topic",sub_topic);
        nh.getParam("sonar_processor_pub_topic",pub_topic);

        // Initiate subscribers
        sonar_raw_data_sub = nh.subscribe(sub_topic,1000,&sonarProcessor::sonarCallback, this);
        // Initiate publisher
        sonar_data_pub = nh.advertise<sonar_msgs::sonar_processed_data>(pub_topic,100);
    }
    
    ~sonarProcessor() {

    }

    // Callback functions
    void sonarCallback(const sonar_msgs::sonar_raw_data::ConstPtr& msg);

};