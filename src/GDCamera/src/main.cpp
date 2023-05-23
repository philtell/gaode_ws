#include "SgpApi.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "../include/GDCamera/GDCamera.hpp"



int main(int argc,char **argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image", 10);
	gaode_camera::GDCamera gdCamera;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		cv::Mat img = gdCamera.getImage();
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		pub.publish(msg);
		loop_rate.sleep();
	}
	ros::spin();
    return 0;
}