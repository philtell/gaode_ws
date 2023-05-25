#include "../include/GDCamera/SgpApi.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "../include/GDCamera/GDCamera.hpp"

int main(int argc,char **argv)
{
	ros::init(argc, argv, "gd_camera_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image_raw", 10);
	gaode_camera::GDCamera gdCamera;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		cv::Mat img = gdCamera.getGDCamera();
		cv::imshow("gd img",img);
		ROS_INFO("Read Img from gd camera");
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		pub.publish(msg);
		loop_rate.sleep();
	}
	ros::spin();
    return 0;
}