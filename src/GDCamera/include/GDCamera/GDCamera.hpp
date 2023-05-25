//
// Created by philtell on 23-5-23.
//

#ifndef GAODE_WS_GDCAMERA_HPP
#define GAODE_WS_GDCAMERA_HPP
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "SgpApi.h"
#include "SgpParam.h"

using namespace std;

namespace gaode_camera
{
	class GDCamera
	{
	public:
		GDCamera();
		~GDCamera();
		cv::Mat getGDCamera();
	private:
		void Init();
		void getImage();
		static void GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr);
		string ip_;
		string username_;
		string password_;
		SGP_HANDLE  m_handle;
		static cv::Mat image_mat;
	};

} // gaode_camera

#endif //GAODE_WS_GDCAMERA_HPP
