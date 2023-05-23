//
// Created by philtell on 23-5-23.
//

#ifndef GAODE_WS_GDCAMERA_HPP
#define GAODE_WS_GDCAMERA_HPP
#include <string>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

namespace gaode_camera
{

	class GDCamera
	{
	public:
		void Init();
		cv::Mat getImage();
		GDCamera();
		~GDCamera();
	private:
		static void GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr);
		string ip_;
		string username_;
		string password_;
		SGP_HANDLE  m_handle;
		cv::Mat image_mat;
	};

} // gaode_camera

#endif //GAODE_WS_GDCAMERA_HPP
