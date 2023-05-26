//
// Created by philtell on 23-5-23.
//

#include "../include/GDCamera/GDCamera.hpp"

namespace gaode_camera
{
	cv::Mat GDCamera::image_mat = cv::Mat();
	cv::Mat GDCamera::getGDCamera()
	{
		getImage();
		return this->image_mat;
	}
	void GDCamera::Init()
	{

		m_handle = SGP_InitDevice();
		if (m_handle)
		{
			ROS_INFO("SGP_InitDevice SUCCESS!");
			//成功，TODO......
			if(SGP_OK == SGP_Login(m_handle,ip_.c_str(),username_.c_str(),password_.c_str(),80))
			{
				ROS_INFO("Init SGP_Login SUCCESS!");
				SGP_FOCUS_TYPE type = SGP_FOCUS_AUTO; // 电机位置值0~750，当type传入SGP_FOCUS_PLACE有效
				int value =0;
				int ret = SGP_SetFocus(m_handle,type,value);
				if (ret == SGP_OK )
				{
					  //成功，TODO......
					ROS_INFO("SGP_SetFocus SUCCESS!");
				}
				else
				{
					ROS_ERROR("SGP_SetFocus FAILED!");
					  //失败，TODO......
				} 
			}
			else
			{
				ROS_ERROR("Init SGP_Login FAILED!");
			}
			//getImage();
		}
		else
		{
			ROS_ERROR("SGP_InitDevice Failed!");
			//失败，TODO......
		}

	}

	void GDCamera::getImage()
	{
		if(SGP_OK == SGP_Login(m_handle,ip_.c_str(),username_.c_str(),password_.c_str(),80))
		{
			ROS_INFO("SGP_Login SUCCESS!");
			SGP_OpenIrVideo(m_handle, GetIrRtsp, this);
		}
		else
		{
			ROS_ERROR("SGP_Login Failed!ip username password or port is incorrect!!! please rectify it try again!!");
		}
	}

	void GDCamera::GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr)
	{
		if(outdata)
		{
			cv::Mat mat(h, w, CV_8UC3, outdata);
			cv::cvtColor(mat, image_mat, cv::COLOR_RGB2BGR);
			ROS_INFO("GetIrRtsp SUCCESS!");
		}
		else
		{
			ROS_ERROR("GetIrRtsp Failed!");
		}
	}

	GDCamera::~GDCamera()
	{
		SGP_Logout(m_handle);
		ROS_INFO("SGP_Logout SUCCESS!");
		SGP_UnInitDevice(m_handle);
		ROS_INFO("SGP_UnInitDevice SUCCESS!");
		image_mat.release();
	}

	GDCamera::GDCamera()
	{
		ip_ = "192.168.1.168";
		username_ = "admin";
		password_ = "admin123";
		m_handle = 0;
		Init();
	}
} // gaode_camera
