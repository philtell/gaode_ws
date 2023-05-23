//
// Created by philtell on 23-5-23.
//

#include "../include/GDCamera/GDCamera.hpp"

namespace gaode_camera
{
	void GDCamera::Init()
	{

		m_handle = SGP_InitDevice();
		if (m_handle)
		{
			//成功，TODO......
		}
		else
		{
			//失败，TODO......
		}

	}

	cv::Mat GDCamera::getImage()
	{
		if(SGP_OK == SGP_Login(m_handle,ip_,username_,password_,80))
		{
			SGP_OpenIrVideo(m_handle, GetIrRtsp, this);
		}
		return image_mat;
	}

	static void GDCamera::GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr)
	{
		if(outdata)
		{
			cv::Mat mat(h, w, CV_8UC3, outdata);
			cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
			image_mat = mat.copy();
		}
	}

	GDCamera::~GDCamera()
	{
		SGP_Logout(m_handle);
		SGP_UnInitDevice(m_handle);
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