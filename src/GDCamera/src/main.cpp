#include "SgpApi.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
SGP_HANDLE  handle = 0;

void Init()
 {
      
      handle = SGP_InitDevice();
      if (handle)
      {
          //成功，TODO......
      }
      else
      {
          //失败，TODO......
      }
 }

int main(int argc,char **argv)
{
    ros::init(argc,argv);
    SGP_HANDLE m_handle = 0;
    //生成一个SGPSDK的独立实例，之后所有函数操作
    //都作用于这个实例。可多次调用此函数生成多个实例，
    //多个实例之间的操作互不影响。
    m_handle = SGP_InitDevice(); // 创建实例

    return 0;
}