1. 编译前注意事项,修改目录为本地sdk linux64所在目录
set(SGD_PATH "/home/philtell/Downloads/SGP_SDK_V1.2.0.20220718_Release/SDK/linux64/x64Lib")
2. 编译程序命令catkin_make
3. 加载动态库，具体命令 sudo ldconfig /home/philtell/Downloads/SGP_SDK_V1.2.0.20220718_Release/SDK/linux64/x64Lib
4. 运行程序 rosrun GDCamera GDCamera_node  如果不对，注意修改ip,用户名，密码和端口(可能不需要修改)
5. 程序话题为 camera/image_raw
6. 录制话题rosbag record camera/image_raw

