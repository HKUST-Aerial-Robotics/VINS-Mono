# run the system with our device

It is forked from HKUST and we try to run it with our IMU and camera.

## camera: the left eye of the ZED stereo camera.
![zed](https://www.stereolabs.com/documentation/overview/getting-started/images/ZED-Camera.png)
 
 [document of it](https://www.stereolabs.com/documentation/overview/positional-tracking/coordinate-frames.html)

## IMU:LPMS-CU2

![IMU](http://www.alubi.cn/wp-content/uploads/2016/08/LpmsCU2_860%C3%97470_20170210.jpg)


[Chinese document of LPMS-CU2](http://www.alubi.cn/lpms-cu2/ ) >> 资料下载 >> [数据手册](http://www.alubi.cn/wp-content/uploads/2016/08/Lpms-CU2Flyer-20170421cn.pdf) / [快速使用手册](http://www.alubi.cn/wp-content/uploads/2016/08/LpmsCU2%E5%BF%AB%E9%80%9F%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C20161013.pdf)

[English document](https://www.lp-research.com/site/wp-content/uploads/2016/10/LpmsCU2QuickStartGuide20161013.pdf)

[the software to control lpms on Windows](https://www.lp-research.com/support/) >> OPENMAT BINARIES >> OpenMAT


# the frame of our system


![frame](http://oljkaeely.bkt.clouddn.com/frame.png)




# Launch it

build it
```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Yvon-Shong/VINS-Mono.git
cd ../
catkin_make
```

run it
```bash
roslaunch zed_wrapper zed.launch
rosrun lpms_imu imu_data
roslaunch vins_estimator ZED.launch
roslaunch vins_estimator vins_rviz.launch
```


# What we have changed
we didn't edit the core code of the program.
we just:
```
add config/ZED/ZED_config.yaml  
add vins_estimator/launch/ZED.launch

```

the parameters of the ZED_config.yaml are suitable for our device, IMU:LPMS-CU2 and Camera:ZED Stereo Camera


