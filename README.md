#VINS (Beta test)
Visual-Inertial System

**Authors:** Tong QIN, Zhenfei YANG, and [Shaojie SHEN](https://scholar.google.com.hk/citations?user=u8Q0_xsAAAAJ&hl=en) from [HUKST UAV GROUP](http://uav.ust.hk/)

VINS is a real-time SLAM for **Visual-Inertial** system. This code implements an initilization-free **monocular** sliding window based Visual-Inertial odometry and a basic AR demo, which is running on ROS ubuntu. 

**Videos:**

<a href="https://www.dropbox.com/s/b2gr4pd8n2pplnk/ar_demo.mp4?dl=0" target="_blank"><img src="https://github.com/qintony/VINS/blob/devel/support_material/img/small_ar.png" 
alt="ar_demo" width="280" height="210" border="30" /></a>
<a href="http://www.ece.ust.hk/~eeshaojie/icra2017tong.mp4" target="_blank"><img src="https://github.com/qintony/VINS/blob/devel/support_material/img/robust_initial.png" 
alt="robust_initial" width="280" height="210" border="30" /></a>
<a href="https://www.dropbox.com/s/btsyibhr5aq59r4/tase_final_compressed_480p_500kbps.mp4?dl=0" target="_blank"><img src="https://github.com/qintony/VINS/blob/devel/support_material/img/tase.png" 
alt="ar_demo" width="280" height="210" border="30" /></a>

**Related Papers**
* **Robust Estimator Initialization with Gyroscope Bias Calibration for Monocular Visual-Inertial Systems**(not published yet)
* **Monocular Visual-Inertial State Estimation With Online Initialization and Camera-IMU Extrinsic Calibration**
* **Tightly-coupled monocular visual-inertial fusion for autonomous flight of rotorcraft MAVs**

##1. Prerequisites
### 1.1 Ubuntu and ROS
Ubuntu 12.04, 14.04.

ROS Indigo, jade. [ROS Installation](http://wiki.ros.org/indigo/Installation/Ubuntu)

additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```
### 1.2 Ceres Solver
Following [Ceres Installation](http://ceres-solver.org/installation.html), remeber to make install.

##2. Build VINS on ROS
Clone the repository and catkin_make:
```
	cd catkin_ws/src
	git clone git@github.com:qintony/VINS.git
	cd ../
	catkin_make
```

##3. Monocular Examples on EuRoC dataset
1. Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera.

2. Modify path in VINS/vins_estimator/launch/euroc_dataset.launch. Change **dataset_bag_path** to the your downloaded dataset folder. Change **dataset_name** to MH_01_easy, MH_02_easy, MH_03_medium, MH_04_difficult, MH_05_difficult, V1_01_easy, V1_02_medium, V1_03_difficult, V2_01_easy, V2_02_medium, V2_03_difficult. When you enter the following command, wait a moment to load bag file. In RVIZ, you will see the raw image, tracked image, path and feature points.
```
	roslaunch vins_estimator euroc_dataset.launch
```
We test all the datas in EuRoc dataset, followings are the thumbnail of our result (Green line is VINS result, red line is ground truth). 

<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_01/MH_01.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_02/MH_02.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_03/MH_03.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_04/MH_04.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_05/MH_05.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_01/V1_01.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_02/V1_02.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_03/V1_03.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_01/V2_01.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_02/V2_02.png width="280" height="210" />
<img src=https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_03/V2_03.png width="280" height="210" />

We put our detailed results in (/support_material/dataset_result/). [MH_01](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_01/result.png)，
[MH_02](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_02/result.png)，
[MH_03](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_03/result.png)，
[MH_04](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_04/result.png)，
[MH_05](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/MH_05/result.png)，
[V1_01](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_01/result.png)，
[V1_02](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_02/result.png)，
[V1_03](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V1_03/result.png)，
[V2_01](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_01/result.png)，
[V2_02](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_02/result.png)，
[V2_03](https://github.com/qintony/VINS/blob/devel/support_material/dataset_result/V2_03/result.png)
We run these dataset with WINDOW_SIZE 15, MAX_SOLVER_TIME 60ms, MAX_FEATURE_COUNT 200 in real time. If you carefully set the parameters and give more computational resource, you will get better performance.

##4. AR Demo
1. Download the [bag file](https://www.dropbox.com/s/ap6s6w22rvg1c04/ar_box.bag?dl=0), witch is collected form HKUST Robotic Insititue.

2. Modify path in VINS/ar_demo/launch/online_bag.launch. Change **dataset_bag_path** to the your downloaded dataset folder.

3. Run the following command
```
	roslaunch ar_demo online_bag.launch
```
	We put one virtual boxed in front of your view. 

##5. Run with your device 

	Suppose you are familiar with ROS and you can get camera and IMU in ROS topic, you can follow these steps to set up your device. Note that it's not easily to do these, be careful and patient.

1. Camera calibration

	It's necessary to have accurate intrinsic parameters. You can calibrated camera by any tools you like, you just need to write your calibrated results into the fixed format in (featuretrack/config/). Our framework support PINHOLE model (common in OPENCV and ROS) and [MEI model](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf). MEI model is more suitable for wide angle camera.

	If you don't have prefered calibration tools, you can use the one contained in our framework. We use [camodocal](https://github.com/hengli/camodocal) to help us calibrate the camera intrinsic parameters. You can choose to use MEI or PINHOLE model.  The following commod will tell you how to use [camodocal](https://github.com/hengli/camodocal).
	```
		rosrun camera_model Calibration --help
	```
	The calibration result is put in (featuretrack/config/). Meanwhile, make sure to change correspoinding camera name in VINS lauch file.  

2. Camera-Imu extrinsic parameters.

	If you know the exactily extrinsic parameters, you can direcly write it into (vins_estimator/config/), and set ESTIMATE_EXTRINSIC false in launch file. If your parameter is from kalibr, use the inverse. 

	If you don't know the accurate extrinsic parameters, don't worry, our code will help you calibrate extrinsix parameters. Firstly, you should wirte an initial guess into (vins_estimator/config/), and set ESTIMATE_EXTRINSIC true in launch file. After you launch the whole system, do smooth movement in all direction and all orientation. If everything goes well, the self_calibration result will be writen into (vins_estimator/config/ex_calib_result.yaml), next time you directly use these result, and set ESTIMATE_EXTRINSIC false. Note that you had better give a reasonble intial guess, no more than 30 degrees error in rotation. In most cases, camera and imu are orthogonal fixed, so than you can easily figure out the initial extrinsic parameters by eye. 

3. Parameters setting

	As sliding window based method, the first parameter you can change is WINDOW_SIZE (vins_estimator/parameters.h). Default value is 15, it depends on your computation resources, the larger WINDOW_SIZE, the better performace, however, the more copuation needed.

	Other usual parameters are set in the launch file. We write breif description for these paramters in launch file. Remeber that the more accurate parameter you set, the better perfomance you get. 

##6. Acknowledgement
1. camera model. The camera model we use is from [camodocal](https://github.com/hengli/camodocal). To make user easily build, we directly put the source code in our file. 
2. some useful basic package form [Tianbo LIU](https://github.com/groundmelon).

##7. Future work
1. Release multi-camera code
2. Release IOS/Android code
3. Add loop closure
4. Calibrate camera intrinsic parameter

##8. License

Welcome to any issue! Ang question please contact tong.qin@connect.ust.hk