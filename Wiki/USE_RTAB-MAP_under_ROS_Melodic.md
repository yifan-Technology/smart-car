# USE RTAB-MAP under ROS Melodic

## Install ROS melodic
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ source /opt/ros/melodic/setup.bash
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt install python-rosdep
$ sudo rosdep init
$ rosdep update
```

## Install LibRealsense and Realsense_ROS

- ### For normal PC:

  #### 1.Install LibRealsense

  Reference: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md 
  Install from debian binary libraries
  ```bash
  $ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  $ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
  $ sudo apt-get install librealsense2-dkms
  $ sudo apt-get install librealsense2-utils
  $ sudo apt-get install librealsense2-dev
  $ sudo apt-get install librealsense2-dbg
  ```
  but can also build from source with reference: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md. Recommended git checkout v2.38.1 when following the this installation guide, also git checkout in building.
  
  #### 2.Install Realsense_ROS

  Reference: https://github.com/IntelRealSense/realsense-ros/tree/2.2.17
  Adjust "catkin_ws" name under the following command lines  
  ```bash
  $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc
  ```
  ```bash
  # when need to create a new ros workspace, otherwise cd to the existing workspace directly
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src/
  $ git clone https://github.com/IntelRealSense/realsense-ros.git
  $ cd realsense-ros/
  $ git checkout 2.2.17
  
  $ cd ..
  # when new ros workspace
  $ catkin_init_workspace 
  $ cd ..
  $ catkin_make clean
  $ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  $ catkin_make install
  ```

- ### For jetson nano with JetPack: L4T 32.4.3, JP 4.4, 2020/07/07 installed:

  #### 1. Install LibRealsense

  Reference: https://www.jetsonhacks.com/2019/12/22/install-realsense-camera-in-5-minutes-jetson-nano/

  ```bash
  $ git clone https://github.com/JetsonHacksNano/installLibrealsense
  $ cd installLibrealsense
  # Build from source
  # Chnage LIBREALSENSE_VERSION=v2.31.0 to LIBREALSENSE_VERSION=v2.38.1 and
  # Change NVCC_PATH=/usr/local/cuda-10.0/bin/nvcc to NVCC_PATH=/usr/local/cuda-10.2/bin/nvcc in buildLibrealsense.sh
  $ ./buildLibrealsense.sh
  ```

  #### 2. Install Realsense_ROS

  Reference: https://www.jetsonhacks.com/2019/10/25/realsense-ros-wrapper-jetson-nano/  

  ```bash
  $ git clone https://github.com/JetsonHacksNano/installRealSenseROS
  $ cd installRealSenseROS
  # Change REALSENSE_ROS_VERSION=2.2.11 to REALSENSE_ROS_VERSION=2.2.17
  $ ./installRealSenseROS <catkin workplace name>
  # If <catkin workplace name> not existed, create it firstly
  ```
  
## Install RTAB-MAP
  ```bash
  $ sudo apt install ros-melodic-rtabmap-ros
  ```
  
## HOW TO USE
- ### In Terminal 1
  ```bash 
  $ roslaunch realsense2_camera rs_d400_and_t265.launch 
  ```
  Make sure change rs_d400_and_t265.launch first. The origin does not run. Changed version as below:
  ```xml
    <launch>
      <arg name="device_type_camera1"       default="t265"/>
      <arg name="device_type_camera2"       default="d435"/>    
      <arg name="serial_no_camera1"         default=""/>
      <arg name="serial_no_camera2"         default=""/>
      <arg name="camera1"                   default="t265"/>
      <arg name="camera2"                   default="d400"/>
      <arg name="tf_prefix_camera1"         default="$(arg camera1)"/>
      <arg name="tf_prefix_camera2"         default="$(arg camera2)"/>
      <arg name="initial_reset"             default="false"/>
      
      <arg name="color_width"               default="640"/>
      <arg name="color_height"              default="480"/>
      <arg name="depth_width"               default="640"/>
      <arg name="depth_height"              default="480"/>
      <arg name="clip_distance"             default="-2"/>
      <arg name="allow_no_texture_points"   default="false"/>
    
      <arg name="fisheye_width"       default="848"/> 
      <arg name="fisheye_height"      default="800"/>
      <arg name="enable_fisheye"      default="false"/>
      <arg name="fisheye_fps"         default="30"/>
      <arg name="gyro_fps"            default="200"/>
      <arg name="accel_fps"           default="62"/>
      <arg name="enable_gyro"         default="true"/>
      <arg name="enable_accel"        default="true"/>
      <arg name="enable_pose"         default="true"/>
      <arg name="enable_sync"         default="false"/>
      <arg name="linear_accel_cov"    default="0.01"/>
      <arg name="unite_imu_method"    default=""/>
    
      <arg name="publish_odom_tf"     default="true"/>
    
      <group ns="$(arg camera1)">
        <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
          <arg name="device_type"           value="$(arg device_type_camera1)"/>
          <arg name="serial_no"             value="$(arg serial_no_camera1)"/>
          <arg name="tf_prefix"             value="$(arg tf_prefix_camera1)"/>
          <arg name="initial_reset"         value="$(arg initial_reset)"/>
  
          <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
          <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
          <arg name="enable_fisheye1"       value="$(arg enable_fisheye)"/>
          <arg name="enable_fisheye2"       value="$(arg enable_fisheye)"/>
          <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
    
          <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
          <arg name="accel_fps"                value="$(arg accel_fps)"/>
          <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
          <arg name="enable_accel"             value="$(arg enable_accel)"/>
          <arg name="enable_pose"              value="$(arg enable_pose)"/>
           <arg name="enable_sync"              value="$(arg enable_sync)"/>
      
          <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
          <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
    
          <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
        </include>
      </group>
      
      <group ns="$(arg camera2)">
        <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
          <arg name="device_type"           value="$(arg device_type_camera2)"/>
          <arg name="serial_no"             value="$(arg serial_no_camera2)"/>
          <arg name="tf_prefix"             value="$(arg tf_prefix_camera2)"/>
          <arg name="initial_reset"         value="$(arg initial_reset)"/>
          <arg name="align_depth"           value="true"/>
          <arg name="filters"               value=""/>
          <arg name="color_width"           value="$(arg color_width)"/>
          <arg name="color_height"          value="$(arg color_height)"/>
          <arg name="depth_width"           value="$(arg depth_width)"/>
          <arg name="depth_height"          value="$(arg depth_height)"/>
          <arg name="clip_distance"         value="$(arg clip_distance)"/>
          <arg name="allow_no_texture_points"  value="$(arg allow_no_texture_points)"/>
        </include>
      </group>
      <node pkg="tf" type="static_transform_publisher" name="t265_to_d400" args="0 0 0.03 0 0 0 /$(arg tf_prefix_camera1)_link /$(arg tf_prefix_camera2)_link 100"/>
    </launch>
    ```
   #### Just Copy It into ~/<catkin_workspace>/src/realsense-ros/realsense2_camera/launch/rs_d400_and_t265.launch !
   **0 0 0.03 0 0 0**  in **<node pkg="tf" type="static_transform_publisher" name="t265_to_d400" args="0 0 0.03 0 0 0 /$(arg tf_prefix_camera1)_link /$(arg tf_prefix_camera2)_link 100"/>** mean the transformation between D435 and T65. Here d435 is placed directly above t265. The camera center of d435 is about 30cm above t265.
  
  If error occurres, plug out camera out, wait 5 seconds and then plug in again 
 
- ### In Terminal 2
  - Mapping mode
    ```bash
    $ roslaunch rtabmap_ros rtabmap.launch \
    args:="-d --Mem/UseOdomGravity true --Optimizer/GravitySigma 0.3" \
    odom_topic:=/t265/odom/sample \
    frame_id:=t265_link \
    rgbd_sync:=true \
    depth_topic:=/d400/aligned_depth_to_color/image_raw \
    rgb_topic:=/d400/color/image_raw \
    camera_info_topic:=/d400/color/camera_info \
    approx_rgbd_sync:=false \
    visual_odometry:=false \
    rtabmapviz:=false \
    rviz:=true
    ```
    Note that the generated map is stored under ~/.ros/rtabmap.db. When ctrl+c clicked, the map is stored automatically.
    
  - Localization mode
    ```bash
    $ roslaunch rtabmap_ros rtabmap.launch \
    args:="--Mem/UseOdomGravity true --Optimizer/GravitySigma 0.3" \
    odom_topic:=/t265/odom/sample \
    frame_id:=t265_link \
    rgbd_sync:=true \
    depth_topic:=/d400/aligned_depth_to_color/image_raw \
    rgb_topic:=/d400/color/image_raw \
    camera_info_topic:=/d400/color/camera_info \
    approx_rgbd_sync:=false \
    visual_odometry:=false \
    rtabmapviz:=false \
    rviz:=true \ 
    localiztion:=true
    ```
    Rviz startd automatically. You must firstly set the topics correctly. Recommended to set TF, Map. Fixed frame is to set to map. Then you will see your position in the loaded map.
    
- ## Use BAG File
  ### Record:
  ```bash
  $ rosbag record /d400/color/camera_info /d400/color/image_raw /d400/aligned_depth_to_color/camera_info /d400/aligned_depth_to_color/image_raw /tf /tf_static /t265/odom/sample 
   ```
   It is possible that the rs_d400_and_t265.launch does not run correctly. Recommended to run **rostopic list** first to see if all to record topics are listed. When not, then start rs_d400_and_t265.launch again or plug out and plug in cameras.
   
   ### Play bag file
   **First run static_transform_mux**. First install it:
   ```bash 
   $ cd <catkin workspace>/src
   $ git clone https://github.com/tradr-project/static_transform_mux.git
   $ cd ..
   $ catkin_make 
   ```
   Then play bag file:
   ```bash
   $ rosbag play <bagfile>.bag 
   ```
   Bag file is located where you run rosbag record.
  
