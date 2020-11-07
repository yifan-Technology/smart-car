# Install ros2 Realsense and ros2 rtabmap for jetson

1. [Prerequisite](#Prerequisite)

    [Install ROS 2 Eloquent Elusor](#jump1)
    
    [Install ROS 2 Intel Realsense](#jump2)
    
    [Install ROS2 RTAB-Map](#jump3)
    
2. [Change some parameters of launch file](#jump4)
3. [How To Use](#jump5)

## Prerequisite
#### <span id="jump1">Install ROS 2 Eloquent Elusor</span>
[Reference](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/)

Install ros2 eloquent via debian packages
```cmd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-eloquent-desktop

source /opt/ros/eloquent/setup.bash
sudo apt install -y python3-pip
pip3 install -U argcomplete

echo 'source /opt/ros/eloquent/setup.bash' >> ~/.bashrc
```

#### <span id="jump2">Install ROS 2 Intel Realsense</span>
For jetson nano with JetPack: L4T 32.4.3, JP 4.4, 2020/07/07 installed:

  ##### 1. Install LibRealsense

  Reference: https://www.jetsonhacks.com/2019/12/22/install-realsense-camera-in-5-minutes-jetson-nano/

  ```bash
  $ git clone https://github.com/JetsonHacksNano/installLibrealsense
  $ cd installLibrealsense
  # Build from source
  # Chnage LIBREALSENSE_VERSION=v2.31.0 to LIBREALSENSE_VERSION=v2.38.1 and
  # Change NVCC_PATH=/usr/local/cuda-10.0/bin/nvcc to NVCC_PATH=/usr/local/cuda-10.2/bin/nvcc in buildLibrealsense.sh
  $ ./buildLibrealsense.sh
  ```

  ##### 2. Install Realsense_ROS

  Reference: https://github.com/intel/ros2_intel_realsense/tree/refactor 

  ```bash
  mkdir -p ~/ros2_ws/src 
  cd ~/ros2_ws/src 
  git clone https://github.com/intel/ros2_intel_realsense.git 
  cd ros2_intel_realsense 
  git checkout refactor 
  colcon build --symlink-install
  ```

#### <span id="jump3">Install ROS2 RTAB-Map</span>
  ##### 1. RTAB-Map library: 
  ```cmd
  cd ~ 
  git clone https://github.com/introlab/rtabmap.git rtabmap 
  cd rtabmap/build 
  cmake .. 
  make -j3
  sudo make install 
  ```
  ##### 2. Install message_filters and image_common
  These two packages are essensial for RTAB-Map ROS2 package
  ```cmd
  cd ~/ros2_ws/src
  git clone https://github.com/ros2/message_filters/tree/eloquent
  git clone https://github.com/ros-perception/image_common/tree/eloquent
  colcon build --symlink-install
  ```
  ##### 3. RTAB-Map ROS2 package
  ```cmd
  cd ~/ros2_ws 
  git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros 
  colcon build --symlink-install --packages-select rtabmap_ros
  ```
  
#### <span id="jump4">Change some parameters of launch file</span>
  **JUST COPY FILES**
  ```bash
  cp -r ~/smart-car/mapping/ros2/rs_demo* ~/ros2_ws/src/ros2_intel_realsense/realsense_examples/launch
  cp ~/smart-car/mapping/ros2/realsense_d400.launch.py ~/ros2_ws/src/rtabmap_ros/launch/ros2
  ```
#### <span id="jump5">How To Use</span>
  ##### 1. Open Terminal 1
  ```bash
  ros2 launch realsense_examples rs_demo_t265.launch.py
  ```
  ##### 2. Open Terminal 2
  ```bash
  ros2 launch realsense_examples rs_demo_rgbd.launch.py
  ```
  ##### 2. Open Terminal 3
  ```bash
  ros2 launch rtabmap_ros realsense_d400.launch.py
  ```
  

