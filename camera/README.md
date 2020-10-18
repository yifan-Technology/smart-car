# 从零开始安装yfcam
## 安装Jetson NX系统

## 安装ROS 1

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full

source /opt/ros/melodic/setup.bash

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install python-rosdep

sudo rosdep init
rosdep update```
```

## 安装带有ROS1 bridge 的 ROS 2 

```shell

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'


sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
# install some thing missed 
sudo apt-get install libasio-dev -y

mkdir -p ~/ros2_dashing/src
cd ~/ros2_dashing
wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos
vcs import src < ros2.repos

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

cd ~/ros2_dashing/
colcon build --symlink-install --packages-skip ros1_bridge
# gibe it a ros1 env
source /opt/ros/melodic/setup.bash
. install/local_setup.bash

colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

```
## 安装Zed SDK for Jetson 4.4
```shell
# download sdk in {https://www.stereolabs.com/developers/release/}
cd ~/downloda

chmod +x ZED_[your download file].run
./ZED_[your download file].run

# to quit the file use q
# install all options
```

## 安装Zed for ROS1 
```shell
source /opt/ros/melodic/setup.bash

cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

## 安装 ROS2 Navigation
```shell
sudo apt install ros-dashing-navigation2 -y
sudo apt install ros-dashing-nav2-bringup -y
sudo apt install ros-dashing-turtlebot3* -y
```

## 安装 Google Object Detection　API
```shell

sudo apt-get update
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran -y
sudo apt-get install python3-pip -y
sudo pip3 install -U pip testresources setuptools
sudo pip3 install -U numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11

sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow

sudo apt-get install protobuf-compiler python-pil python-lxml python-tk -y
pip3 install --user Cython
pip3 install --user contextlib2
pip3 install --user pillow
pip3 install --user lxml
pip3 install --user jupyter
pip3 install --user matplotlib
pip3 install --user tf_slim

cd ~
mkdir tf_ws
cd tf_ws
git clone https://github.com/tensorflow/models.git

git clone https://github.com/cocodataset/cocoapi.git
cd cocoapi/PythonAPI
make
cp -r pycocotools ~/tf_ws/models/research/

pip3 install --user pycocotools

cd ~/tf_ws/model/research
protoc object_detection/protos/*.proto --python_out=.

wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip
unzip protobuf.zip
./bin/protoc object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:'pwd':'pwd'/slim

python3 object_detection/builders/model_builder_test.py
```
若无报错，则上述运行安装无误

## 新建ROS2 工作空间
```shell
source  /opt/ros/dashing/setup.bash
source ~/ros2_dashing/install/local_setup.bash
cd ~
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b dashing-devel
cd ..
rosdep install -i --from-path src --rosdistro dashing -y
colcon build
```

## 结束语
至此，已完成所有安装，具体执行，详见教程cmd line run。

[2020 06 06 22:22:22]
