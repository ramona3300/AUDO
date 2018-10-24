#!/bin/bash
cd $HOME
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get -y install ocl-icd-opencl-dev
sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libsdl-image1.2-dev
#sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg0-dev
sudo apt-get -y install libglfw3-dev
sudo apt-get -y install beignet-dev
sudo apt-get -y install beignet
sudo apt-get -y install opencl-headers
sudo apt-get -y install libva-dev libjpeg-dev
sudo apt-get -y install i965-va-driver
sudo apt-get -y install libopenni2-dev
mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
cd $HOME/catkin_ws/src/
git clone https://github.com/tud-pses/iai_kinect2.git
cd iai_kinect2
source /opt/ros/melodic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
rosdep install -r -y --from-paths .
cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
