#navigation_step

##Instalation

1) Install youBot ROS packages

    $ sudo apt-get install ros-indigo-navigation
    $ sudo apt-get install ros-indigo-youbot-description ros-indigo-youbot-driver-ros-interface

2) Install v-rep (http://www.v-rep.eu/) into i.e. /opt/v-rep

3) Clone this repository into your catkin workspace

4) Clone vrep_youbot_plugin from https://github.com/Ram2301/vrep_youbot_plugin.git

5) Copy vrep_common and vrep_plugin ros packages from v-rep directory to your catkin workspace

    $ cp -r /opt/v-rep/programming/ros_packages/vrep_plugin ~/catkin_ws/src/
    $ cp -r /opt/v-rep/programming/ros_packages/vrep_common ~/catkin_ws/src/

6) Compile

    $ catkin_make

7) Copy the compiled libraries libv_repExtyouBot.so, libv_repExtRos.so and libv_repExtyouBot.so into your /opt/v-rep directory

##Usage

1) Start ros

    $ roscore

2) Start v-rep

    $ cd /opt/v-rep && ./vrep.sh

3) Open a scene in v-rep and run a BNT.launch file

--------------------------------

    $ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    $ sudo apt-get update
    $ sudo apt-get install gcc-6 g++-6
    $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
