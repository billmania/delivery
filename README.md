# Delivery

## Install

This node requires manual installation of the OS package crcmod.

Get dependencies:
~~~
rosdep update
rosdep install -y --from-paths . --ignore-src
~~~

MAVROS depends on GeographicLib, and GeographicLib needs some datasets:
~~~
sudo ~/delivery_ws/src/delivery/scripts/install_geographiclib_datasets.sh
~~~

Build the workspace:
~~~
cd ~/delivery_ws
colcon build
~~~

## Dive

Terminal 1
~~~
socat pty,raw,nonblock,b115200,echo=0,link=/tmp/usbl0 TCP4:[192.168.2.2]:55443
~~~

Terminal 2
~~~
cd ~/delivery_ws
. install/setup.bash
ros2 launch delivery delivery.launch.py
~~~

Terminal 3
~~~
cd ~/delivery_ws
. install/setup.bash
ros2 launch delivery log.launch.py
~~~

Terminal 4
~~~
cd ~/Desktop
./QGC_sub_426.AppImage
~~~
