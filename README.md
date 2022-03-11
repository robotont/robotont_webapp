# Robotont Webapp

## Prerequisites

### Install prerequisites from APT repositories
```
curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get update
sudo apt-get install -y build-essential cmake git libjson-c-dev libwebsockets-dev ros-noetic-web-video-server ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher nodejs
```

### Build & install ttyd (for sharing terminal on the web)

```
git clone https://github.com/tsl0922/ttyd.git
cd ttyd && mkdir build && cd build
cmake ..
make && sudo make install
```
For further information please see the [ttyd readme](https://github.com/tsl0922/ttyd#install-on-linux).

### Depthcloud encoder

Add depthcloud\_encoder package under the src folder of your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/RobotWebTools/depthcloud_encoder.git
```


## Build the package with nodejs backend and frontend

In your catkin workspace run:
```
cd ~/catkin_ws
catkin build
```

## Launch the web application
```
roslaunch robotont_webapp webapp.launch
```

