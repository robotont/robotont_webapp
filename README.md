# Robotont Webapp

## Prerequisites

### node.js

```
curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
```
```
sudo apt-get install -y nodejs
```

### Prerequisites from APT repositories
```
sudo apt-get update
sudo apt-get install ros-noetic-web-video-server ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher build-essential cmake libjson-c-dev libwebsockets-dev
```

### Depthcloud encoder

Add depthcloud\_encoder package under the src folder of your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/RobotWebTools/depthcloud_encoder.git
```

### ttyd

Follow the Install instructions on the [ttyd readme](https://github.com/tsl0922/ttyd#install-on-linux).

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

