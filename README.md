# Robotont Webapp

## Install node.js

```
curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
```
```
sudo apt-get install -y nodejs
```

## Install prerequisites
```
sudo apt-get update
sudo apt-get install ros-melodic-web-video-server ros-melodic-rosbridge-server ros-melodic-tf2-web-republisher ros-melodic-depthcloud-encoder build-essential cmake libjson-c-dev libwebsockets-dev
```

## Project setup

Go to the scripts folder and run build.sh
```
./build.sh
```

## Launch the web application
```
roslaunch robotont_webapp webapp.launch
```

