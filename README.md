# Robotont Webapp

## Prerequisites

### Install prerequisites from APT repositories
```
curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get update
sudo apt-get install -y build-essential cmake git libjson-c-dev libwebsockets-dev ros-noetic-web-video-server ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher nodejs
```

### Update node version

```
npm install -g n
n latest
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

The website will be available on port 3000 e.g. http://192.168.200.1:3000

## Redirecting port 80

To access the server on port 80, such that no port has to be specified in the addressbar, a redirect firewall rule can be set up with the following commands:

```
sudo iptables -t nat -A PREROUTING -i wlp58s0 -p tcp --dport 80 -j REDIRECT --to-port 3000
sudo apt install -y iptables-persistent
sudo iptables-save | sudo tee /etc/iptables/rules.v4
```
You might need to adapt the network interface name (`wlp58s0`) in the above commands to what is present in your robotont. Check with `ip addr` command.
