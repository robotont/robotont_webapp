# robotont-webapp

### Install rosbridge if it is not installed
```
sudo apt-get install ros-<rosdistro>-rosbridge-server
```

## Clone repository
```
cd robotont-webapp
```

### Run python http server
```
python3 -m http.server
```

### Run roscore
```
roscore
```

### Run rosbridge websocket
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Customize configuration
See [Configuration Reference](https://cli.vuejs.org/config/).
