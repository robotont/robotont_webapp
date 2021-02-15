# roslibnodejs

Install if not installed

```
sudo apt-get install ros-melodic-web-video-server
sudo apt-get install ros-melodic-rosbridge-server
sudo apt-get install ros-melodic-tf2-web-republisher
sudo apt-get install ros-melodic-depthcloud-encoder 
```

For the web applications full functionality run these
```
rosrun web_video_server web_video_server _port:=4000
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
rosrun tf2_web_republisher tf2_web_republisher
rosrun depthcloud_encoder depthcloud_encoder_node _depth:=/camera/depth/image_rect_raw _rgb:=/camera/color/image_raw
```

## Project setup

### Run Frontend

Go to the frontend folder
```
cd frontend
```

Run this only on the first launch
```
npm install
```

To run the frontend
```
npm run serve
```


### Run Backend

Go to the backend folder
```
cd backend
```

Run this only on the first launch
```
npm install
```

To run the backend
```
npm start
```

### Run ttyd for web terminal

Go to ttyd build folder
```
cd ttyd/build/
```
Run the terminal app
```
ttyd -p 5000 bash
````

### Customize configuration
See [Configuration Reference](https://cli.vuejs.org/config/).
