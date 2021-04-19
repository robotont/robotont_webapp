<template>
    <div>
        <b-container fluid="lg">
            <br>
            <!-- Joystick controls -->
            <b-row class="text-center">
                <b-col col lg="12">
                    <h5>Joystick Controls</h5>
                </b-col>
            </b-row>

            <br><br><br><br>

            <b-row>
                <b-col col lg="6" id="joystick_zone1"></b-col>
                <b-col col lg="6" id="joystick_zone2"></b-col>
            </b-row>

            <br><br><br><br>
            <hr>

            <!-- Camera feed and DepthCloud -->
            <b-row>
                <b-col class="text-center" col lg="6" fluid>
                    <h5 class="text-center">Camera feed</h5><br>
                    <b-img :src="video_src" fluid></b-img><br>
                </b-col>
                <b-col class="text-center" col lg="6">
                    <h5 class="text-center">Depthcloud</h5><br>
                    <div id="webViewer" fluid></div>
                </b-col>
            </b-row>

        </b-container>
    </div>
</template>

<script>
import ROSLIB from 'roslib';
import nipplejs from 'nipplejs';
import { mapGetters, mapActions } from 'vuex';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import * as ROS3D from 'ros3d';

export default {
    name: "User",

    //lehekülje staatuse salvestamine
    data: function () {
        return {
            connected: false,
            video_src: "http://localhost:4000/stream?topic=/camera/color/image_raw&type=ros_compressed",
            topic: null,
            message: null,
            joystick_manager1: null,
            joystick_manager2: null,
        }
    },

    computed: {
        ...mapGetters(["getRos", "getIP", "ifConnected"])
    },

    methods: {
        ...mapActions(['setRos', 'setConnect', 'setIP']),

        setTopic: function () {
            this.topic = new ROSLIB.Topic({
                ros: this.getRos.ros,
                name: "/cmd_vel",
                messageType: "geometry_msgs/Twist",
            });
        },

        stop: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },

        move: function (linear_x, linear_y, angular) {
            this.message = new ROSLIB.Message({
                linear: { x: linear_x, y: linear_y, z: 0 },
                angular: { x: 0, y: 0, z: angular },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },

        joystick: function () {
            var options1 = {
                zone: document.getElementById("joystick_zone1"),
                threshold: 0.1,
                position: { left: 40 + "%" },
                mode: "static",
                size: 150,
                color: "#000000",
            };
            var options2 = {
                zone: document.getElementById("joystick_zone2"),
                threshold: 0.1,
                position: { right: 40 + "%" },
                mode: "static",
                size: 150,
                color: "#000000",
                lockX: true,
            };
            var ref = this;
            ref.joystick_manager1 = nipplejs.create(options1);
            ref.joystick_manager2 = nipplejs.create(options2);

            var linear_speed_x = 0;
            var linear_speed_y = 0
            var angular_speed = 0;
            var timer1;
            var timer2;

            ref.joystick_manager1.on("start", function (event, nipple) {
                timer1 = setInterval(function () {
                    ref.move(linear_speed_x, linear_speed_y, angular_speed);
                }, 25);
            });

            ref.joystick_manager1.on("move", function (event, nipple) {
                var max_linear = 2.0; // m/s
                var max_angular = 2.0; // rad/s
                var max_distance = 75.0; // pixels;
                linear_speed_x =
                    (Math.sin(nipple.angle.radian) *
                        max_linear *
                        nipple.distance) /
                    max_distance;

                linear_speed_y =
                    (-Math.cos(nipple.angle.radian) *
                        max_angular *
                        nipple.distance) /
                    max_distance;

                angular_speed = 0;
            });

            ref.joystick_manager1.on("end", function () {
                if (timer1) {
                    clearInterval(timer1);
                }
                ref.move(0, 0, angular_speed);
            });


            ref.joystick_manager2.on("start", function (event, nipple) {
                timer2 = setInterval(function () {
                    ref.move(linear_speed_x, linear_speed_y, angular_speed);
                }, 25);
            });

            ref.joystick_manager2.on("move", function (event, nipple) {
                var max_linear = 2.0; // m/s
                var max_angular = 2.0; // rad/s
                var max_distance = 75.0; // pixels;
                linear_speed_x = 0;
                linear_speed_y = 0;

                angular_speed =
                    (-Math.cos(nipple.angle.radian) *
                        max_angular *
                        nipple.distance) /
                    max_distance;
            });

            ref.joystick_manager2.on("end", function () {
                if (timer2) {
                    clearInterval(timer2);
                }
                ref.move(linear_speed_x, linear_speed_y, 0);
            });
        },

        cameraFeed: function() {
            this.video_src = "http://" + this.getIP.ip + ":4000/stream?topic=/camera/color/image_raw&type=ros_compressed"
        },

        depthCloud: function() {
            var url = 'http://' + this.getIP.ip + ':4000/stream?topic=/depthcloud_encoded&type=mjpeg'
            var path = 'http://' + this.getIP.ip + ':3000/files/'
            var width = window.innerWidth / 3.4;

            var viewer = new ROS3D.Viewer({
                divID : 'webViewer',
                width : window.innerWidth / 3.3,
                height : width,
                antialias : true,
                background : '#111111'
            });
            viewer.addObject(new ROS3D.Grid())

            if (window.innerWidth < 400) {
                viewer.resize(window.innerWidth - 50, 410);
            }

            // Setup a client to listen to TFs.
            var tfClient = new ROSLIB.TFClient({
                ros: this.getRos.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
            });

            /*const mesh = new ROS3D.MeshResource({
                resource: 'robotont_description/meshes/body.stl',
                path: path,
                warnings: true
            });*/
            //console.log("Loaded mesh: ", mesh);
            
            var loader = new ColladaLoader();
            // Setup the URDF client
            var urdfClient = new ROS3D.UrdfClient({
                ros: this.getRos.ros,
                tfClient: tfClient,
                path: path,
                rootObject: viewer.scene,
                loader : loader
            });

            // Setup Camera DepthCloud stream
            var depthCloud = new ROS3D.DepthCloud({
                url : url,
                streamType: "mjpeg",
                f : 525.0
            });
            depthCloud.startStream();

            // Create Camera scene node
            var cameraNode = new ROS3D.SceneNode({
                frameID: "camera_depth_optical_frame",
                tfClient : tfClient,
                object : depthCloud
            });
            viewer.scene.add(cameraNode);
        },    
    },

    mounted: function() {
        if (this.ifConnected.connected) {
            this.joystick();
            this.cameraFeed();
            this.depthCloud();
        }
    },

    watch: {
        '$store.state.connected': function () {
            if (this.ifConnected.connected) {
                this.joystick();
                this.cameraFeed();
                this.depthCloud();
            }
            else {
                this.joystick_manager1.destroy();
                this.joystick_manager2.destroy();
            }
        },
    },
}
</script>