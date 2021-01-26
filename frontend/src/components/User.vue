<template>
    <div>
        <b-container fluid="lg">
            <!-- Connection input and log -->
            <b-row>
                <b-col col lg="6">
                    <h3>Connection status</h3>
                    <p class="text-success" v-if="connected">Connected!</p>
                    <p class="text-danger" v-if="!connected">Not connected!</p>

                    <label>Websockect server address</label>
                    <br>
                    <b-input type="text" v-model="ws_address" />

                    <br>

                    <b-button @click="disconnect" class="mt-1" block variant="danger" v-if="connected">Disconnect</b-button>
                    <b-button @click="connect" class="mt-1" block variant="success" v-if="!connected">Connect</b-button>
                </b-col>
                <b-col col lg="6">
                    <h3>Log messages: </h3>
                    <div style="max-height: 170px; overflow: auto;">
                        <p v-for="log in logs" :key="log">
                            {{ log }}
                        </p>
                    </div>
                </b-col>
            </b-row>
            <hr>
            

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
            <br><br><br><br>
            <hr>

            <!-- Camera feed -->
            <b-row>
                <b-col col lg="6">
                    <h5 class="text-center">Camera feed</h5>
                </b-col>
            </b-row>
            <b-row>
                <b-col col lg="6">
                    <b-img v-if="showCamera" :src="video_src" fluid></b-img> <br>
                    <b-button @click="cameraFeed" :disabled="!ifConnected.connected" class="mt-1" v-if="!showCamera" block variant="info">Show camera feed</b-button>
                    <b-button @click="cameraFeed" :disabled="!ifConnected.connected" class="mt-1" v-if="showCamera" block variant="info">Hide camera feed</b-button>
                </b-col>
            </b-row>
            <br>
        </b-container>
    </div>
</template>

<script>
import ROSLIB from 'roslib';
import nipplejs from 'nipplejs';
import { mapGetters, mapActions } from 'vuex';

export default {
    name: "User",

    //lehekülje staatuse salvestamine
    data: function () {
        return {
            connected: false,
            ros: null,
            ws_address: "localhost:9090",
            video_src: "http://localhost:4000/stream?topic=/camera/color/image_raw&type=ros_compressed",
            logs: [],
            loading: false,
            topic: null,
            message: null,
            joystick_manager1: null,
            joystick_manager2: null,
            showCamera: false
        }
    },

    computed: {
        ...mapGetters(["getRos", "getIP", "ifConnected"])
    },

    methods: {
        ...mapActions(['setRos', 'setConnect', 'setIP']),
        connect: function () {
            this.loading = true;
            console.log("Connect to rosbridge server"); 

            this.ros = new ROSLIB.Ros({
                url: "ws://" + this.ws_address
            });
            
            
            this.ros.on("connection", () => {
                this.connected = true;
                this.setRos({ros: this.ros})
                this.setConnect({connected: true});
                this.setIP({ip: this.ws_address.slice(0, -5)})
                this.loading = false;
                console.log("Connected");
                this.logs.unshift(new Date().toLocaleTimeString("it-IT") + " Connected!");
            });
            
            this.ros.on("error", (error) => {
                console.log("Error connecting to websocekt server: ", error);
                this.logs.unshift(new Date().toLocaleTimeString("it-IT") + " Error connecting to websocekt server");
            });
            
            this.ros.on("close", () => {
                this.connected = false;
                this.setConnect({connected: false})
                this.loading = false;
                console.log("Connection to websocket server closed");
                this.logs.unshift(new Date().toLocaleTimeString("it-IT") + " Connection to websocket server closed");
            });
        },
        disconnect: function () {
            this.connected = false;
            this.setConnect({connected: false})
            this.loading = false;
            this.ros.close();
        },

        setTopic: function () {
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: "/robotont/cmd_vel",
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
            this.video_src = "http://" + this.ws_address.slice(0, -5) + ":4000/stream?topic=/camera/color/image_raw&type=ros_compressed"
            if (this.showCamera) {
                this.showCamera = false
            }
            else {
                this.showCamera = true;
            }

        }    
    },
    mounted: function() {
        this.connected = this.ifConnected.connected;
        this.ros = this.getRos.ros
    },

    watch: {
        connected: function () {
            if (this.connected) {
                this.joystick();
            }
            else {
                this.joystick_manager1.destroy();
                this.joystick_manager2.destroy();
            }
        },
    },
}
</script>