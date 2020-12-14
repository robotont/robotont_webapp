<template>
    <div id='user' class="container">
        <div class="jumbotron">
            <h1>Robotont</h1>
            <br>
            <router-view />
            <router-link tag="a" to="/">User</router-link>
            <br>
            <router-view />
            <router-link tag="a" to="Admin">Admin</router-link>
        </div>

        <div class="row" style="max-height: 200px;">
            <div class="col-md-6">
                <h3>Connection status</h3>

                <p class="text-success" v-if="connected">Connected!</p>
                <p class="text-danger" v-if="!connected">Not connected!</p>

                <label>Websockect server address</label>
                <input type="text" v-model="ws_address" />
                <br />

                <button @click="disconnect" class="btn btn-danger btn mr-5" v-if="connected">Disconnect</button>
                <button @click="connect" class="btn btn-success btn mr-5" v-else>Connect</button>
                <button @click="shutdown" class="btn btn-danger btn mr-5">Shutdown</button>
            </div>

            <div class="col-md-6">
                <h3>Log messages: </h3>
                <div style="max-height: 170px; overflow: auto;">
                    <p v-for="log in logs" v-bind:key="log">
                        {{ log }}
                    </p>
                </div>
            </div>
        </div>

        <hr>        

        <div class="col-md-12 text-center">
            <h5>Joystick</h5>
        </div>
        <br><br><br><br>
        <div class="row">
            <div class="col-md-4" id="joystick_zone1" style="position: relative;"></div>
            <div class="col-md-6" id="joystick_zone2" style="position: relative;"></div>
        </div>
        <div class="col-md-12 text-center">
            <button @click="stop" :disabled="loading || !connected" class="btn btn-danger">Stop</button>
        </div>
        <br><br><br><br>

        <hr>

        <div class="col-md-12">
            <img src="http://localhost:4000/stream?topic=/usb_cam/image_raw&type=ros_compressed">
        </div>

        <div class="row">
            <div class="col-md-6">
                <h3>Topics: </h3>
                <div style="max-height: 400px; max-width: 500px; overflow: auto;">
                    <p v-for="t in topics[0]" v-bind:key="t">
                        {{ t }}
                    </p>
                </div>
                <button @click="getTopics" :disabled="loading || !connected" class="btn btn-info">Show topics</button>
            </div>
            
            <div class="col-md-4">
                <h3>Nodes: </h3>
                <div style="max-height: 400px; max-width: 500px; overflow: auto;">
                    <p v-for="n in nodes[0]" v-bind:key="n">
                        {{ n }}
                    </p>
                </div>
                <button @click="getNodes" :disabled="loading || !connected" class="btn btn-info">Show nodes</button>
            </div>
        </div>
        <br><br><br><br>
    </div>
</template>

<script>
import ROSLIB from 'roslib';
import nipplejs from 'nipplejs';
import 'bootstrap/dist/css/bootstrap.min.css';
import axios from 'axios';

export default {
    name: "User",

    components: {
    },

    //lehekülje staatuse salvestamine
    data: function () {
        return {
            connected: false,
            ros: null,
            ws_address: "localhost:9090",
            logs: [],
            loading: false,
            topic: null,
            message: null,
            topics: [],
            nodes:  [],
            joystick_manager1: null,
            joystick_manager2: null
        }
    },

    methods: {
        shutdown: function() {
            if (confirm("Are you sure you want to shutdown?")) {
                axios.get('http://localhost:3000/shutdown')
            }
        },

        connect: function () {
            this.loading = true;
            console.log("Connect to rosbridge server"); 

            this.ros = new ROSLIB.Ros({
                url: "ws://" + this.ws_address
            });
            this.ros.on("connection", () => {
                this.connected = true;
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
                this.loading = false;
                console.log("Connection to websocket server closed");
                this.logs.unshift(new Date().toLocaleTimeString("it-IT") + " Connection to websocket server closed");
            });
        },
        disconnect: function () {
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
                position: { left: 50 + "%" },
                mode: "static",
                size: 150,
                color: "#000000",
            };
            var options2 = {
                zone: document.getElementById("joystick_zone2"),
                threshold: 0.1,
                position: { left: 90 + "%" },
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

        getTopics: function() {
            var topicsClient = new ROSLIB.Service({
            ros: this.ros,
            name: '/rosapi/topics',
            serviceType: 'rosapi/Topics'
            });

            var request = new ROSLIB.ServiceRequest();

            var topicsList = [];
            topicsClient.callService(request, function(result) {
                topicsList.push(result.topics);
            });
            this.topics = topicsList;
        },
        getNodes: function() {
            var nodesClient = new ROSLIB.Service({
            ros: this.ros,
            name: '/rosapi/nodes',
            serviceType: 'rosapi/Nodes'
            });

            var request = new ROSLIB.ServiceRequest();
       
            var nodesList = [];
            nodesClient.callService(request, function(result) {
                nodesList.push(result.nodes);
            });
            this.nodes = nodesList;
        },
        
    },
    watch: {
        connected: function () {
            if (this.connected) {
                this.joystick();
                this.getNodes();
                this.getTopics();
            }
            else {
                this.joystick_manager1.destroy();
                this.joystick_manager2.destroy();
            }
        },
    },
}
</script>