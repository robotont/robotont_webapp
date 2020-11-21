<template>
    <div id='user' class="container">
        <div class="jumbotron">
            <h1>Robotont</h1>
        </div>

        <div class="row" style="max-height: 200px;">
            <div class="col-md-6">
                <h3>Connection status</h3>

                <p class="text-success" v-if="connected">Connected!</p>
                <p class="text-danger" v-if="!connected">Not connected!</p>

                <label>Websockect server address</label>
                <input type="text" v-model="ws_address" />
                <br />

                <button @click="disconnect" class="btn btn-danger" v-if="connected">Disconnect</button>
                <button @click="connect" class="btn btn-success" v-else>Connect</button>
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

        <div class="row">
            <div class="col-md-12 text-center">
                <h5>Commands</h5>
            </div>

            <div class="col-md-12 text-center">
                <button @click="forward" :disabled="loading || !connected" class="btn btn-primary">Go forward</button>
                <br><br>
            </div>

            <div class="col-md-4 text-center">
                <button @click="turnLeft" :disabled="loading || !connected" class="btn btn-primary">Left</button>
            </div>

            <div class="col-md-4 text-center">
                <button @click="stop" :disabled="loading || !connected" class="btn btn-danger">Stop</button>
                <br><br>
            </div>

            <div class="col-md-4 text-center">
                <button @click="turnRight" :disabled="loading || !connected" class="btn btn-primary">Right</button>         
            </div>

            <div class="col-md-12 text-center">
                <button @click="backward" :disabled="loading || !connected" class="btn btn-primary">Go backward</button>
                <br><br>
            </div>
        </div>
        

        <div class="col-md-12 text-center">
            <h5>Joystick</h5>
        </div>
        <br><br><br><br>

        <div id="joystick_zone" style="position: relative;"></div>
        <br><br><br><br>

        <hr>

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

export default {
    name: "User",

    //lehekülje staatuse salvestamine
    data: function () {
        return {
            connected: false,
            ros: null,
            ws_address: "192.168.1.165:9090",
            logs: [],
            loading: false,
            topic: null,
            message: null,
            topics: [],
            nodes:  [],
            manager: null
        }
    },

    methods: {
        connect: function () {
            this.loading = true;
            console.log("Connect to rosbridge server"); 

            this.ros = new ROSLIB.Ros({
                url: "ws://" + this.ws_address,
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
        forward: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 1, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
            });

            this.setTopic();
            this.topic.publish(this.message);
        },
        stop: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        backward: function () {
            this.message = new ROSLIB.Message({
                linear: { x: -1, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        turnLeft: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0.5 },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        turnRight: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: -0.5 },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        move: function (linear, angular) {
            this.message = new ROSLIB.Message({
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular },
            });
            this.setTopic();
            this.topic.publish(this.message);
        },

        joystick: function () {
            var options = {
                zone: document.getElementById("joystick_zone"),
                threshold: 0.1,
                position: { left: 50 + "%" },
                mode: "static",
                size: 150,
                color: "#000000",
            };
            var ref = this;
            ref.manager = nipplejs.create(options);

            var linear_speed = 0;
            var angular_speed = 0;
            var timer;
            ref.manager.on("start", function (event, nipple) {
                timer = setInterval(function () {
                    ref.move(linear_speed, angular_speed);
                }, 25);
            });

            ref.manager.on("move", function (event, nipple) {
                var max_linear = 2.0; // m/s
                var max_angular = 2.0; // rad/s
                var max_distance = 75.0; // pixels;
                linear_speed =
                    (Math.sin(nipple.angle.radian) *
                        max_linear *
                        nipple.distance) /
                    max_distance;
                angular_speed =
                    (-Math.cos(nipple.angle.radian) *
                        max_angular *
                        nipple.distance) /
                    max_distance;
            });

            ref.manager.on("end", function () {
                if (timer) {
                    clearInterval(timer);
                }
                ref.move(0, 0);
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
                this.manager.destroy();
            }
        },
    },
}
</script>