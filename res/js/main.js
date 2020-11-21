var app = new Vue({
    el: "#app",

    //lehekülje staatuse salvestamine
    data: {
        connected: false,
        ros: null,
        ws_address: "192.168.1.165:9090",
        logs: [],
        loading: false,
        topic: null,
        message: null,
        topics: [],
        nodes:  [],
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
            manager = nipplejs.create(options);

            linear_speed = 0;
            angular_speed = 0;

            manager.on("start", function (event, nipple) {
                timer = setInterval(function () {
                    ref.move(linear_speed, angular_speed);
                }, 25);
            });

            manager.on("move", function (event, nipple) {
                max_linear = 2.0; // m/s
                max_angular = 2.0; // rad/s
                max_distance = 75.0; // pixels;
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

            manager.on("end", function () {
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
                manager.destroy();
            }
        },
    },
});
