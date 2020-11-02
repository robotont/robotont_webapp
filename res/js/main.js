var app = new Vue({
    el: '#app',

    //lehekülje staatuse salvestamine
    data: {
        connected: false,
        ros: null,
        //ws://
        ws_address: 'ws://localhost:9090',
        logs: [],
        loading: false,
        topic: null,
        message: null,
    },

    //ROS-i funktsioonid

    methods: {
        connect: function() {
            this.loading = true
            console.log('Connect to rosbridge server')
            this.logs.unshift('Connect to rosbridge server')

            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            this.ros.on('connection', () => {
                this.connected = true
                this.loading = false
                console.log('Connected!!!')
                this.logs.unshift('Connected!')
            })
            this.ros.on('error', (error) => {
                console.log('Error connecting to websocekt server: ', error)
                this.logs.unshift('Error connecting to websocekt server')
            })
            this.ros.on('close', () => {
                this.connected = false
                this.loading = false
                console.log('Connection to websocket server closed')
                this.logs.unshift('Connection to websocket server closed')
            })
        },
        disconnect: function() {
            this.ros.close()
        },
        setTopic: function() {
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/robotont/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
        },
        forward: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })

            this.setTopic()
            this.topic.publish(this.message)
        }, 
        stop: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        backward: function() {
            this.message = new ROSLIB.Message({
                linear: { x: -1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnLeft: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnRight: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        }, 
    },
})