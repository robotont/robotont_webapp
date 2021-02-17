<template>
    <div id="app">
    <b-navbar toggleable="true" type="dark" sticky variant="dark">
        <b-navbar-toggle target="nav-collapse"></b-navbar-toggle>
        <b-navbar-brand class="ml-2">Robotont</b-navbar-brand>
        <b-navbar-nav class="ml-auto mr-2">
            <b-nav-text>Connection:</b-nav-text>
        </b-navbar-nav>
        <b-navbar-nav class="mr-2">
            <div v-if="!ifConnected.connected" class="dot" style="background-color: red"></div>
            <div v-if="ifConnected.connected" class="dot" style="background-color: green"></div>
        </b-navbar-nav>
        <b-collapse id="nav-collapse" is-nav>
            <b-navbar-nav>
                <router-link tag="b-nav-item" to="/">User</router-link>
                <router-link tag="b-nav-item" to="Admin">Advanced</router-link>
            </b-navbar-nav>
            <b-button @click="shutdown" variant="danger" size="md">Shutdown</b-button>
        </b-collapse>
    </b-navbar>
    <router-view />
    </div>
</template>

<script>
import axios from 'axios';
import ROSLIB from 'roslib';
import { mapGetters, mapActions } from 'vuex';

export default {
    name: 'App',
    
    data: function() {
        return {
            connected: false,
            ros: null,
            ws_address: null,
            loading: false
        }
    },

    computed: {
        ...mapGetters(["getIP", "ifConnected", "getRos"])
    },

    methods: {
        ...mapActions(['setRos', 'setConnect', 'setIP']),

        shutdown: function() {
            let ip = this.getIP.ip;
            let url = 'http://' + ip +':3000/shutdown'
            this.$confirm("Are you sure you want to shutdown the robot?", "Warning", "warning").then(() => {
                axios.post(url);
            });
        },

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
            });
            
            this.ros.on("error", (error) => {
                console.log("Error connecting to websocekt server: ", error);
                this.connect();
            });
            
            this.ros.on("close", () => {
                this.connected = false;
                this.setConnect({connected: false})
                this.loading = false;
                console.log("Connection to websocket server closed");
            });
        },
    },
    created: function() {
        this.ws_address = window.location.host.slice(0, -5) + ":9090";
        this.connect();
    },

    watch: {
        connected: function() {
            if (!this.connected) {
                this.$alert("Connection to websocket server is lost. Retrying to connect.", "Error", "error").then(() => {
                    this.connect();
                });
            }
            else {
                this.$alert("Connection to websocket server was created.", "Success", "success").then(() => {
                });
            }
        }
    }
}
</script>

<style scoped>
    .dot {
        height: 10px;
        width: 10px;
        background-color: #bbb;
        border-radius: 50%;
    }
</style>


