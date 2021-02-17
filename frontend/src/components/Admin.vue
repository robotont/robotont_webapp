<template>
    <b-container fluid="lg" class="container">

        <!-- Terminal -->
        <b-row>
            <b-col col lg="12">
                <h3 class="text-center">Terminal</h3>
            </b-col>
        </b-row>
        <b-row>
            <b-col col lg="12">
                <iframe v-if="isTerminalOpen" :src="terminalUrl" width="100%" height="400" frameborder="0"></iframe>
                <b-button v-if="!isTerminalOpen" @click="toggleTerminal" size="md" :disabled="!ifConnected.connected" class="btn btn-info">Show terminal</b-button>
                <b-button v-if="isTerminalOpen" @click="toggleTerminal" size="md" :disabled="!ifConnected.connected" class="btn btn-info">Hide terminal</b-button>
            </b-col>
        </b-row>
        <hr>

        <!-- Topics and nodes -->
        <b-row>
            <b-col col lg="6">
                <h3>Topics: </h3>
                <div class="lists">
                    <p v-for="topic in topics[0]" v-bind:key="topic">
                        {{ topic }}
                    </p>
                </div>
                <b-button @click="getTopics" :disabled="!ifConnected.connected" class="btn btn-info">Refresh</b-button>
            </b-col>
            <b-col col lg="6">
                <h3>Nodes: </h3>
                <div class="lists">
                    <p v-for="node in nodes[0]" v-bind:key="node">
                        {{ node }}
                    </p>
                </div>
                <b-button @click="getNodes" :disabled="!ifConnected.connected" class="btn btn-info">Refresh</b-button>
            </b-col>
        </b-row>
        <hr>

        <!-- Ros service buttons -->
        <b-row>
            <b-col col lg="12">
                <h3 class="text-center">ROS service control</h3>
                <div class="text-center">
                    <b-button @click="rosRestart" :disabled="!ifConnected.connected" class="mr-2" size="lg" variant="success">Restart ROS service</b-button>
                    <b-button @click="rosStart" :disabled="!ifConnected.connected" class="mr-2" size="lg" variant="success">Start ROS service</b-button>
                    <b-button @click="rosStop" :disabled="!ifConnected.connected" class="mr-2" size="lg" variant="danger">Stop ROS service</b-button>
                </div>
            </b-col>
        </b-row>
        <hr>      
    </b-container>
</template>

<script>
import axios from 'axios';
import ROSLIB from 'roslib';
import { mapGetters } from 'vuex';
export default {

    name: "Admin",
    data: function() {
        return {
            isTerminalOpen: false,
            topics: [],
            nodes: [],
            terminalUrl: "http://localhost:5000"
        }
    },
    computed: {
        ...mapGetters(["getRos", "getIP", "ifConnected"])
    },
    methods: {
        rosRestart: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosRestart';
            this.$confirm("Are you sure you want to continue?", "Warning", "warning").then(() => {
                axios.post(url);
            });
        },
        rosStart: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStart';
            this.$confirm("Are you sure you want to continue?", "Warning", "warning").then(() => {
                axios.post(url);
            });
        },
        rosStop: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStop';
            this.$confirm("Are you sure you want to continue?", "Warning", "warning").then(() => {
                axios.post(url);
            });
        },

        toggleTerminal: function() {
            let url = 'http://' + this.getIP.ip + ':5000';
            this.terminalUrl = url;

            if (!this.isTerminalOpen) {
                this.isTerminalOpen = true;
            }
            else {
                this.isTerminalOpen = false;
            }
        },

        getTopics: function() {
            var topicsClient = new ROSLIB.Service({
            ros: this.getRos.ros,
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
            ros: this.getRos.ros,
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
    mounted: function() {
        if (this.ifConnected.connected) {
            this.getTopics();
            this.getNodes();
            this.toggleTerminal();
        }
    }
}
</script>

<style scoped>
    .lists {
        overflow: auto;
        max-height: 400px;
    }

</style>