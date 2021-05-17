<template>
    <b-container fluid="lg" class="container">

        <!-- Terminal -->
        <b-row>
            <b-col col lg="12">
                <h3 class="text-center">{{$store.state.currentLang.terminal}}</h3>
            </b-col>
        </b-row>
        <b-row>
            <b-col col lg="12">
                <iframe :src="terminalUrl" width="100%" height="400" frameborder="0"></iframe>
            </b-col>
        </b-row>
        <hr>

        <!-- Topics and nodes -->
        <b-row>
            <b-col col lg="6">
                <h3>{{$store.state.currentLang.topics}}: </h3>
                <div class="lists">
                    <p v-for="topic in topics[0]" v-bind:key="topic">
                        {{ topic }}
                    </p>
                </div>
                <b-button @click="getTopics" :disabled="!ifConnected.connected" class="btn btn-info">{{$store.state.currentLang.refresh}}</b-button>
            </b-col>
            <b-col col lg="6">
                <h3>{{$store.state.currentLang.nodes}}: </h3>
                <div class="lists">
                    <p v-for="node in nodes[0]" v-bind:key="node">
                        {{ node }}
                    </p>
                </div>
                <b-button @click="getNodes" :disabled="!ifConnected.connected" class="btn btn-info">{{$store.state.currentLang.refresh}}</b-button>
            </b-col>
        </b-row>
        <hr>

        <!-- Ros service buttons -->
        <b-row>
            <b-col col lg="12">
                <h3 class="text-center">{{$store.state.currentLang.service}}</h3>
                <div class="text-center">
                    <b-button @click="rosRestart" :disabled="!ifConnected.connected" class="mr-2 mt-2 mb-2" size="lg" variant="success">{{$store.state.currentLang.restart}}</b-button>
                    <b-button @click="rosStart" :disabled="!ifConnected.connected" class="mr-2 mt-2 mb-2" size="lg" variant="success">{{$store.state.currentLang.start}}</b-button>
                    <b-button @click="rosStop" :disabled="!ifConnected.connected" class="mr-2 mt-2 mb-2" size="lg" variant="danger">{{$store.state.currentLang.stop}}</b-button>
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
            this.$confirm(this.$store.state.currentLang.confirmation, this.$store.state.currentLang.warning, "warning").then(() => {
                axios.post(url);
            });
        },
        rosStart: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStart';
            this.$confirm(this.$store.state.currentLang.confirmation, this.$store.state.currentLang.warning, "warning").then(() => {
                axios.post(url);
            });
        },
        rosStop: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStop';
            this.$confirm(this.$store.state.currentLang.confirmation, this.$store.state.currentLang.warning, "warning").then(() => {
                axios.post(url);
            });
        },

        showTerminal: function() {
            let url = 'http://' + this.getIP.ip + ':5000';
            this.terminalUrl = url;
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
        this.getTopics();
        this.getNodes();
        this.showTerminal();
    }
}
</script>

<style scoped>
    .lists {
        overflow: auto;
        max-height: 400px;
    }

</style>