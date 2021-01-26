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
                <b-button @click="rosRestart" :disabled="!ifConnected.connected" class="mt-1" size="md" block variant="success">Restart ROS service</b-button><br>
                <b-button @click="rosStart" :disabled="!ifConnected.connected" size="md" block variant="success">Start ROS service</b-button><br>
                <b-button @click="rosStop" :disabled="!ifConnected.connected" size="md" block variant="danger">Stop ROS service</b-button><br>
            </b-col>
        </b-row>

        <!-- Ros3d -->
        <b-row>
            <b-col col lg="12">
                <div class="text-center" id="webViewer"></div>
                <b-button :disabled="showPointCloud" @click="depthCloud" size="lg" class="mt-1" block variant="info">Show DepthCloud</b-button><br>
            </b-col>
        </b-row>
        
    </b-container>
</template>

<script>
import axios from 'axios';
import ROSLIB from 'roslib';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import * as ROS3D from 'ros3d';
import { mapGetters } from 'vuex';
export default {

    name: "Admin",
    data: function() {
        return {
            isTerminalOpen: false,
            showPointCloud: false,
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
            let url = 'http://' + this.getIP.ip + ':3000/rosRestart'
            axios.post(url)
        },
        rosStart: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStart'
            axios.post(url)
        },
        rosStop: function() {
            let url = 'http://' + this.getIP.ip + ':3000/rosStop'
            axios.post(url)
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

        depthCloud: function() {
            let url = 'http://' + this.getIP.ip + ':4000/stream?topic=/depthcloud_encoded&type=mjpeg'
            this.showPointCloud = true;

            var viewer = new ROS3D.Viewer({
                divID : 'webViewer',
                width : 800,
                height : 600,
                antialias : true,
                background : '#111111'
            });
            viewer.addObject(new ROS3D.Grid())

            // Setup a client to listen to TFs.
            var tfClient = new ROSLIB.TFClient({
                ros: this.getRos.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
            });

            /*console.log("Loading mesh resource");
            const mesh = new ROS3D.MeshResource({
                resource: 'robotont_description/meshes/body.stl',
                path: 'http://localhost:8000/',
                warnings: true
            });
            console.log("Loaded mesh: ", mesh);*/
            var loader = new ColladaLoader();
            // Setup the URDF client
            var urdfClient = new ROS3D.UrdfClient({
                ros: this.getRos.ros,
                tfClient: tfClient,
                path: "http://0.0.0.0:8000/",
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
    }
}
</script>

<style scoped>
    .lists {
        overflow: auto;
        max-height: 400px;
    }

</style>