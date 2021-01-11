<template>
    <b-container fluid="lg" class="container">
        <b-row>
            <b-col col lg="12">
                <b-button @click="rosRestart" class="mt-1" size="lg" block variant="success">Restart ROS</b-button><br>
                <b-button @click="rosStart" size="lg" block variant="success">Start ROS</b-button><br>
                <b-button @click="rosStop" size="lg" block variant="danger">Stop ROS</b-button><br>
                <b-button :disabled="showPointCloud" @click="pointCloud" size="lg" block variant="info">Show PointCloud</b-button><br>
            </b-col>
        </b-row>
        <div id="webViewer"></div>
    </b-container>
</template>

<script>
import axios from 'axios';
import ROSLIB from 'roslib'
import * as ROS3D from 'ros3d';
import { mapGetters } from 'vuex';
export default {

    name: "Admin",
    data: function() {
        return {
            showPointCloud: false
        }
    },
    computed: {
        ...mapGetters(["getRos", "getIP"])
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


        pointCloud: function() {
            this.showPointCloud = true;

            var viewer = new ROS3D.Viewer({
                divID : 'webViewer',
                width : 800,
                height : 600,
                antialias : true,
                background : '#111111'
            });


            // Setup a client to listen to TFs.
            var tfClient = new ROSLIB.TFClient({
                ros : this.getRos.ros,
                angularThres : 0.01,
                transThres : 0.01,
                rate : 10.0,
            });

            var tmpSub = new ROS3D.PointCloud2({
                ros:this.getRos.ros,
                tfClient: tfClient, 
                rootObject: viewer.scene,
                topic: '/rtabmap/cloud_map',
                material: {size: 0.01, color: 0xeeeeee }
            });
        },
    } 
}
</script>

<style scoped>

</style>