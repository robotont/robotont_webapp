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
        <br>
        <div class="col-md-5">
            <button @click="rosRestart" class="btn btn-primary btn mr-3">Restart ROS</button>
            <button @click="rosStart" class="btn btn-primary btn mr-3">Start ROS</button>
            <button @click="rosStop" class="btn btn-primary btn mr-3">Stop ROS</button>
        </div>   
    </div>
</template>

<script>
import axios from 'axios';
export default {

    name: "Admin",
    methods: {
        rosRestart: function() {
            axios.get('http://localhost:3000/rosRestart')
        },
        rosStart: function() {
            axios.get('http://localhost:3000/rosStart')
        },
        rosStop: function() {
            axios.get('http://localhost:3000/rosStop')
        },
    } 
    
}
</script>

<style scoped>

</style>