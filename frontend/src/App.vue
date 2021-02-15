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
import { mapGetters, mapActions } from 'vuex';

export default {
    name: 'App',
    components: {
    },

    computed: {
        ...mapGetters(["getIP", "ifConnected", "getRos"])
    },

    methods: {
        ...mapActions(['setRos', 'setConnect', 'setIP']),

        shutdown: function() {
            let ip = this.getIP.ip;
            let url = 'http://' + ip +':3000/shutdown'
            if (confirm("Are you sure you want to shutdown the robot?")) {
                axios.post(url)
            }
        },



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


