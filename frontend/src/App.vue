<template>
    <div id="app">
        <b-navbar toggleable="lg" type="dark" sticky variant="info">
            <b-navbar-brand>Robotont</b-navbar-brand>
            <b-navbar-toggle target="nav-collapse"></b-navbar-toggle>
            <b-collapse id="nav-collapse" is-nav>
                <b-navbar-nav>
                    <router-link tag="b-nav-item" to="/">User</router-link>
                    <router-link tag="b-nav-item" to="Admin">Advanced</router-link>
                </b-navbar-nav>
                <b-navbar-nav class="ml-auto">
                    <b-nav-text v-if="ifConnected.connected">Connected</b-nav-text>
                    <b-nav-text v-if="!ifConnected.connected">Not connected</b-nav-text>
                    <b-button @click="shutdown" class="ml-3" variant="danger" size="md">Shutdown</b-button>
                </b-navbar-nav>
            </b-collapse>
        </b-navbar>
        <router-view />
    </div>
</template>

<script>
import axios from 'axios';
import { mapGetters } from 'vuex';

export default {
    name: 'App',
    components: {
    },

    computed: {
        ...mapGetters(["getIP", "ifConnected"])
    },

    methods: {
        shutdown: function() {
            let ip = this.getIP.ip;
            let url = 'http://' + ip +':3000/shutdown'
            if (confirm("Are you sure you want to shutdown?")) {
                axios.post(url)
            }
        },

    }
}
</script>


