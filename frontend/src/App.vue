<template>
    <div id="app">
        <b-navbar toggleable="lg" type="dark" sticky variant="info">
            <b-navbar-brand>Robotont</b-navbar-brand>
            <b-navbar-toggle target="nav-collapse"></b-navbar-toggle>
            <b-collapse id="nav-collapse" is-nav>
                <b-navbar-nav>
                    <router-link tag="b-nav-item" to="/">User</router-link>
                    <router-link tag="b-nav-item" to="Admin">Admin</router-link>
                </b-navbar-nav>
                <b-button class="ml-auto" @click="shutdown" variant="danger" size="md">Shutdown</b-button>
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
        ...mapGetters(["getIP"])
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


