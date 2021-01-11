import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

const state = {
    ros: null,
    connected: false,
    ip_address: "localhost"
}

const getters = {
    getRos: (state) => state.ros,
    ifConnected: (state) => state.connected,
    getIP: state => state.ip_address
}

const mutations = {
    SET_ROS: (state, ros) => (state.ros = ros),
    SET_CONNECT: (state, connected) => (state.connected = connected),
    SET_IP: (state, ip) => (state.ip_address = ip)
}

const actions = {
    async setRos({ commit }, ros) {
        commit("SET_ROS", ros)
    },
    async setConnect({ commit }, connected) {
        commit('SET_CONNECT', connected)
    },
    async setIP({ commit }, ip) {
        commit('SET_IP', ip)
    }
}
export default new Vuex.Store({
    state,
    getters,
    mutations,
    actions
})