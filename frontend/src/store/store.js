import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

const state = {
    ros: null,
    connected: false,
    ip_address: "localhost",
    lang: 'est',
    locale: {
        "en": {
            "camera": "Camera feed",
            "depthcloud": "Depthcloud",
            "connection": "Connection",
            "user": "User",
            "advanced": "Advanced",
            "shutdown": "Shutdown",
            "show": "Show",
            "hide": "Hide",
            "joysticks": "Joysticks",
            "terminal": "Terminal",
            "refresh": "Refresh",
            "topics": "Topics",
            "nodes": "Nodes",
            "service": "Robotont service",
            "restart": "Restart",
            "start": "Start",
            "stop": "Stop",
            "warning": "Warning",
            "error": "Error",
            "success": "Success",
            "shutdownMsg": "Are you sure you want to shutdown the robot?",
            "connectionError": "Connection to websocket server is lost. Retrying to connect.",
            "connectionSuccess": "Connection to websocket server was created.",
            "confirmation": "Are you sure you want to continue?",
            "cameraButtonShow": "Show camera feed",
            "depthcloudButtonShow": "Show depthcloud",
            "cameraButtonHide": "Hide camera feed",
            "depthcloudButtonHide": "Hide depthcloud"
        },
        "est": {
            "camera": "Kaamerapilt",
            "depthcloud": "Sügavuspilv",
            "connection": "Ühendus",
            "user": "Kasutaja",
            "advanced": "Administratiivne",
            "shutdown": "Lülita välja",
            "show": "Näita",
            "hide": "Peida",
            "joysticks": "Juhtkangid",
            "terminal": "Käsurida",
            "refresh": "Värskenda",
            "topics": "Rubriigid",
            "nodes": "Sõlmed",
            "service": "Robotondi teenus",
            "restart": "Taaskäivita",
            "start": "Käivita",
            "stop": "Peata",
            "warning": "Hoiatus",
            "error": "Viga",
            "success": "Õnnestumine",
            "shutdownMsg": "Olete kindel, et tahate robotit välja lülitada?",
            "connectionError": "Ühendus websocket serveriga on katkenud. Ühenduse taasloomine.",
            "connectionSuccess": "Ühendus websocket serveriga on loodud.",
            "confirmation": "Olete kindel, et soovite jätkata?",
            "cameraButtonShow": "Näita kaamerapilti",
            "depthcloudButtonShow": "Näita sügavuspilve",
            "cameraButtonHide": "Peida kaamerapilt",
            "depthcloudButtonHide": "Peida sügavuspilv"
        }
    },
    currentLang: null
}

const getters = {
    getRos: (state) => state.ros,
    ifConnected: (state) => state.connected,
    getIP: (state) => state.ip_address,
    getLocale: (state) => state.locale,
    getLang: (state) => state.lang,
    getCurrentLang: (state) => state.currentLang
}

const mutations = {
    SET_ROS: (state, ros) => (state.ros = ros),
    SET_CONNECT: (state, connected) => (state.connected = connected),
    SET_IP: (state, ip) => (state.ip_address = ip),
    SET_LANG: (state, lang) => (state.lang = lang),
    SET_CURRENT: (state, currentLang) => (state.currentLang = currentLang)
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
    },
    async setLang({ commit }, lang) {
        commit("SET_LANG", lang)
    },
    async setCurrentLang({ commit }, currentLang) {
        commit("SET_CURRENT", currentLang)
    },
}
export default new Vuex.Store({
    state,
    getters,
    mutations,
    actions
})