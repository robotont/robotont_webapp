import Vue from 'vue'
import Router from 'vue-router'

import Admin from '@/components/Admin.vue'
import User from '@/components/User.vue'

Vue.use(Router)

export default new Router({
    routes: [
        {
            path: '/',
            name: 'User',
            component: User
        },

        {
            path: '/admin',
            name: 'Admin',
            component: Admin
        },
    ]
})