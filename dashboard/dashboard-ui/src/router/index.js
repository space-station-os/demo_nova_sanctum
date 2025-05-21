import { createRouter, createWebHistory } from "vue-router";
import Ars from "../pages/Ars.vue";
import Oxygen from "../pages/Oxygen.vue";
import WaterSystemsPage from "../pages/Waterrecovery.vue";
const routes = [
  { path: "/", redirect: "/ars" },
  { path: "/ars", name: "ARS", component: Ars },
  { path: "/ogs", name: "OGS", component: Oxygen },
  { path: "/water", component: WaterSystemsPage },
];

export default createRouter({
  history: createWebHistory(),
  routes,
});
