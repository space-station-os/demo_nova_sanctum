<template>
  <div>
    <div class="top-bar">
      <select v-model="selectedSystem" @change="navigate">
        <option value="/ars">Air Revitalization (ARS)</option>
        <option value="/ogs">Oxygen Generation (OGS)</option>
        <option value="/water">Water Recovery System (WRS)</option>
        <!-- Future: <option value="/wrs">Water Recovery (WRS)</option> -->
      </select>
    </div>

    <router-view />
  </div>
</template>

<script>
export default {
  data() {
    return {
      selectedSystem: "/ars",
    };
  },
  methods: {
    navigate() {
      this.$router.push(this.selectedSystem);
    },
  },
  mounted() {
    this.selectedSystem = this.$route.path;
  },
};
</script>

<style scoped>
.top-bar {
  position: fixed;
  top: 30px;
  left: 10px;
  z-index: 999;
  background-color: rgba(0, 0, 0, 0.8);
  padding: 8px 12px;
  border-radius: 6px;
  box-shadow: 0 0 5px black;
}

select {
  font-size: 0.9rem;
  padding: 6px 10px;
  background-color: #fff;
  color: #000;
  border-radius: 4px;
  border: none;
  outline: none;
}
</style>
