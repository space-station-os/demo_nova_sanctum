<template>
  <div class="hud">
    <div><strong>Status:</strong> {{ status }}</div>
    <div><strong>Mode:</strong> {{ mode }}</div>
    <div><strong>Temp:</strong> {{ temperature }} °C</div>
    <div><strong>Pressure:</strong> {{ pressure }} psi</div>
  </div>
</template>

<script>
export default {
  props: {
    status: String,
    mode: String,
    temperature: Number,
    pressure: Number,
  },
};
</script>

<style scoped>
.hud {
  display: flex;
  justify-content: center;
  gap: 30px;
  font-family: "Courier New", monospace;
  font-size: 1rem;
  background: rgba(0, 0, 0, 0.5);
  color: #00ffe0;
  padding: 10px 20px;
  border: 2px solid #00bcd4;
  border-radius: 8px;
  margin: auto;
  width: fit-content;
  box-shadow: 0 0 10px #00bcd4;
}
</style>
