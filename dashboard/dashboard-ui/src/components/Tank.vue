<template>
  <div class="schematic-tank">
    <div class="tank-header">{{ label }}</div>
    <div class="tank-body">
      <div
        class="tank-fill"
        :style="{ height: co2Height + '%', backgroundColor: '#00bcd4' }"
      ></div>
      <div
        class="moisture-layer"
        :style="{ height: moistureHeight + '%' }"
      ></div>
    </div>
    <div class="tank-readout">
      <p><strong>CO₂:</strong> {{ co2.toFixed(2) }} g</p>
      <p><strong>Moisture:</strong> {{ moisture.toFixed(2) }}%</p>
      <p><strong>Contaminants:</strong> {{ contaminants.toFixed(2) }}%</p>
    </div>
  </div>
</template>

<script>
export default {
  name: "Ars-Tank",
  props: ["label", "co2", "moisture", "contaminants", "capacity"],
  computed: {
    co2Height() {
      const total = this.co2 + this.moisture + this.contaminants;
      return Math.min((this.co2 / total) * 100, 100);
    },
    moistureHeight() {
      const total = this.co2 + this.moisture + this.contaminants;
      return Math.min((this.moisture / total) * 100, 100);
    },
  },
};
</script>

<style scoped>
.schematic-tank {
  width: 120px;
  background: #eee;
  border: 4px solid black;
  border-radius: 4px;
  overflow: hidden;
  box-shadow: 0 0 6px rgba(0, 0, 0, 0.3);
  text-align: center;
}

.tank-header {
  background: black;
  color: white;
  font-weight: bold;
  padding: 4px;
  font-size: 0.85rem;
}

.tank-body {
  height: 240px;
  position: relative;
  background: #dcdcdc;
}

.tank-fill {
  position: absolute;
  bottom: 0;
  width: 100%;
  transition: height 0.5s ease-in-out;
  background-color: #00bcd4;
  opacity: 0.9;
}

.moisture-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background: rgba(173, 216, 230, 0.4);
  transition: height 0.5s ease-in-out;
}

.tank-readout {
  padding: 8px;
  font-size: 0.8rem;
  background-color: rgba(255, 255, 255, 0.85);
}
</style>
