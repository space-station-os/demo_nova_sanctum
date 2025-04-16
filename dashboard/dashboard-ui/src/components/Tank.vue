<template>
  <div class="schematic-tank">
    <div class="tank-header">{{ label }}</div>
    <div class="tank-body">
      <div
        class="contaminants-layer"
        :style="{ height: contaminantsHeight + '%' }"
      ></div>
      <div
        class="moisture-layer"
        :style="{ height: moistureHeight + '%' }"
      ></div>
      <div class="co2-layer" :style="{ height: co2Height + '%' }"></div>
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
    totalContent() {
      return this.co2 + this.moisture + this.contaminants;
    },
    co2Height() {
      return Math.min((this.co2 / this.totalContent) * 100, 100);
    },
    moistureHeight() {
      return Math.min((this.moisture / this.totalContent) * 100, 100);
    },
    contaminantsHeight() {
      return Math.min((this.contaminants / this.totalContent) * 100, 100);
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

.co2-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #ffd700; /* Yellow */
  opacity: 0.85;
  transition: height 0.5s ease-in-out;
  z-index: 3;
}

.moisture-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #00bfff; /* Blue */
  opacity: 0.6;
  transition: height 0.5s ease-in-out;
  z-index: 2;
}

.contaminants-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #32cd32; /* Green */
  opacity: 0.5;
  transition: height 0.5s ease-in-out;
  z-index: 1;
}

.tank-readout {
  padding: 8px;
  font-size: 0.8rem;
  background-color: rgba(255, 255, 255, 0.85);
}
</style>
