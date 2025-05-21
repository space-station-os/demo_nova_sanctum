<template>
  <div class="schematic-tank">
    <div class="tank-header">{{ label }}</div>

    <div class="tank-body">
      <!-- OGS -->
      <template v-if="type === 'ogs'">
        <div class="oxygen-layer" :style="{ height: oxygenHeight + '%' }"></div>
        <div
          class="hydrogen-layer"
          :style="{ height: hydrogenHeight + '%' }"
        ></div>
      </template>

      <!-- WRS -->
      <template v-else-if="type === 'wrs'">
        <div class="water-layer" :style="{ height: waterHeight + '%' }"></div>
        <div class="iodine-layer" :style="{ height: iodineHeight + '%' }"></div>
        <div
          class="contaminants-layer"
          :style="{ height: contaminantsHeight + '%' }"
        ></div>
      </template>

      <!-- ARS -->
      <template v-else>
        <div
          class="contaminants-layer"
          :style="{ height: contaminantsHeight + '%' }"
        ></div>
        <div
          class="moisture-layer"
          :style="{ height: moistureHeight + '%' }"
        ></div>
        <div class="co2-layer" :style="{ height: co2Height + '%' }"></div>
      </template>
    </div>

    <div class="tank-readout">
      <template v-if="type === 'ogs'">
        <p><strong>O₂:</strong> {{ formatValue(oxygen) }} mol</p>
        <p><strong>H₂:</strong> {{ formatValue(hydrogen) }} mol</p>
      </template>

      <template v-else-if="type === 'wrs'">
        <p><strong>H₂O:</strong> {{ formatValue(water) }} mL</p>
        <p><strong>Iodine:</strong> {{ formatValue(iodine) }}%</p>
        <p><strong>Contaminants:</strong> {{ formatValue(contaminants) }}%</p>
      </template>

      <template v-else>
        <p><strong>CO₂:</strong> {{ formatValue(co2) }} g</p>
        <p><strong>Moisture:</strong> {{ formatValue(moisture) }}%</p>
        <p><strong>Contaminants:</strong> {{ formatValue(contaminants) }}%</p>
      </template>
    </div>
  </div>
</template>

<script>
export default {
  name: "Ars-Tank",
  props: {
    label: String,
    type: { type: String, default: "ars" }, // 'ars', 'ogs', 'wrs'
    co2: { type: Number, default: 0 },
    moisture: { type: Number, default: 0 },
    contaminants: { type: Number, default: 0 },
    oxygen: { type: Number, default: 0 },
    hydrogen: { type: Number, default: 0 },
    water: { type: Number, default: 0 },
    iodine: { type: Number, default: 0 },
    capacity: { type: Number, required: true },
  },
  computed: {
    totalContent() {
      if (this.type === "ogs") return this.oxygen + this.hydrogen;
      if (this.type === "wrs")
        return this.water + this.iodine + this.contaminants;
      return this.co2 + this.moisture + this.contaminants;
    },
    co2Height() {
      return this.toPercentage(this.co2);
    },
    moistureHeight() {
      return this.toPercentage(this.moisture);
    },
    contaminantsHeight() {
      return this.toPercentage(this.contaminants);
    },
    oxygenHeight() {
      return this.toPercentage(this.oxygen);
    },
    hydrogenHeight() {
      return this.toPercentage(this.hydrogen);
    },
    waterHeight() {
      return this.toPercentage(this.water);
    },
    iodineHeight() {
      return this.toPercentage(this.iodine);
    },
  },
  methods: {
    toPercentage(value) {
      if (this.totalContent === 0 || isNaN(value)) return 0;
      return Math.min((value / this.totalContent) * 100, 100);
    },
    formatValue(val) {
      return val !== undefined ? val.toFixed(2) : "0.00";
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

/* ARS layers */
.co2-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #ffd700;
  opacity: 0.85;
  z-index: 3;
  transition: height 0.5s ease-in-out;
}

.moisture-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #00bfff;
  opacity: 0.6;
  z-index: 2;
  transition: height 0.5s ease-in-out;
}

.contaminants-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #32cd32;
  opacity: 0.5;
  z-index: 1;
  transition: height 0.5s ease-in-out;
}

/* OGS layers */
.oxygen-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #ff6347;
  opacity: 0.75;
  z-index: 2;
  transition: height 0.5s ease-in-out;
}

.hydrogen-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #87cefa;
  opacity: 0.75;
  z-index: 1;
  transition: height 0.5s ease-in-out;
}

/* WRS layers */
.water-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #3399ff;
  opacity: 0.8;
  z-index: 3;
  transition: height 0.5s ease-in-out;
}

.iodine-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: #ff69b4;
  opacity: 0.6;
  z-index: 2;
  transition: height 0.5s ease-in-out;
}

.tank-readout {
  padding: 8px;
  font-size: 0.8rem;
  color: #000;
  background-color: rgba(255, 255, 255, 0.85);
}
</style>
