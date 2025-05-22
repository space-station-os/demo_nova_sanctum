<template>
  <div class="wrs-wrapper">
    <h1>💧 Water Recovery System</h1>

    <!-- STATUS CARDS -->
    <div class="status-grid">
      <StatusCard title="System Health" :value="mainStatus" delta="+5%" />
      <StatusCard
        title="Recovery Rate"
        :value="`${recoveryRate}%`"
        delta="+2%"
      />
      <StatusCard
        title="Tank Level"
        :value="`${finalWater.level}%`"
        delta="-10%"
      />
    </div>

    <!-- TANK LEVEL BAR -->
    <div class="level-bar">
      <label>Water Tank Level</label>
      <div class="bar-wrapper">
        <div class="bar-fill" :style="{ width: `${finalWater.level}%` }"></div>
        <span class="bar-label">{{ finalWater.level.toFixed(1) }}%</span>
      </div>
    </div>

    <!-- PIPELINE VISUALIZATION -->
    <div class="pipeline-layout">
      <TankCard title="WHC">
        <Tank
          type="wrs"
          :water="crewUse.moisture"
          :contaminants="crewUse.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Waste Collection">
        <Tank type="wrs" :water="wasteStatus.level" :capacity="100" />
      </TankCard>
      <Pipe />
      <TankCard title="UPA">
        <Tank
          type="wrs"
          :water="upaStatus.distillate"
          :contaminants="upaStatus.brine"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Filtration Unit">
        <Tank
          type="wrs"
          :water="filterationStatus.filtered_output"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Ionization Bed">
        <Tank
          type="wrs"
          :water="ionStatus.cleaned_output"
          :iodine="ionStatus.iodine"
          :contaminants="ionStatus.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Catalytic Chamber">
        <Tank type="wrs" :water="cleanStatus.pure_water" :capacity="100" />
      </TankCard>
      <Pipe />
      <TankCard title="Product Water Tank">
        <Tank
          type="wrs"
          :water="finalWater.level"
          :iodine="finalWater.iodine"
          :contaminants="finalWater.contaminants"
          :capacity="100"
        />
      </TankCard>
    </div>
  </div>
</template>

<script>
/* global ROSLIB */
import Tank from "../components/Tank.vue";
import Pipe from "../components/Pipe.vue";
import TankCard from "../components/TankCard.vue";
import StatusCard from "../components/StatusCard.vue";

export default {
  name: "WaterSystemsPage",
  components: { Tank, Pipe, TankCard, StatusCard },
  data() {
    return {
      ros: null,
      crewUse: { moisture: 0, contaminants: 0 },
      wasteStatus: { level: 0 },
      upaStatus: { distillate: 0, brine: 0 },
      filterationStatus: { filtered_output: 0 },
      ionStatus: { cleaned_output: 0, iodine: 0, contaminants: 0 },
      cleanStatus: { pure_water: 0 },
      finalWater: { level: 0, iodine: 0, contaminants: 0 },
      mainStatus: "Operational",
      recoveryRate: 95,
    };
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // Final Tank
    const tankStatusSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/wpa/tank_status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    tankStatusSub.subscribe((msg) => {
      this.finalWater = {
        level: msg.water,
        iodine: msg.iodine_level,
        contaminants: msg.contaminants,
      };
    });

    // Ionization Bed
    const ionStatusSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/ionization/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    ionStatusSub.subscribe((msg) => {
      this.ionStatus = {
        cleaned_output: msg.water,
        iodine: msg.iodine_level,
        contaminants: msg.contaminants,
      };
    });

    // WHC
    const whcSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/whc/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    whcSub.subscribe((msg) => {
      this.crewUse = {
        moisture: msg.water,
        contaminants: msg.contaminants,
      };
    });

    // Waste Collection
    const wasteSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/waste_collection/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    wasteSub.subscribe((msg) => {
      this.wasteStatus = {
        level: msg.water,
      };
    });

    // UPA
    const upaSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/upa/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    upaSub.subscribe((msg) => {
      this.upaStatus = {
        distillate: msg.water,
        brine: msg.contaminants,
      };
    });

    // Filtration Unit
    const filterSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/filtration/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    filterSub.subscribe((msg) => {
      this.filterationStatus = {
        filtered_output: msg.water,
      };
    });

    // Catalytic Chamber
    const catSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/catalytic/status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    catSub.subscribe((msg) => {
      this.cleanStatus = {
        pure_water: msg.water,
      };
    });
  },
};
</script>

<style scoped>
.wrs-wrapper {
  background-color: #0d1117;
  color: white;
  padding: 40px;
}

h1 {
  text-align: center;
  font-size: 2.3rem;
  margin-bottom: 40px;
}

/* Status Cards Grid */
.status-grid {
  display: flex;
  gap: 20px;
  justify-content: center;
  margin-bottom: 30px;
}

/* Tank Level Bar */
.level-bar {
  max-width: 600px;
  margin: 20px auto 50px;
}

.bar-wrapper {
  background: #333;
  height: 14px;
  border-radius: 6px;
  overflow: hidden;
  position: relative;
  margin-top: 4px;
}

.bar-fill {
  height: 100%;
  background-color: #00aaff;
  transition: width 0.5s ease-in-out;
}

.bar-label {
  position: absolute;
  right: 10px;
  top: -22px;
  font-size: 0.9rem;
  color: #ccc;
}

/* Tank Layout */
.pipeline-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 30px;
  flex-wrap: wrap;
  margin-top: 30px;
}
</style>
