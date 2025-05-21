<template>
  <div :style="backgroundStyle">
    <h1>💧 ISS Water Recovery System</h1>

    <StatusHUD
      :status="mainStatus"
      :mode="systemMode"
      :temperature="temperature"
      :pressure="pressure"
      :stlStatus="stlStatus"
    />

    <div class="pipeline-layout">
      <Tank
        label="WHC"
        type="wrs"
        :water="crewUse.moisture"
        :contaminants="crewUse.contaminants"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="Waste Collection"
        type="wrs"
        :water="wasteStatus.level"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="UPA"
        type="wrs"
        :water="upaStatus.distillate"
        :contaminants="upaStatus.brine"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="Filtration Unit"
        type="wrs"
        :water="filterationStatus.filtered_output"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="Ionization Bed"
        type="wrs"
        :water="ionStatus.cleaned_output"
        :iodine="ionStatus.iodine"
        :contaminants="ionStatus.contaminants"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="Catalytic Chamber"
        type="wrs"
        :water="cleanStatus.pure_water"
        :capacity="100"
      />
      <Pipe />

      <Tank
        label="Product Water Tank"
        type="wrs"
        :water="finalWater.level"
        :iodine="finalWater.iodine"
        :contaminants="finalWater.contaminants"
        :capacity="100"
      />
    </div>

    <div class="controls">
      <button @click="fetchSystemStatus">🔍 Get Water System Status</button>
    </div>
  </div>
</template>

<script>
/* global ROSLIB */
import Tank from "../components/Tank.vue";
import Pipe from "../components/Pipe.vue";
import StatusHUD from "../components/StatusHUD.vue";

export default {
  name: "WaterSystemsPage",
  components: { Tank, Pipe, StatusHUD },
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
      imgUrl: "/assets/iss_bg.jpg",
      mainStatus: "Nominal",
      systemMode: "Recycle",
      temperature: 22.0,
      pressure: 14.7,
      stlStatus: {
        WHC: "PASS",
        UPA: "PASS",
        FILTER: "PASS",
        IONIZATION: "PASS",
        CATALYTIC: "PASS",
      },
    };
  },
  computed: {
    backgroundStyle() {
      return {
        backgroundImage: `url(${this.imgUrl})`,
        backgroundSize: "cover",
        backgroundPosition: "center",
        backgroundAttachment: "fixed",
        minHeight: "100vh",
        paddingTop: "20px",
      };
    },
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // ✅ Live Topic: Product Water Tank
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

    // ✅ Live Topic: Ionization Bed
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
  },
  methods: {
    async callService(serviceName, type) {
      return new Promise((resolve, reject) => {
        const service = new ROSLIB.Service({
          ros: this.ros,
          name: serviceName,
          serviceType: type,
        });
        service.callService(
          new ROSLIB.ServiceRequest(),
          (res) => resolve(res),
          (err) => reject(err)
        );
      });
    },
    async fetchSystemStatus() {
      try {
        const crew = await this.callService(
          "/wpa/dispense_water",
          "demo_nova_sanctum/srv/Water"
        );
        const waste = await this.callService(
          "/waste_collector/get_parameters",
          "demo_nova_sanctum/srv/Water"
        );
        const upa = await this.callService(
          "/upa/process_urine",
          "demo_nova_sanctum/srv/Upa"
        );
        const filt = await this.callService(
          "/wpa/filtered_water",
          "demo_nova_sanctum/srv/Filteration"
        );
        const cat = await this.callService(
          "/wpa/catalytic_reactor",
          "demo_nova_sanctum/srv/CleanWater"
        );

        console.log("[CREW]", crew);
        console.log("[WASTE]", waste);
        console.log("[UPA]", upa);
        console.log("[FILTER]", filt);
        console.log("[CAT]", cat);

        this.crewUse = {
          moisture: crew?.moisture ?? 0,
          contaminants: crew?.contaminants ?? 0,
        };
        this.wasteStatus = { level: waste?.volume ?? 0 };
        this.upaStatus = {
          distillate: upa?.distillate ?? 0,
          brine: upa?.brine ?? 0,
        };
        this.filterationStatus = {
          filtered_output: filt?.filtered_output ?? 0,
        };
        this.cleanStatus = {
          pure_water: cat?.pure_water ?? 0,
        };
      } catch (err) {
        console.error("❌ Service call failed:", err);
      }
    },
  },
};
</script>

<style scoped>
h1 {
  text-align: center;
  color: #ffffff;
  text-shadow: 1px 1px 4px #000;
  font-size: 2.2rem;
  margin-bottom: 20px;
}

.pipeline-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 30px;
  flex-wrap: wrap;
  margin-top: 30px;
}

.controls {
  display: flex;
  justify-content: center;
  margin-top: 40px;
}

button {
  padding: 10px 20px;
  font-size: 1rem;
  background-color: #0abde3;
  border: none;
  border-radius: 6px;
  color: white;
  cursor: pointer;
  box-shadow: 0 2px 4px #00000040;
  transition: 0.3s;
}
button:hover {
  background-color: #0984e3;
}
</style>
