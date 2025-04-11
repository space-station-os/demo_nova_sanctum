<template>
  <div :style="backgroundStyle">
    <h1>🛰️ ISS Air Revitalization Dashboard</h1>

    <StatusHUD
      :status="status"
      :mode="mode"
      :temperature="temperature"
      :pressure="pressure"
    />

    <div class="dashboard-layout">
      <Tank
        label="Air Collector"
        :co2="co2_mass"
        :moisture="moisture_content"
        :contaminants="contaminants"
        :capacity="1000"
      />

      <Pipe />

      <Tank
        label="Desiccant Bed"
        :co2="desiccant_co2"
        :moisture="desiccant_moisture"
        :contaminants="desiccant_contaminants"
        :capacity="1000"
      />

      <Pipe />

      <Tank
        label="Adsorbent Bed"
        :co2="adsorbent_co2"
        :moisture="adsorbent_moisture"
        :contaminants="adsorbent_contaminants"
        :capacity="1000"
      />
    </div>
  </div>
</template>

<script>
import Tank from "./components/Tank.vue";
import Pipe from "./components/Pipe.vue";
import StatusHUD from "./components/StatusHUD.vue";

export default {
  components: { Tank, Pipe, StatusHUD },
  data() {
    return {
      co2_mass: 0,
      moisture_content: 0,
      contaminants: 0,
      desiccant_co2: 0,
      desiccant_moisture: 0,
      desiccant_contaminants: 0,
      adsorbent_co2: 0,
      adsorbent_moisture: 0,
      adsorbent_contaminants: 0,
      imgUrl: "/assets/iss_bg.jpg",
      status: "Nominal",
      mode: "Idle",
      temperature: 72.0,
      pressure: 14.5,
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
    const ros = new window.ROSLIB.Ros({ url: "ws://localhost:9090" });

    const subscribeTopic = (topic, cb) => {
      const sub = new window.ROSLIB.Topic({
        ros,
        name: topic,
        messageType: "demo_nova_sanctum/msg/AirData",
      });
      sub.subscribe(cb);
    };

    subscribeTopic("/collector_air_quality", (msg) => {
      this.co2_mass = msg.co2_mass;
      this.moisture_content = msg.moisture_content;
      this.contaminants = msg.contaminants;
    });

    subscribeTopic("/desiccant_air_quality", (msg) => {
      this.desiccant_co2 = msg.co2_mass;
      this.desiccant_moisture = msg.moisture_content;
      this.desiccant_contaminants = msg.contaminants;
    });

    subscribeTopic("/adsorbent_air_quality", (msg) => {
      this.adsorbent_co2 = msg.co2_mass;
      this.adsorbent_moisture = msg.moisture_content;
      this.adsorbent_contaminants = msg.contaminants;
    });
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
  font-weight: 600;
}

.dashboard-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 50px;
  margin-top: 20px;
  flex-wrap: wrap;
}
</style>
