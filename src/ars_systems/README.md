# ENVIRONMENTAL CONTROL AND LIFE SUPPORT SYSTEM


# README - ROS2 ARS System

## Overview
The **ARS System** (Air Revitalization System) is a ROS2-based implementation for air collection, moisture removal, and CO₂ adsorption in a closed-loop environment. This system simulates an advanced life support system similar to those used in space habitats, ensuring air quality is maintained through multiple processing stages.

## System Components
The system consists of three main nodes:
1. **Air Collector Node** - Gathers and conditions the cabin air.
2. **Desiccant Server Node** - Removes moisture and contaminants.
3. **Adsorbent Bed Node** - Adsorbs and vents CO₂ to maintain breathable air.

Each node operates independently but communicates through **ROS2 services, publishers, and subscribers** to form a complete air revitalization process.

---

## Installation & Setup
### 1. Prerequisites
Ensure you have **ROS2 Humble or later** installed on your system.
```bash
source /opt/ros/humble/setup.bash
```

### 2. Clone the Repository
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/space-station-os/demo_nova_sanctum.git
cd ..
colcon build --symlink-install --packages-select demo_nova_sanctum
```

### 3. Source the Workspace
```bash
source install/setup.bash
```

### 4. Launch the System
Run each node in separate terminals:
```bash
ros2 run demo_nova_sanctum collector
ros2 run demo_nova_sanctum desiccant
ros2 run demo_nova_sanctum adsorbent
```
Or launch all nodes together:
```bash
ros2 launch demo_nova_sanctum ars_system_v3.launch.py
```

---

## Node Descriptions & Communication
### **Air Collector Node** (`air_collector`)
- Collects and monitors air in the environment.
- Simulates CO₂ production, moisture, and contaminants accumulation.
- Interfaces with the **Desiccant Server** for moisture removal and **Adsorbent Bed** for CO₂ processing.

**Communication Overview:**
- **Publishes:**
  - `/temperature` - Reports current temperature
  - `/pipe_pressure` - Reports system pressure
  - `/cdra_status` - Status of air processing
- **Subscribes:**
  - `/crew_co2_service` - Requests processing of accumulated CO₂ and contaminants
- **Service Client:**
  - `/crew_co2_service` - Calls Desiccant Server for air processing

**Parameters:**
```yaml
crew_onboard: 4
cabin_pressure: 14.7
temperature_cutoff: 450.0
max_crew_limit: 6
power_consumption: 1.0
tank_capacity: 1000.0
system_name: "demo_nova_sanctum"
mode_of_operation: "standby"
co2_threshold: 500.0
moisture_threshold: 70.0
contaminants_threshold: 30.0
temp_kp: 0.1
temp_ki: 0.01
temp_kd: 0.005
press_kp: 0.1
press_ki: 0.01
press_kd: 0.005
```

---

### **Desiccant Server Node** (`desiccant_server`)
- Removes moisture and contaminants from incoming air.
- Ensures proper humidity levels before sending air to **Adsorbent Bed**.
- Uses PID controllers for temperature and pressure regulation.

**Communication Overview:**
- **Publishes:**
  - `/cdra_status` - Publishes system health and status
- **Subscribes:**
  - `/temperature` - Reads temperature sensor data
  - `/pipe_pressure` - Reads pressure sensor data
- **Service Server:**
  - `/crew_co2_service` - Handles air processing requests from the Air Collector Node
- **Service Client:**
  - `/adsorbent_server` - Requests CO₂ adsorption from the Adsorbent Bed

**Parameters:**
```yaml
moisture_removal_rate: 0.95
contaminant_removal_rate: 0.90
emergency_threshold: 5.0
target_temperature: 70.0
target_pressure: 150000.0
humidification_rate: 1.5
```

---

### **Adsorbent Bed Node** (`adsorbent_bed`)
- Adsorbs CO₂ from the air, preventing dangerous buildup.
- Uses PID control to regulate the adsorption process.
- Handles CO₂ venting operations.

**Communication Overview:**
- **Publishes:**
  - `/co2_vent` - Reports CO₂ venting status
  - `/cdra_status` - System health and operational data
- **Subscribes:**
  - `/adsorbent_server` - Processes air from the desiccant bed
- **Service Server:**
  - `/adsorbent_server` - Handles air adsorption requests from the Desiccant Server
- **Service Client:**
  - `/desiccant_bed2` - Sends processed air back for humidification

**Parameters:**
```yaml
co2_removal_efficiency: 0.95
co2_to_space_ratio: 0.40
desired_temperature: 420.0
temperature_tolerance: 30.0
kp: 0.6
kd: 0.15
```

---

## System Flow
1. **Air Collection:** The **Air Collector** gathers air, monitors CO₂ and humidity levels, and determines when processing is needed.
2. **Moisture Removal:** The **Desiccant Server** removes excess moisture and contaminants, preparing air for adsorption.
3. **CO₂ Adsorption:** The **Adsorbent Bed** captures CO₂, preventing buildup in the habitat.
4. **CO₂ Venting:** Periodically, the **Adsorbent Bed** releases stored CO₂ into space.


## Future Improvements
- Add real-time monitoring via a ROS2 visualization tool.
- Implement a machine learning model for predictive maintenance.

---


