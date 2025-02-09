# ENVIRONMENTAL CONTROL AND LIFE SUPPORT SYSTEM


## Air Revitalization System (ARS) Module - Version 2

### Project Overview
In this version, we introduce a comprehensive system for air quality management onboard the ISS, focusing on the collection, processing, and reduction of moisture, contaminants, and CO2. This system consists of three interconnected nodes, each handling specific tasks to optimize air processing using advanced desiccant and adsorbent technologies. The following sections explain the functionalities of each component in detail.

---

### System Components

### 1. Air Collection and Publishing Node

#### Purpose
This node simulates the collection of unprocessed air onboard the ISS, which contains moisture, CO2, and various contaminants. The air is continuously published on the `/unpure_air` topic, providing real-time data for downstream processing. This node also manages the entire system by triggering the reduction of CO2 using desiccant and adsorbent technologies.

#### Functionality
- **Publishes Custom Message**: A custom message `AirData.msg` encapsulates the following properties of the air:
  - **CO2 Content**: The amount of carbon dioxide in grams.
  - **Moisture**: Percentage of moisture in the air.
  - **Contaminants**: Percentage of contaminants in the air.
  - **Flow Rate**: Standard cubic feet per minute (SCFM).
  - **Temperature**: Cabin temperature in degrees Celsius.
  - **Pressure**: Cabin pressure in mmHg.
- **Topic**: Publishes this data on the `/unpure_air` topic.
- **Control Role**: Acts as a controller node by triggering two servers:
  - **Desiccant Bed Server**: Handles moisture and contaminant reduction.
  - **Adsorbent Bed Server**: Handles CO2 reduction.

---

### 2. Desiccant Bed Node

#### Purpose
This node acts as a server that reduces moisture and contaminants from the air received from the `/unpure_air` topic. It ensures that only the CO2 content is retained for further processing while publishing the processed data.

#### Functionality
- **Subscriber**: Subscribes to the `/unpure_air` topic and processes the incoming air data.
- **Reduction**:
  - **Moisture Content**: Reduced based on the desiccant bed's efficiency rate (e.g., 95% removal).
  - **Contaminants**: Reduced based on the removal efficiency (e.g., 90% removal).
- **Publisher**: Publishes the processed air data (retaining only CO2 content) on the `/removed_moisture` topic.
- **Server Role**: Operates as a server that can be triggered by the Air Collection Node for activation and deactivation.

---

### 3. Adsorbent Bed Node

#### Purpose
This node processes the reduced air from the desiccant node and performs CO2 reduction in two ways:
1. Sending a portion of the CO2 to space.
2. Retaining a portion of the CO2 for further reactions with hydrogen to generate methane and water.

#### Functionality
- **Subscriber**: Subscribes to the `/removed_moisture` topic to receive air data with reduced moisture and contaminants.
- **CO2 Processing**:
  - **Stream 1**: CO2 sent to space for removal.
  - **Stream 2**: CO2 retained and published on the `/processed_co2` topic for use in the water recovery system.
- **Server Role**: Operates as a server triggered by the Air Collection Node for activation and deactivation.

---

## System Flow

### Step 1: Air Collection
- The Air Collection Node continuously gathers air data and publishes it to the `/unpure_air` topic.
- Once the air data exceeds a defined threshold (e.g., tank capacity), the node triggers the desiccant and adsorbent servers for air processing.

### Step 2: Moisture and Contaminant Reduction
- The Desiccant Bed Node, upon activation, subscribes to the `/unpure_air` topic, processes the air to remove moisture and contaminants, and publishes the CO2 content on the `/removed_moisture` topic.

### Step 3: CO2 Reduction
- The Adsorbent Bed Node, upon activation, subscribes to the `/removed_moisture` topic, processes the CO2 content by splitting it into two streams, and publishes the retained CO2 on the `/processed_co2` topic for further utilization.

---

## Custom Message Definition

**File**: `msg/AirData.msg`
```plaintext
float64 co2_mass           # Mass of CO2 in grams
float64 moisture_content   # Moisture content as a percentage
float64 contaminants       # Contaminants as a percentage
float64 flow_rate          # Flow rate in SCFM
float64 temperature        # Temperature in Celsius
float64 pressure           # Pressure in mmHg
```

---

## Services

### Desiccant Server Service (`std_srvs/Trigger`)
- **Purpose**: Activated by the Air Collection Node to reduce moisture and contaminants.

### Adsorbent Server Service (`std_srvs/Trigger`)
- **Purpose**: Activated by the Air Collection Node to perform CO2 reduction.

---

## Benefits of Version 2

1. **Optimized Resource Management**
   - The system ensures efficient removal of unwanted air components while retaining valuable CO2 for further use.

2. **Modularity**
   - Each node is independently responsible for a specific aspect of air processing, making the system scalable and maintainable.

3. **Real-Time Processing**
   - Air data is processed in real-time with precise control over the activation and deactivation of servers.

---

## References

For detailed information on sensors and bed performance, refer to:
- **4BCO2.EDU.Performance_ICES-2021.pdf**

---

This documentation reflects the updated architecture and flow for **Version 2** of the ARS module simulation, ensuring engineers can understand, extend, and effectively use this system in their research and development.

