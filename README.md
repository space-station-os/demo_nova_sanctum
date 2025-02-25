### **Environmental Control and Life Support Systems (ECLSS)**
Environmental Control and Life Support Systems (ECLSS) are essential for sustaining human life in space by providing a controlled environment that includes air revitalization, water recovery, and waste management. This document serves as an overview of ECLSS and provides links to specific subsystems implemented as part of this project.

## **Overview of ECLSS Subsystems**
ECLSS consists of multiple interconnected subsystems to maintain habitable conditions for astronauts:

- **Air Revitalization System (ARS):** Handles **CO₂ removal, moisture control, and contaminant filtration** to maintain breathable air.
- **Oxygen Recovery System (ORS):** Converts **water into oxygen** through electrolysis and uses **hydrogen recovery** to form a closed-loop system.
- **Water Recovery and Balance Systems:** Processes crew urine, atmospheric condensation, and Sabatier-produced water for reuse.
- **Temperature and Humidity Control:** Regulates cabin conditions to ensure thermal comfort and moisture control.

## **Available Subsystem Implementations**
Below are the specific subsystems implemented as part of this project. Click on the links to access their respective documentation.

### **1. Air Revitalization System (ARS)**
The ARS is responsible for maintaining breathable air by removing CO₂, moisture, and contaminants from the cabin environment. The system consists of multiple ROS2 nodes working together to simulate air purification onboard the **International Space Station (ISS)**.

🔗 [Read the full ARS documentation](https://github.com/space-station-os/demo_nova_sanctum/blob/main/src/ars_systems/README.md)

### **2. Oxygen Recovery System (ORS)**
The ORS simulates the oxygen generation process used on the ISS. It leverages **electrolysis, Sabatier reaction, and deionization** to create a closed-loop system that efficiently recycles oxygen from water.

🔗 [Read the full ORS documentation](https://github.com/space-station-os/demo_nova_sanctum/blob/main/src/ors_systems/README.md)

### **2. Water Recovery And Purification Systems (WRPS)**
The WRS system purifies the waste accumulated from the crew and converts it into potable water that is fit for consumption. Some amount of water is also used to get oxygen by electrolysis

🔗 [Read the full WRS documentation](https://github.com/space-station-os/demo_nova_sanctum/blob/main/src/wrs_systems/README.md)


