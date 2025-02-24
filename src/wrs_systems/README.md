### **Water Recovery System**
#### **Overview**
The **Water Recovery System (WRS)** is a critical subsystem of the **Environmental Control and Life Support System (ECLSS)** onboard the **International Space Station (ISS)**. The goal of this system is to **recycle wastewater** into **potable drinking water** for astronauts.

This project **simulates** the **water recovery process** using **ROS 2 nodes** representing different filtration, oxidation, and purification components.

---

## **Water Recovery System in Detail**
The **Water Recovery System (WRS)** consists of the following **major components**:

### **1. Waste & Hygiene Compartment (WHC)**
- The **starting point** of the system.
- Collects **crew urine and hygiene wastewater**.
- Mixes urine with **pretreatment chemicals** to prevent microbial growth.
- Sends **processed waste** to the **Waste Collection Node**.

### **2. Waste Collection System**
- Receives **wastewater from WHC**.
- Accumulates water **until a threshold is reached**.
- Sends **collected urine** to the **Urine Processor Assembly (UPA)** for purification.

### **3. Urine Processor Assembly (UPA)**
- **Distills urine** into **usable water**.
- Uses a **centrifugal vacuum distillation process**.
- Sends **cleaned water** to the **Filtration Unit**.

### **4. Filtration Unit**
- Removes **dissolved solids, organics, and ammonia**.
- Uses **Multifiltration (MF) beds**.
- Passes **filtered water** to the **Catalytic Chamber**.

### **5. Catalytic Chamber**
- **Oxidizes volatile organic compounds (VOCs)**.
- **Kills bacteria and microbes** using **heat & oxidation**.
- Sends **sterilized water** to the **Ionization Bed**.

### **6. Ionization Bed**
- Removes **remaining contaminants**.
- Adds **iodine** as a **biocide** to prevent microbial growth.
- Sends **final purified water** to the **Clean Water Tank**.

### **7. Product Water Tank**
- Stores **100 liters of clean drinking water**.
- Supplies water for **drinking, food rehydration, and hygiene**.
- Sends data to the **Oxygen Generation System (OGS)** for electrolysis.

---

## **Node Communication and Topics**
| **Node Name**        | **Incoming Data**                     | **Outgoing Data**                      | **Purpose** |
|----------------------|--------------------------------------|--------------------------------------|------------|
| `whc`               | Crew urine input                     | Waste to `waste_collector`            | Collects urine & hygiene wastewater. |
| `waste_collector`   | Waste from `whc`                     | Processed waste to `upa`              | Stores wastewater and transfers it to UPA. |
| `upa`               | Wastewater from `waste_collector`     | Distillate water to `filter`          | Processes urine to extract water. |
| `filter`            | Distillate from `upa`                | Filtered water to `catalytic_chamber` | Removes organics, ammonia, and particulates. |
| `catalytic_chamber` | Water from `filter`                  | Oxidized water to `ionization`        | Sterilizes water and removes VOCs. |
| `ionization`        | Water from `catalytic_chamber`        | Purified water to `clean_water_tank`  | Final purification and iodine treatment. |
| `clean_water_tank`  | Clean water from `ionization`         | Dispensed water for crew, OGS system  | Stores potable water and sends data. |

---

## **Sample Output Log for Each Node**
### **1. WHC Node (Waste & Hygiene Compartment)**
```plaintext
[INFO] Waste & Hygiene Compartment activated.
[INFO] Urine collection in progress...
[INFO] 1.5 liters of waste collected. Sending to Waste Collector...
```

### **2. Waste Collection Node**
```plaintext
[INFO] Waste Collection System initialized.
[INFO] Received 1.5 liters of wastewater from WHC.
[INFO] Stored waste level: 5.0 liters.
[INFO] Threshold reached! Sending to Urine Processor Assembly...
```

### **3. Urine Processor Assembly (UPA)**
```plaintext
[INFO] Urine Processor Assembly started.
[INFO] Received 5.0 liters of wastewater.
[INFO] Distillation in progress...
[INFO] 4.0 liters of clean water extracted.
[INFO] Sending water to Filtration Unit...
```

### **4. Filtration Unit**
```plaintext
[INFO] Filtration Unit active.
[INFO] Received 4.0 liters of water from UPA.
[INFO] Removing dissolved solids and ammonia...
[INFO] 90% organics removed. Sending to Catalytic Chamber...
```

### **5. Catalytic Chamber**
```plaintext
[INFO] Catalytic Chamber activated.
[INFO] Received 3.8 liters of filtered water.
[INFO] Heating water to 150°C for sterilization...
[INFO] Microbial sterilization complete. Sending to Ionization Bed...
```

### **6. Ionization Bed**
```plaintext
[INFO] Ionization Bed active.
[INFO] Removing final contaminants and adding iodine...
[INFO] Iodine level: 0.5 mg/L.
[INFO] Purified water sent to Product Water Tank...
```

### **7. Product Water Tank**
```plaintext
[INFO] Product Water Tank initialized.
[INFO] Water level: 80.0 liters.
[INFO] Crew requested 2.0 liters for drinking.
[INFO] Dispensing 2.0 liters. Remaining: 78.0 liters.
[INFO] Publishing tank status to OGS...
```

---

## **How to Run the Simulation**
### **1. Build and Source the ROS 2 Package**
```bash
colcon build --packages-select demo_nova_sanctum
source install/setup.bash
```

### **2. Launch All Nodes**
```bash
ros2 launch demo_nova_sanctum wrs_systems.launch.py
```

### **3. Monitor Logs**
To check logs for each node:
```bash
ros2 topic echo /wpa/tank_status
ros2 topic echo /wpa/product_water_tank
```

### **4. Request Water from Product Water Tank**
```bash
ros2 service call /wpa/dispense_water demo_nova_sanctum/srv/CleanWater "{water: 2.0, iodine_level: 0.5}"
```

### **5. Check System Performance**
To check CPU & memory usage:
```bash
htop
```

### REFERENCES: 

![Image](https://github.com/user-attachments/assets/90783fa2-7603-4ee5-bd7e-04d7174cbc52)


![Image](https://github.com/user-attachments/assets/93306b71-0c29-4237-9f29-42a373c9fe30)