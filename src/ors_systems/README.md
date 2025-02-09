
### **Oxygen Recovery System Simulation**

This project simulates the **Oxygen Recovery System (ORS)** used on the **International Space Station (ISS)**. The system utilizes multiple processes, including **water electrolysis, hydrogen separation, oxygen adsorption, and water recirculation**, to sustain a closed-loop environment for astronauts.

## **Overview**
The **ORS** consists of the following primary processes:

1. **Deionization Process**:
   - Removes **contaminants, iodine, and gas bubbles** from the water before electrolysis.
   - Ensures that **ionized water** is pure and ready for oxygen recovery.

2. **Electrolysis Process**:
   - Splits **H₂O into hydrogen (H₂) and oxygen (O₂)** using electrolysis.
   - Releases **oxygen to the cabin** and **sends hydrogen for further processing**.

3. **Sabatier Process**:
   - Uses **hydrogen (H₂) from electrolysis** and **carbon dioxide (CO₂) from the crew**.
   - Produces **methane (CH₄) and water (H₂O)**, closing the loop.

4. **Water Recirculation**:
   - Cools **excess water** and **returns it to the electrolysis chamber**.
   - Ensures a **sustainable closed-loop system**.

---

## **System Architecture & Communication**

### **Nodes & Topics**
| **Node** | **Function** | **Published Topics** | **Subscribed Topics** |
|----------|-------------|---------------------|---------------------|
| **Deionization Chamber** (`deionization_chamber.cpp`) | Purifies incoming water | `/ionized_water` | `/water_input` |
| **Electrolysis Process** (`electrolysis.cpp`) | Splits H₂O into O₂ & H₂ | `/hydrogen`, `/oxygen_supply`, `/thermal_control` | `/ionized_water` |
| **Sabatier Reactor** (`sabatier.cpp`) | Converts H₂ + CO₂ into CH₄ & H₂O | `/water_recovered`, `/methane` | `/hydrogen`, `/co2_input` |
| **Water Publisher** (`water_pub.cpp`) | Simulates incoming water | `/water_input` | None |
| **Thermal Control System** (Integrated into electrolysis) | Cools recirculated water | `/thermal_control` | None |

---

## **Flow of Data**
1. **Water enters the deionization chamber** (`/water_input → /ionized_water`).
2. **Electrolysis converts water into hydrogen and oxygen** (`/ionized_water → /hydrogen, /oxygen_supply`).
3. **Hydrogen is sent to the Sabatier reactor**, where it reacts with CO₂ (`/hydrogen → /water_recovered, /methane`).
4. **Recirculation Loop** cools and returns water to electrolysis (`/thermal_control`).

---

## **Detailed Process Breakdown**
### **1. Deionization Process**
- **Input:** Raw water (`/water_input`).
- **Output:** Purified **ionized water** (`/ionized_water`).
- **Tasks:**
  - Removes **iodine**, **contaminants**, and **gas bubbles**.
  - Ensures that **clean water enters electrolysis**.

---

### **2. Electrolysis Process**
- **Input:** Purified water (`/ionized_water`).
- **Output:**
  - **Hydrogen** (`/hydrogen`) → Sent to **Sabatier Reactor**.
  - **Oxygen** (`/oxygen_supply`) → Sent to **cabin** for crew.
  - **Heat** (`/thermal_control`) → Sent to **cooling system**.
- **Tasks:**
  - Uses **electrolysis reaction** to split H₂O into H₂ and O₂.
  - Water level **decreases with each cycle** (30% per iteration).
  - Once water is **depleted, it waits for new input**.

---

### **3. Sabatier Reactor Process**
- **Input:**
  - Hydrogen (`/hydrogen`) from electrolysis.
  - Carbon dioxide (`/co2_input`) from the crew.
- **Output:**
  - **Methane (CH₄)** (`/methane`) → Vented into space.
  - **Recovered Water (H₂O)** (`/water_recovered`) → Sent back to **electrolysis**.
- **Reaction:**
  \[
  4H₂ + CO₂ → CH₄ + 2H₂O
  \]
- **Tasks:**
  - Uses **hydrogen from electrolysis**.
  - Reacts with **CO₂ to form methane and water**.
  - **Returns water to recirculation**.

---

### **4. Water Recirculation**
- **Input:** Excess water from **Sabatier reactor & electrolysis**.
- **Output:** **Cooled water returned to electrolysis**.
- **Tasks:**
  - **Cools down heated water** from electrolysis.
  - **Maintains pressure balance** for electrolysis.
  - Ensures **continuous oxygen recovery**.

---

## **ROS 2 Parameters**
| **Parameter** | **Default Value** | **Description** |
|--------------|------------------|----------------|
| `efficiency_factor` | `1.0` | Electrolysis efficiency (100%) |
| `cooling_rate` | `5.0` | Cooling per second |
| `required_pressure` | `2.0` | Minimum pressure for electrolysis |
| `depletion_factor` | `0.3` | Water depletion per cycle (30%) |

---

## **Example Execution**
1. **Start ROS 2 system:**
   ```sh
   ros2 launch demo_nova_sanctum ogs_systems_v1.launch.py
   ```
2. **Set Parameters Dynamically: TODO**
   ```sh
   ros2 param set /electrolysis_node efficiency_factor 0.98
   ros2 param set /electrolysis_node cooling_rate 7.0
   ros2 param set /electrolysis_node required_pressure 3.0
   ros2 param set /electrolysis_node depletion_factor 0.3
   ```

---

## **System Behavior: Expected Outputs**
- **Water depletes** over cycles → `100L → 70L → 49L → ... → 0L`
- **Electrolysis stops when water = 0**, waits for **new water**.
- **Hydrogen is converted in Sabatier reactor** to **recover water**.
- **Recirculation ensures continuous water supply**.

---

## **Image Reference**
This diagram illustrates the **Oxygen Recovery System** architecture.

![image](https://github.com/user-attachments/assets/3b07155a-b415-404f-ba10-e230ec83b447)


Courtesy: Analysis of Resin Samples From a Return-To -Ground Inlet 
Deionizing Bed for the ISS Oxygen Generation System 
Elizabeth M. Bowman0F1, Danielle N. Bowman1F2, Darren S. Dunlap2F3, David A. Jackson3F4, and Natalee E. Weir4F5 
The Boeing Company, Huntsville, AL, 35824

