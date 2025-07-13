# 🤖 Robotic Mining Exploration

This project simulates autonomous navigation through an **abandoned mine** using a ground robot equipped with ultrasonic and laser sensors. The system is implemented in **MATLAB** and **APOLO**, a robotics simulation environment developed by the Technical University of Madrid (UPM), similar to Gazebo.

## 🗺️ Project Overview

The environment represents a mine filled with obstacles and narrow passages. A map of the mine has been created in XML format and loaded into APOLO. The robot must navigate from the entrance to the exit, avoiding collisions and optimizing its path.

Key features:
- **A\*** path planning algorithm to compute optimal navigation routes.
- **Ultrasound and laser-based perception** for obstacle detection.
- **Odometry correction** and sensor error compensation for accurate localization.
- Integration between **MATLAB code** and **APOLO simulation**.

## 🧠 How It Works

1. **Environment Setup**:
   - The mine map is loaded into APOLO (`Map/` folder, `.xml` format).

2. **Path Planning and Control**:
   - Using sensor data, the robot builds a local understanding of its surroundings.
   - An A\* algorithm calculates the best path from the start to the goal.
   - MATLAB handles the robot's control logic, state updates, and trajectory tracking.

3. **Execution**:
   - Once the map is loaded into APOLO, the MATLAB script `Ejectuable_Mina.m` is executed to start the simulation.
   - The robot autonomously moves through the mine toward the goal, adjusting its behavior based on real-time feedback.

## 📁 Repository Structure

```
Robotic-Mining-Exploration/
├── Map/
│   └── mine_map.xml         # XML map file for APOLO
├── Code/
│   ├── Ejectuable_Mina.m    # Main executable MATLAB script
│   └── other_scripts.m      # Supporting MATLAB functions
└── README.md
```

## ▶️ How to Run

1. Open **APOLO**.
2. Load the XML map from the `Map/` directory.
3. Run the file `Ejectuable_Mina.m` located in the `Code/` directory using MATLAB.
4. The robot will begin navigating through the simulated mine.

## 🎥 Demonstration Videos

You can watch the robot in action at the following links:

- [🔗 Video 1](https://www.youtube.com/watch?v=ThW9-Oaerkg)
- [🔗 Video 2](https://www.youtube.com/watch?v=Gasc9nnEPzw)
- [🔗 Video 3](https://www.youtube.com/watch?v=q--AdVs8ZvI)

---

Made with 💻 using MATLAB and APOLO  
By **Héctor Gordillo García** & **Javier Fernández Pintor**
