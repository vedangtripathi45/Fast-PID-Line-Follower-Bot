# Autonomous Maze-Solving Line Follower Robot

An autonomous high-speed differential drive line-following and maze-solving robot. Controlled via PID closed-loop logic on an Arduino Nano, this robot navigates complex track grids, solves mazes via dynamic path exploration, and backtracks to run optimized shortest paths.

Access the demo clip and snapshots through this link: [Google Drive Media Folder](https://drive.google.com/drive/folders/1v7QmGmdtYQx76mmlt2fbsmfiMcFZn4-S?usp=sharing)

---

## Key Features

* **PID-Controlled Line Tracking:** Tuned closed-loop PID algorithm for minimal error, smooth trajectory keeping, and sharp high-speed cornering.
* **Maze Exploration & Backtracking:** Capable of mapping unknown grids, determining junctions using multi-case conditional logic, and backtracing to compute the shortest path.
* **Dual Execution Modes:** Single push-button interface for quick operational mode selection:
  * **1 Click:** **Exploration Run** — Maps and solves the maze/grid.
  * **2 Clicks:** **Shortest Path Run** — Replays the optimized backtrack path at maximum speed.
* **Audio-Visual Feedback:** Integrated piezo buzzer and status LEDs trigger clear cues at every junction and turn.

---

## Hardware Architecture

| Component | Specification / Model | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano | Primary processing & control logic |
| **Sensors** | 8-Channel IR Array | Real-time line position & junction sensing |
| **Actuators** | N20 Gear Motors (600 RPM) | Drive actuation with integrated quadrature encoders |
| **Chassis** | Custom CAD Model (SolidWorks) | 3D-printed chassis fabricated on Creality Ender 6 |
| **Feedback** | Piezo Buzzer & LED | Turn indications & system status signaling |
| **User Input** | Push Button | Mode selection toggle |

---

## Software & Algorithm Overview

1. **Junction Processing:** Evaluates 8-channel IR sensor states across multiple conditional loops to identify distinct junction configurations (T-junctions, cross-sections, sharp 90° angles, and dead ends).
2. **PID Loop:** Computes positional offset relative to the line center and applies dynamic PWM adjustments to the differential drive motors.
3. **Maze Backtracking:** Stores turn history during exploration mode to prune dead ends and compute the shortest direct return path.

---

## Getting Started

1. **Setup:** Flash the provided firmware to the Arduino Nano via Arduino IDE.
2. **Calibration:** Place the robot on the track surface to calibrate the IR sensor array threshold values.
3. **Operation:** 
   * Press the push button **once** to initiate the **Exploration Run**.
   * After the robot returns or finishes mapping, press the button **twice** to execute the **Shortest Path Run**.
