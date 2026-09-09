# Autonomous Maze-Solving Line Follower Robot

An autonomous high-speed differential drive line-following and maze-solving robot. Controlled via PID closed-loop logic on an Arduino Nano, this robot navigates complex track grids, solves mazes via dynamic path exploration using selectable maze-solving algorithms, and backtracks to run optimized shortest paths.

Access the demo clip and snapshots through this link: [Google Drive Media Folder](https://drive.google.com/drive/folders/1v7QmGmdtYQx76mmlt2fbsmfiMcFZn4-S?usp=sharing)

---

## Key Features

* **PID-Controlled Line Tracking:** Tuned closed-loop PID algorithm for minimal error, smooth trajectory keeping, and sharp high-speed cornering.
* **Configurable Maze Solvers (LSRB / RSLB):** Supports both **Left-Hand-On-Wall (LSRB)** and **Right-Hand-On-Wall (RSLB)** exploration algorithms, allowing runtime switching prior to execution.
* **Dual Execution Modes:** Single push-button interface for mode selection:
  * **1 Click:** **Exploration Run** — Maps and solves the maze/grid using the selected priority algorithm.
  * **2 Clicks:** **Shortest Path Run** — Replays the optimized backtrack path at maximum speed.
* **Audio-Visual Feedback:** Integrated piezo buzzer and status LEDs trigger clear cues at every junction, turn, and mode change.

---

## Hardware Architecture

| Component | Specification / Model | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano | Primary processing & control logic |
| **Sensors** | 8-Channel IR Array | Real-time line position & junction sensing |
| **Actuators** | N20 Gear Motors (600 RPM) | Drive actuation with integrated quadrature encoders |
| **Chassis** | Custom CAD Model (SolidWorks) | 3D-printed chassis fabricated on Creality Ender 6 |
| **Feedback** | Piezo Buzzer & LED | Turn indications & system status signaling |
| **User Input** | Push Button | Mode & algorithm selection toggle |

---

## Software & Algorithm Overview

1. **Algorithm Switching (LSRB vs. RSLB):** When the buzzer beeps twice, click the button to select the path priority:
   * **1 Click:** Selects **LSRB** (Left, Straight, Right, Back).
   * **2 Clicks:** Selects **RSLB** (Right, Straight, Left, Back).
2. **Junction Processing:** Evaluates 8-channel IR sensor states across multiple conditional loops to identify distinct junction configurations (T-junctions, cross-sections, sharp 90° angles, and dead ends).
3. **PID Loop:** Computes positional offset relative to the line center and applies dynamic PWM adjustments to the differential drive motors.
4. **Path Optimization:** Stores turn history during exploration mode to prune dead ends and compute the shortest direct return path.

---

## Operational Guide

1. **Setup:** Flash the firmware to the Arduino Nano and place the robot on the starting position.
2. **Algorithm Selection:** Wait for the **double beep** from the buzzer:
   * Press **1 time** for LSRB rule priority.
   * Press **2 times** for RSLB rule priority.
3. **Run Execution:**
   * Press **1 click** to start the **Exploration Run**.
   * After mapping and backtracking, press **2 clicks** to execute the **Shortest Path Run**.
