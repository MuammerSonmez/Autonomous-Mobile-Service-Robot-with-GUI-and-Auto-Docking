# Autonomous-Mobile-Service-Robot-with-GUI-and-Auto-Docking

This project is a graduation thesis focusing on the design and simulation of an autonomous mobile service robot (AMR) tailored for cafe and restaurant environments. Developed using **ROS2 Humble**, **C++**, and **Python**, the system features a custom simulation environment, a robust navigation stack, and a user-friendly desktop control interface.

## Project Overview

The primary goal is to automate repetitive tasks such as order delivery in service sectors. The robot is capable of autonomous mapping, localization, and navigation within a dynamic cafe environment while communicating seamlessly with a desktop GUI used by the staff.

## Key Features

### 🧠 Intelligent Task Management (FIFO)
The core logic of the robot is governed by a custom **C++ Manager Node**. It implements a **First-In, First-Out (FIFO)** queuing system. This ensures that orders received from the GUI are processed sequentially and fairly, preventing task conflicts and ensuring smooth service flow.

### 🔋 Lidar-Based Auto-Docking
Unlike traditional IR-based solutions, this project implements a custom **Lidar-based docking algorithm**. When the battery level is critical or the "Return Home" command is issued, the robot detects the specific geometry of the charging station using laser scan data and performs a precise, autonomous alignment and docking maneuver.

### 🖥️ Desktop Control Interface (GUI)
A modern dashboard developed with **Qt for Python (PySide6)** allows cafe staff to:
* Create table-specific orders.
* Monitor real-time robot telemetry (Battery, Velocity, Position).
* Visualize the robot's location on the map.
* Execute manual override or emergency stop commands.

### 🗺️ Autonomous Navigation
Built on top of the **Nav2 Stack** and **SLAM Toolbox**, the robot performs real-time obstacle avoidance and path planning in a custom-designed Gazebo simulation world.

## System Architecture

The project follows a modular architecture based on the "Separation of Concerns" principle:

* **`cafe_simulation`**: Contains the custom Gazebo world (`kafe.world`), robot description (URDF/XACRO), and simulation launch files.
* **`cafe_robot_manager` (The Brain)**: A C++ package handling the State Machine, FIFO task queue, Nav2 action clients, and the auto-docking algorithm.
* **`cafe_gui`**: A Python package providing the visual interface for human-robot interaction.

---
*Developed using ROS2 Humble Hawksbill & Gazebo Classic.*