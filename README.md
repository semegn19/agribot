# AgriBot-ET 🚜🌾  
ROS-Based Autonomous Crop Monitoring and Disease Detection for Precision Agriculture in Ethiopia

## Overview

AgriBot-ET is a **ROS-based autonomous field robot** simulated in Gazebo that navigates crop rows, detects unhealthy plants using YOLOv8, and logs detections with spatial coordinates for targeted treatment. The project is designed as a **proof-of-concept** for low-cost precision agriculture in Ethiopia, focusing on small-to-medium scale farms. 

Core capabilities:

- Autonomous navigation in crop rows using the ROS Navigation Stack (SLAM + AMCL). 
- Real-time plant health detection (healthy vs. diseased) using YOLOv8. 
- CSV-based logging of diseased plant coordinates with confidence scores.  
- Optional post-processing **health map** generation from logs (heatmap of disease distribution).

***

## Features

- **Full ROS Simulation Stack**
  - Gazebo world of a 50 m × 20 m farm with 10 crop rows.
  - Four-wheeled differential-drive robot with RGB-D camera and 2D LiDAR.

- **Navigation & SLAM**
  - Gmapping-based SLAM for map building.
  - AMCL for localization and planning. 

- **AI Vision**
  - YOLOv8-nano model trained on healthy/diseased crop images.
  - Real-time inference (~17–20 FPS at 640×480). 
  - link for the model: https://colab.research.google.com/drive/1_N3QFujBcuTH644Mk3EqxbfX8R2GNJOy?usp=sharing

- **Data Logging & Analytics**
  - ROS node that logs detections (timestamp, position, status).
  
