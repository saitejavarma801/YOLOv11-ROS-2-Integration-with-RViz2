# YOLOv11-ROS-2-Integration-with-RViz2

This repository demonstrates a **simple and working integration of YOLOv11 with ROS 2**
and visualization in **RViz2**.

It is intended for **students, beginners, and robotics developers** who want to:
- Run YOLOv11 on a ROS 2 camera topic
- Visualize detections in RViz2
- Use the setup as a base for robot perception projects

This repository focuses **only on perception**, not navigation or robot control.

---

## 🚀 What this repository does

- Subscribes to a ROS 2 camera image topic
- Runs YOLOv11 object detection
- Draws bounding boxes on the image
- Publishes the annotated image
- Displays the result directly in RViz2

That’s it — clean, simple, and reliable.

---

## 🛠 Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- NumPy (1.26.4)
- OpenCV
- Ultralytics YOLO

---

## ⚙️ Installation

### 1. Install YOLO V11 (Ultralytics)

    pip install ultralytics 
    
### 2. Install Numpy 1.26.4

    pip install Numpy 

----
***Download the YoloV11m.pt in the yolo_ws, it should be next to the src folder***

<img width="612" height="106" alt="image" src="https://github.com/user-attachments/assets/b56e69f9-55bb-49db-a6bc-e727f37fe980" />


## Download the Workspace

    git clone https://github.com/saitejavarma801/YOLOv11-ROS-2-Integration-with-RViz2.git

## Build The Workspace

    cd ~/yolo_ws
  
    colcon build
    
    source install/setup.bash

## Run the Package

    ros2 launch yolo_ros rviz.launch.py


<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/f7e71ccd-2d83-46d1-bd0a-69e112171974" />
