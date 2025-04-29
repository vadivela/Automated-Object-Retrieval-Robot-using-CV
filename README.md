# Automated Object Retrieval System Using Color and Shape Recognition

This project integrates computer vision and robotics to develop an innovative system for detecting, retrieving, and relocating objects within a predefined arena. Utilizing a Raspberry Pi-based overhead vision system and a mobile robot equipped with a Parallax Propeller Board, the system identifies objects based on color and shape using OpenCV and executes precise robotic manipulation for object retrieval.

---
## Project Demo
![Project Demo](Demo.gif)

## Key Features
- **Overhead Vision System**: Captures and processes images of the arena for object and robot localization.
- **Object Recognition**: Identifies objects based on color and shape using advanced OpenCV algorithms.
- **Robotic Navigation**: Implements PID control for smooth and precise navigation.
- **Object Manipulation**: Efficiently picks and places objects using a servo-controlled gripper.
- **Real-World Applications**: Demonstrates potential in logistics, automation, and smart manufacturing.

---
## Workflow

1. **Camera Calibration**
   - Intrinsic and extrinsic parameters of the camera are determined using a checkerboard pattern for accurate mapping of image coordinates to real-world coordinates.

2. **Color and Shape Detection**
   - Images are processed to isolate objects of specific colors using HSV masking.
   - Centroids of objects are calculated, and objects are classified based on their area.

3. **Object Localization**
   - The system transforms image coordinates to world coordinates using the calibrated camera parameters.

4. **Navigation**
   - The robot navigates to the target object using a PID control loop.
   - Real-time feedback ensures smooth and accurate motion toward the object.

5. **Object Manipulation**
   - The robot picks up the identified object using the gripper and relocates it to a designated area based on its color.

---

## Algorithm Highlights

### **Camera Calibration**
- Captures 30 images of a checkerboard for calibration.
- Processes images in MATLAB to compute intrinsic and extrinsic parameters, enabling accurate real-world coordinate mapping.

### **Object Detection**
1. Converts captured images from BGR to HSV color space for better color segmentation.
2. Masks specific colors (blue, green, magenta) and identifies object centroids using moments.
3. Classifies objects (patches and cubes) based on area.

### **PID Navigation**
- Uses the difference between the robot’s orientation and the target direction as the error term.
- PID parameters tuned in Simulink: Kp=1, Ki=0, Kd=0.
- Ensures smooth trajectory control and precise navigation.

---
## Components Used

### **1. Propeller Activity Board**
A microcontroller with 8 parallel-processing cores (cogs), enabling multitasking for robot navigation, gripper control, and communication.

### **2. Parallax Boe-Bot Robot**
Serves as the mobile platform for the robot, providing a sturdy chassis with mounting points for servos and sensors.

### **3. Raspberry Pi 5**
Acts as the brain of the system, running the computer vision algorithms and directing the robot's actions.

### **4. Pi Camera 3**
Captures high-quality images of the arena for object detection and localization.

### **5. Parallax Servo Motors**
Provides precise control for driving the robot and manipulating objects with the gripper.

### **6. Gripper Mechanism**
A servo-based gripper for securely picking and placing objects.

### **7. Bluetooth Module**
Facilitates wireless communication between the Raspberry Pi and the Propeller Activity Board.

---

## Results

- **High Accuracy**: The system achieved reliable detection of objects and precise relocation within the arena.
- **Efficient Navigation**: PID control provided smooth and accurate movement toward targets.
- **Robust Manipulation**: The servo-based gripper successfully executed pick-and-place tasks with minimal errors.

---

## Potential Applications
- Logistics: Automating warehouse operations for item retrieval and sorting.
- Manufacturing: Enhancing efficiency in assembly lines.
- Research: Serving as a platform for further exploration in robotics and computer vision.

---





---


