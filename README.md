# Cuboid Rotation Estimation from Depth Images

This project estimates the rotation of a 3D cuboidal box by analyzing a series of depth images provided in a ROS 2 bag file. The implemented algorithm calculates the orientation and visible area of the box's largest face in each frame and then determines the overall axis of rotation.

This solution was developed for the Perception Task by 10xConstruction.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Algorithm](#-algorithm)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [How to Run](#-how-to-run)
- [Output Files](#-output-files)
- [Visualization Features](#-visualization-features)

---

## 📖 Overview

The goal is to analyze a ROS bag file containing depth sensor data of a rotating cuboid. The script processes each depth frame to achieve two primary objectives:

1.  **Per-Frame Analysis**: For each image, calculate the normal angle and visible area (in $m^2$) of the largest visible face relative to the camera.
2.  **Aggregate Analysis**: After processing all frames, determine the single 3D vector representing the box's axis of rotation.

---

## ⚙️ Algorithm

The solution follows a multi-stage pipeline:

1.  **Data Ingestion**: Reads the ROS 2 bag file and deserializes depth image messages from the specified topic.
2.  **Point Cloud Conversion**: Each 2D depth image (an array of distance values) is projected into a 3D point cloud using a pinhole camera model.
3.  **Plane Segmentation**: The **RANSAC (Random Sample Consensus)** algorithm is used to robustly identify the dominant plane in the point cloud, which corresponds to the largest visible face of the cuboid.
4.  **Property Calculation**:
    - The **normal vector** of the detected plane is calculated.
    - The **angle** between this normal and the camera's viewing axis is computed.
    - The **visible area** of the plane is determined by calculating the convex hull of its 3D points.
5.  **Axis of Rotation Estimation**: **Principal Component Analysis (PCA)** is performed on the collection of normal vectors from all frames. The eigenvector corresponding to the smallest eigenvalue gives the axis of rotation.

---

## 📦 Requirements

The script is written in Python 3. The following libraries are required:

- `numpy`
- `rosbags`
- `open3d`
- `scikit-learn`
- `scipy`
- `opencv-python`
- `matplotlib`

---

## 🛠️ Installation

1.  **Clone the repository** or download the source files.

2.  **Create a virtual environment** (recommended):
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install the required packages**:
    ```bash
    pip install numpy rosbags open3d scikit-learn scipy opencv-python matplotlib
    ```

---

## ▶️ How to Run

1.  **Configure the Script**: Open the main Python script (`solve_perception_task.py` or similar) and modify the configuration variables at the top of the `main()` function:
    - `BAG_FILE_PATH`: Set this to the path of your ROS 2 bag **folder**.
    - `FX, FY, CX, CY`: Update these with the correct camera intrinsic parameters if they are known. The current values are standard defaults.

2.  **Execute the script** from your terminal:
    ```bash
    python3 main.py
    ```

The script will process all the images in the bag file and generate the output files in the same directory.

---

## 📄 Output Files

Upon successful execution, the script will generate two files:

1.  **`results_table.txt`**: A CSV file containing the frame number, the calculated normal angle (in degrees), and the visible area (in $m^2$) for each frame.

    ```
    Image #   , Normal Angle (deg)  , Visible Area (m^2)
    1         , 88.07, 8.4880
    2         , 13.46, 14.3542
    ...
    ```

2.  **`rotation_axis.txt`**: A text file containing the final computed 3D axis of rotation vector.

    ```
    # Axis of Rotation Vector (X, Y, Z)
    0.17842668 0.73591027 0.65314622
    ```

---

## ✨ Visualization Features

The script includes an optional visualization mode to help verify the algorithm's performance. You can enable them by setting the corresponding flags to `True` in the `main()` function.

- **`VISUALIZE_SHADED_IMAGE`**: Displays the first depth image as a 2D color map. The region identified by RANSAC is shaded.
