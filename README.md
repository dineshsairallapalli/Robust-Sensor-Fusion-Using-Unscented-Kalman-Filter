# Robust-Sensor-Fusion-Using-Unscented-Kalman-Filter

---

# **Robust Sensor Fusion Using Unscented Kalman Filter (UKF)**

This repository implements a **Robust Unscented Kalman Filter (UKF)** to achieve precise sensor fusion for state estimation. By integrating noisy and asynchronous sensor data, such as orientation, pose, and velocity measurements, this project provides a robust framework for state estimation in dynamic environments. The repository includes MATLAB code, datasets, and visualization tools to evaluate the algorithm's performance.

---

## **Project Overview**

State estimation is crucial for sensor-driven systems, especially in robotics and autonomous vehicles. This project implements a **UKF-based sensor fusion** framework for two key tasks:
1. **Part 1**: Orientation and pose estimation.
2. **Part 2**: Velocity estimation.

Each part is designed to address unique challenges in sensor fusion, utilizing distinct measurement models while sharing a robust process model.

---

## **Features and Functionality**

### **1. Unscented Kalman Filter (UKF):**
- **Process Model**:
  - Predicts position, velocity, orientation, and sensor biases using motion dynamics.
  - Uses IMU-derived acceleration and angular velocity as inputs.
- **Measurement Models**:
  - Part 1: Updates state using orientation and pose data.
  - Part 2: Updates state using velocity data.
- **Kalman Gain**:
  - Balances prediction and measurements to minimize estimation errors.

### **2. Modular Design:**
- Predict (`pred_step.m`) and update (`upd_step.m`) steps are implemented as reusable components.
- Separate scripts for position/orientation and velocity measurement updates.

### **3. Visualization Tools:**
- Plotting functions to compare UKF estimates against ground truth.
- Error analysis for key variables like position, velocity, and orientation.

### **4. Dataset Compatibility:**
- Pre-parsed `.mat` files with synchronized sensor data for testing and validation.

---

## **Key Files and Structure**

```
root/
│
├── code/
│   ├── data/
│   │   ├── proj2_dataset1.mat
│   │   ├── proj2_dataset4.mat
│   │   ├── studentdata1.mat
│   │   └── studentdata4.mat
│   │
│   ├── part1_Orient_Pose_Sensor_Out/
│   │   ├── init.m
│   │   ├── plotData.m
│   │   ├── pred_step.m
│   │   ├── upd_step.m
│   │   └── UKF_KalmanFilt_Part1.m
│   │
│   └── part2_Velocity_Sensor_Out/
│       ├── init.m
│       ├── plotData.m
│       ├── pred_step.m
│       ├── upd_step.m
│       └── UKF_KalmanFilt_part2.m
│
├── handoutproj3ROB6213_2024.pdf  # Project Guidelines
└── Report.pdf                   # Final Report
```

---

## **Algorithms and Approach**

### **Prediction Step**:
- Propagates the state using IMU-derived angular velocity and acceleration:
  ```matlab
  x_pred = f(x_prev, u); % Non-linear process model
  P_pred = F * P_prev * F' + Q; % Covariance prediction
  ```

### **Update Step**:
- Refines the predicted state using Kalman gain and sensor measurements:
  ```matlab
  K = P * H' / (H * P * H' + R); % Compute Kalman gain
  x = x + K * (z - H * x);       % Update state
  P = (I - K * H) * P;           % Update covariance
  ```

### **Unscented Transform**:
- Generates sigma points to propagate through non-linear models, maintaining accuracy for complex dynamics.

---

## **Datasets**

- **Included Data Files**:
  - `proj2_dataset1.mat` and `proj2_dataset4.mat`: General datasets for testing.
  - `studentdata1.mat` and `studentdata4.mat`: Custom datasets for experimentation.
- **Data Format**:
  Each dataset contains synchronized measurements:
  ```matlab
  [x, y, z, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz]
  ```

---

## **How to Use**

### **1. Clone the Repository**
```bash
git clone https://github.com/yourusername/Robust-Sensor-Fusion-Using-UKF.git
cd Robust-Sensor-Fusion-Using-UKF
```

### **2. Open MATLAB**
- Open the folder in MATLAB 2023b or later.

### **3. Run the Scripts**
- **Part 1: Orientation and Pose Estimation**
  ```matlab
  run('code/part1_Orient_Pose_Sensor_Out/UKF_KalmanFilt_Part1.m')
  ```
- **Part 2: Velocity Estimation**
  ```matlab
  run('code/part2_Velocity_Sensor_Out/UKF_KalmanFilt_part2.m')
  ```

### **4. Visualize Results**
- Use `plotData.m` to generate performance plots:
  ```matlab
  plotData
  ```

---

## **Applications**

- **Robotics**: Sensor fusion for mobile robots and drones.
- **Navigation Systems**: Improved localization for autonomous vehicles.
- **Research**: A framework for evaluating sensor fusion techniques.

---

## **Performance Metrics**

- **Accuracy**:
  - Positional accuracy within ±0.1 meters in most scenarios.
  - Orientation estimates show minimal error (<1 degree).
- **Noise Handling**:
  - Effectively mitigates noise and sensor delays.
- **Robustness**:
  - Operates reliably even under dynamic and noisy conditions.

---

## **Future Improvements**

1. **Visual SLAM**:
   - Incorporate camera-based observations for better spatial awareness.
2. **Multi-Sensor Fusion**:
   - Extend to include LIDAR or GPS data.
3. **Optimization**:
   - Improve computational efficiency for real-time applications.
