# Head Control Dual Servo (UDP)

This project implements a real-time **head orientation → dual-servo control** system using **MediaPipe FaceMesh**, **OpenCV**, and **UDP communication with ESP32**.

Your head movements (LEFT/RIGHT + UP/DOWN) wirelessly control **two servo motors** with extremely low latency and smooth motion.

---

# Demo

<p align="center">
  <img src="assets/demo.gif" width="480">
</p>



---

# Features

- Real-time head tracking  
- 5-point geometric boundary method  
- Roll compensation (stable even when tilted)  
- Continuous yaw/pitch scores in **[-1, 1]**  
- Adjustable pitch sensitivity (gain)  
- Servo mapping to **0–180°**  
- UDP communication to ESP32  
- Smooth calibrated output  
- Extremely low latency (~50 Hz)  

---

# Method Overview

The algorithm uses the following steps:

1. Extract 5 key points:
   - Nose tip
   - Left eye outer corner
   - Right eye outer corner
   - Four boundary points from the face oval  
2. Compute head roll using eye alignment  
3. Rotate all points to remove roll  
4. Compute normalized distances from nose → boundaries  
5. Compute yaw and pitch scores  
6. Apply smoothing & calibration  
7. Map to servo angles (0–180°)  
8. Send angles over UDP to ESP32  

---

# Mathematical Model

## 1. Roll Angle

$$
\theta_{roll} = \arctan2( y_{RE} - y_{LE},\; x_{RE} - x_{LE} )
$$

---

## 2. Rotation of Points

$$
P' = R(-\theta_{roll})\,(P - C) + C
$$

Where:  
- \( P \) = original point  
- \( C \) = face center  
- \( R \) = rotation matrix  

---

## 3. Effective Face Width and Height

$$
W = \lVert P_R - P_L \rVert
$$

$$
H = \lVert P_D - P_U \rVert
$$

---

## 4. Normalized Distances

$$
d_L = \frac{\lVert P_n - P_L \rVert}{W},
\qquad
d_R = \frac{\lVert P_n - P_R \rVert}{W}
$$

$$
d_U = \frac{\lVert P_n - P_U \rVert}{H},
\qquad
d_D = \frac{\lVert P_n - P_D \rVert}{H}
$$

---

## 5. Continuous Yaw & Pitch Scores

### Yaw (Left/Right)

$$
S_{yaw} = \frac{d_R - d_L}{d_R + d_L}
$$

- \( S_{yaw} > 0 \) → head turned right  
- \( S_{yaw} < 0 \) → head turned left  

---

### Pitch (Up/Down)

$$
S_{pitch} = \frac{d_D - d_U}{d_D + d_U}
$$

- \( S_{pitch} > 0 \) → head tilted up  
- \( S_{pitch} < 0 \) → head tilted down  

---

# Servo Mapping

### Yaw Servo (left/right)

```
servo_yaw = 90 + S_yaw_s * 90
```

### Pitch Servo (up/down, inverted + gain)

```
pitch_eff   = clamp(S_pitch_s * PITCH_GAIN, -1, 1)
servo_pitch = 90 - pitch_eff * 90
```

Then:

```
servo_angle = clamp(servo_angle, 0, 180)
```

Both angles are sent to ESP32 as:

```
<servo1_angle>,<servo2_angle>\n
```

---

# Installation

Install dependencies:

```
pip install opencv-python mediapipe numpy
```

Or:

```
pip install -r requirements.txt
```

---

# Usage

Run:

```
python head_control_dual_servo_udp.py --ip <ESP32_IP> --port 4210
```

Example:

```
python head_control_dual_servo_udp.py --ip 192.168.0.105 --port 4210
```

### Keyboard Controls

- **C** → Calibrate center  
- **Q** → Quit  

---

# ESP32 Side

ESP32 listens for UDP packets in format:

```
yaw_angle,pitch_angle\n
```

Moves both servos using PWM.  
Exactly the same packet format used in the **pinch-to-servo** project.

---

# Folder Structure

```
head_control_dual_servo_udp/
│
├── head_control_dual_servo_udp.py
├── requirements.txt
├── README.md
│
└── assets/
    └── demo.gif
```

---

# Author

**Mohammed Shehsin Thamarachalil Abdulresak**  
Robotics & Automation Engineer  
Poznań University of Technology, Poland

