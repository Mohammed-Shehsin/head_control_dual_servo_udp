# Head Orientation Theory – 5-Point Boundary Method + Roll Compensation

This document describes the **complete mathematical and geometric foundation** of the head-orientation tracking system used in `head_control_dual_servo_udp`.

The system converts:

> **Camera image → MediaPipe FaceMesh → Yaw/Pitch scores → Servo angles (0–180°)**

with fully roll-compensated, scale-invariant orientation estimation.

---

# 1. High-Level Pipeline

```
Camera Frame
     │
     ▼
MediaPipe FaceMesh (468 pts)
     │
     ▼
Extract Nose + Eye + Oval Landmarks
     │
     ▼
Compute Roll (head tilt)
     │
     ▼
Rotate All Points → Upright Virtual Face
     │
     ▼
Compute Distances to 4 Boundaries
     │
     ▼
Compute Continuous Yaw/Pitch Scores [-1,1]
     │
     ▼
Smooth + Calibrate
     │
     ▼
Map to Servo Angles (0–180°)
     │
     ▼
Send "yaw,pitch\n" via UDP to ESP32
     │
     ▼
Servo PWM → Dual Axis Control
```

---

# 2. Geometric Structure of the Method

After roll compensation, the head alignment becomes:

```
             (P_U: Upper Boundary)
                     ●
                     │
          P_L ● -----+----- ● P_R
                     │
                     ●  (P_n: Nose Tip)
                     │
             (P_D: Lower Boundary)
```

Key idea:

- If nose moves closer to **P_L** → head turned left  
- Closer to **P_R** → head turned right  
- Closer to **P_U** → head tilted up  
- Closer to **P_D** → head tilted down  

This method uses only **geometric distances**, making it extremely stable.

---

# 3. Landmark Definitions

From MediaPipe FaceMesh:

- Nose tip:  
  - \(P_n\) (landmark index 1)
- Left eye outer corner:  
  - \(P_{LE}\) (index 33)
- Right eye outer corner:  
  - \(P_{RE}\) (index 263)
- Face oval:  
  - All points in `FACEMESH_FACE_OVAL`

From oval points, derive:

- \(P_L\) = point with minimum x  
- \(P_R\) = point with maximum x  
- \(P_U\) = point with minimum y  
- \(P_D\) = point with maximum y  

We define face center \(C\) as midpoint of oval bounding box.

---

# 4. Roll Angle Computation

We compute roll using the eye-line vector:

$$
\theta_{roll} =\\arctan2\bigl(
  y_{RE} - y_{LE},\;
  x_{RE} - x_{LE}
\bigr)
$$

- If theta_{roll} = 0 → eyes are perfectly horizontal  
- If theta_{roll} \ne 0 → head is tilted  
  
---

# 5. Roll Compensation (Coordinate Rotation)

To make the orientation frame stable, all points are rotated by:

$$
P' = R(-\theta_{roll}) \bigl( P - C \bigr) + C
$$

Where rotation matrix:

$$
R(\alpha) =
\begin{bmatrix}
\cos\alpha & -\sin\alpha \\
\sin\alpha &  \cos\alpha
\end{bmatrix}
$$

After this rotation:

- Horizontal axis = **true left/right**  
- Vertical axis = **true up/down**  
- Tilted faces become upright virtually  

---

# 6. Effective Face Dimensions

Using rotated boundary points:

$$
W = \lVert P_R - P_L \rVert
$$

$$
H = \lVert P_D - P_U \rVert
$$

Where:

- \(W\) = effective face width  
- \(H\) = effective face height  

To prevent division errors:

- If \(W < \varepsilon\), set \(W = \varepsilon\)  
- If \(H < \varepsilon\), set \(H = \varepsilon\)  

---

# 7. Normalized Distances from Nose

Left/right:

$$
d_L = \frac{\lVert P_n - P_L \rVert}{W},
\qquad
d_R = \frac{\lVert P_n - P_R \rVert}{W}
$$

Up/down:

$$
d_U = \frac{\lVert P_n - P_U \rVert}{H},
\qquad
d_D = \frac{\lVert P_n - P_D \rVert}{H}
$$

Features:

- Scale-invariant  
- Jitter-resistant  
- Meaningful geometry  

---

# 8. Continuous Yaw Score

$$
S_{yaw}=
\frac{d_R - d_L}{d_R + d_L}
$$

Interpretation:

- \(S_{yaw} > 0\) → head turned **right**  
- \(S_{yaw} < 0\) → head turned **left**  
- \(S_{yaw} = 0\) → centered  

Range:  
\[
S_{yaw} \in [-1,1]
\]

---

# 9. Continuous Pitch Score

$$
S_{pitch}=
\frac{d_D - d_U}{d_D + d_U}
$$

Interpretation:

- \(S_{pitch} > 0\) → head tilted **up**  
- \(S_{pitch} < 0\) → head tilted **down**  

Range:  
\[
S_{pitch} \in [-1,1]
\]

---

# 10. Calibration (C Key)

The user may not start exactly at center.

So pressing **C** stores:

$$
{yaw\_off} = S_{yaw}^{current},
\qquad
{pitch\_off} = S_{pitch}^{current}
$$

Corrected:

$$
S_{yaw}^{cal} = S_{yaw}^{raw} - {yaw\_off}
$$

$$
S_{pitch}^{cal} = S_{pitch}^{raw} - {pitch\_off}
$$

---

# 11. Smoothing (Moving Average)

For \(N\) previous frames:

$$
S_{yaw}^{smooth}=
\frac{1}{N}
\sum_{k=1}^{N}
S_{yaw}^{(k)}
$$

$$
S_{pitch}^{smooth}=
\frac{1}{N}
\sum_{k=1}^{N}
S_{pitch}^{(k)}
$$

This removes noise and jitter completely.

---

# 12. Mapping Scores to Servo Angles

Servos use angle range:

\[
0^\circ \le a \le 180^\circ
\]

Center = **90°**

---

## 12.1 Yaw Servo Mapping

$$
\text{servo\_yaw}=
90
+
S_{yaw}^{smooth} \cdot 90
$$

- \(+1\) → 180°  
- \(0\) → 90°  
- \(−1\) → 0°  

---

## 12.2 Pitch Servo Mapping (Gain + Inversion)

Because the user cannot tilt head too much, apply gain \(G\):

$$
S_{pitch}^{eff}=
\text{clamp}
\bigl(
G \cdot S_{pitch}^{smooth},
-1,\; 1
\bigr)
$$

Servo pitch:

$$
{servo\_pitch} = 90
-S_{pitch}^{eff} \cdot 90
$$

---

# 13. UDP Packet Format

The ESP32 receives ASCII text:

```
<servo_yaw>,<servo_pitch>\n
```

Example:

```
120,80
45,150
90,90
```

---

# 14. Full Process Visualization

```
                     FaceMesh (468 pts)
                              │
                              ▼
               Select Nose + Eyes + Oval Points
                              │
                              ▼
              Compute Roll (theta_roll)
                              │
                              ▼
             Rotate All Points (-theta_roll)
                              │
                              ▼
      Compute P_L, P_R, P_U, P_D, W, H, d_L,d_R,d_U,d_D
                              │
                              ▼
       Compute S_yaw and S_pitch in [-1,1]
                              │
                              ▼
     Smooth + Calibrate → S_yaw_s, S_pitch_s
                              │
                              ▼
      Servo Mapping (0–180°) for yaw & pitch
                              │
                              ▼
            Send "yaw,pitch\n" via UDP → ESP32
                              │
                              ▼
                     PWM → Dual Servos
```

---

# 15. Why This Method Is Superior

Compared to bounding box center, direct landmarks, or naive trigonometry:

- Fully compensates for **roll**  
- Immune to scale changes  
- Extremely **stable**  
- Continuous, smooth control  
- Works with small head movements  
- Low computational cost  
- Ideal for robotic or HCI applications  

This technique is reliable enough for real-time servo control, robotics, accessibility tools, and camera gimbals.

---

# End of Document
