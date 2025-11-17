import argparse
import socket
import time
import math
from collections import deque

import cv2
import numpy as np
import mediapipe as mp

# =========================
# UDP / Network Parameters
# =========================
p = argparse.ArgumentParser()
p.add_argument("--ip",  type=str, required=True, help="ESP32 IPv4 from Serial Monitor")
p.add_argument("--port", type=int, default=4210)
p.add_argument("--cam",  type=int, default=None, help="Camera index override (0/1/2/3)")
args = p.parse_args()

TARGET_IP, TARGET_PORT = args.ip, args.port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

SEND_MIN_INTERVAL = 0.02   # seconds, max ~50Hz
MIN_DELTA_DEG     = 1      # send only if angle changed at least this much

last_send_ts    = 0.0
last_sent_yaw   = None
last_sent_pitch = None

# =========================
# Servo Mapping Parameters
# =========================
ANGLE_MIN    = 0
ANGLE_MAX    = 180
ANGLE_CENTER = 90
ANGLE_RANGE  = 90   # so -1 → 0, 0 → 90, +1 → 180

# Pitch sensitivity gain (amplify up/down)
PITCH_GAIN  = 1.8        # tune: 1.5–2.2
PITCH_RANGE = ANGLE_RANGE


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# =========================
# Smoother
# =========================
class Smoother:
    def __init__(self, n=7):
        self.qx = deque(maxlen=n)
        self.qy = deque(maxlen=n)

    def push(self, sx, sy):
        self.qx.append(sx)
        self.qy.append(sy)

    def get(self):
        if not self.qx:
            return 0.0, 0.0
        return float(np.mean(self.qx)), float(np.mean(self.qy))


# =========================
# Drawing helper
# =========================
def put_text(img, text, org, scale=0.8, color=(255, 255, 255), thick=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thick, cv2.LINE_AA)


# =========================
# Geometry helper
# =========================
def rotate_points(points, center, angle_rad):
    """
    Rotate Nx2 array of points around 'center' by angle_rad (radians).
    """
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    out = np.empty_like(points)
    cx, cy = center
    for i, (x, y) in enumerate(points):
        dx = x - cx
        dy = y - cy
        xr = cx + c * dx - s * dy
        yr = cy + s * dx + c * dy
        out[i] = (xr, yr)
    return out


# =========================
# Camera initialization
# =========================
def open_camera(idx_hint=None):
    indices = [idx_hint] if idx_hint is not None else [0, 1, 2, 3]
    for i in indices:
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"[INFO] Camera opened on index {i}")
                return cap
        if cap:
            cap.release()
    raise RuntimeError("No camera opened. Try --cam 0/1/2/3 and close apps using the camera.")


# =========================
# MediaPipe / Detection Setup
# =========================
mp_face_mesh = mp.solutions.face_mesh
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

YAW_THRESH   = 0.12
PITCH_THRESH = 0.10
HYSTERESIS   = 0.02

# Colors
COL_TEXT   = (255, 255, 0)
COL_GRID   = (160, 160, 160)
COL_UP     = (255, 0, 0)
COL_DOWN   = (0, 255, 0)
COL_LEFT   = (0, 0, 255)
COL_RIGHT  = (0, 255, 255)
COL_CENTER = (255, 255, 255)


def main():
    global last_send_ts, last_sent_yaw, last_sent_pitch

    cap = open_camera(args.cam)
    cv2.namedWindow("Head → Servo Dual UDP", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Head → Servo Dual UDP", 1280, 720)

    smoother = Smoother(7)
    yaw_off = 0.0
    pitch_off = 0.0
    last_label = "CENTER"

    with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    ) as face_mesh:

        face_oval_connections = mp_face_mesh.FACEMESH_FACE_OVAL
        oval_indices = sorted(list({i for conn in face_oval_connections for i in conn}))

        NOSE_IDX = 1
        LEFT_EYE_IDX = 33
        RIGHT_EYE_IDX = 263

        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Camera read failed, reopening…")
                cap.release()
                time.sleep(0.4)
                cap = open_camera(args.cam)
                continue

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = face_mesh.process(rgb)

            label = "CENTER"
            yaw_raw = 0.0
            pitch_raw = 0.0
            yaw_smooth = 0.0
            pitch_smooth = 0.0
            yaw_angle = None
            pitch_angle = None

            if res.multi_face_landmarks:
                lms = res.multi_face_landmarks[0].landmark
                pts_xy = np.array([(lm.x * w, lm.y * h) for lm in lms], dtype=np.float32)

                oval_pts = pts_xy[oval_indices, :]

                x_min = int(max(0, np.min(oval_pts[:, 0])))
                x_max = int(min(w - 1, np.max(oval_pts[:, 0])))
                y_min = int(max(0, np.min(oval_pts[:, 1])))
                y_max = int(min(h - 1, np.max(oval_pts[:, 1])))

                cx = 0.5 * (x_min + x_max)
                cy = 0.5 * (y_min + y_max)
                face_center = (cx, cy)

                P_n = pts_xy[NOSE_IDX]
                P_le = pts_xy[LEFT_EYE_IDX]
                P_re = pts_xy[RIGHT_EYE_IDX]

                dx_eye = P_re[0] - P_le[0]
                dy_eye = P_re[1] - P_le[1]
                roll = math.atan2(dy_eye, dx_eye)

                all_key_points = np.vstack([
                    oval_pts,
                    P_n.reshape(1, 2),
                    P_le.reshape(1, 2),
                    P_re.reshape(1, 2)
                ])
                all_rot = rotate_points(all_key_points, face_center, -roll)

                oval_rot = all_rot[:len(oval_pts)]
                P_n_rot = all_rot[len(oval_pts)]

                x_min_r = np.min(oval_rot[:, 0])
                x_max_r = np.max(oval_rot[:, 0])
                y_min_r = np.min(oval_rot[:, 1])
                y_max_r = np.max(oval_rot[:, 1])

                P_L = np.array([x_min_r, P_n_rot[1]])
                P_R = np.array([x_max_r, P_n_rot[1]])
                P_U = np.array([P_n_rot[0], y_min_r])
                P_D = np.array([P_n_rot[0], y_max_r])

                W = np.linalg.norm(P_R - P_L)
                H = np.linalg.norm(P_D - P_U)
                W = max(W, 1e-6)
                H = max(H, 1e-6)

                d_L = np.linalg.norm(P_n_rot - P_L) / W
                d_R = np.linalg.norm(P_n_rot - P_R) / W
                d_U = np.linalg.norm(P_n_rot - P_U) / H
                d_D = np.linalg.norm(P_n_rot - P_D) / H

                denom_lr = max(d_L + d_R, 1e-6)
                denom_ud = max(d_U + d_D, 1e-6)

                yaw_raw = (d_R - d_L) / denom_lr      # >0 → right
                pitch_raw = (d_D - d_U) / denom_ud    # >0 → up

                yaw_score = yaw_raw - yaw_off
                pitch_score = pitch_raw - pitch_off

                smoother.push(yaw_score, pitch_score)
                yaw_smooth, pitch_smooth = smoother.get()

                yaw_thr_pos = YAW_THRESH + (HYSTERESIS if last_label == "RIGHT" else 0.0)
                yaw_thr_neg = -(YAW_THRESH + (HYSTERESIS if last_label == "LEFT" else 0.0))
                pit_thr_pos = PITCH_THRESH + (HYSTERESIS if last_label == "UP" else 0.0)
                pit_thr_neg = -(PITCH_THRESH + (HYSTERESIS if last_label == "DOWN" else 0.0))

                if yaw_smooth >= yaw_thr_pos:
                    label = "RIGHT"
                elif yaw_smooth <= yaw_thr_neg:
                    label = "LEFT"
                elif pitch_smooth >= pit_thr_pos:
                    label = "UP"
                elif pitch_smooth <= pit_thr_neg:
                    label = "DOWN"
                else:
                    label = "CENTER"

                last_label = label

                # ==============================
                # Servo Mapping (UPDATED)
                # ==============================

                # Yaw: keep original mapping (works fine)
                yaw_angle = ANGLE_CENTER + yaw_smooth * ANGLE_RANGE

                # Pitch: add gain and invert direction
                pitch_eff = clamp(pitch_smooth * PITCH_GAIN, -1.0, 1.0)
                pitch_angle = ANGLE_CENTER - pitch_eff * PITCH_RANGE

                yaw_angle = int(round(clamp(yaw_angle, ANGLE_MIN, ANGLE_MAX)))
                pitch_angle = int(round(clamp(pitch_angle, ANGLE_MIN, ANGLE_MAX)))

                # ---------- Drawing ----------
                mp_draw.draw_landmarks(
                    frame,
                    res.multi_face_landmarks[0],
                    mp_face_mesh.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mp_styles.get_default_face_mesh_tesselation_style()
                )

                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), COL_GRID, 1)

                put_text(frame, f"S_yaw_raw:   {yaw_raw:+.3f}",   (10, 28), 0.6, COL_TEXT, 1)
                put_text(frame, f"S_pitch_raw: {pitch_raw:+.3f}", (10, 48), 0.6, COL_TEXT, 1)
                put_text(frame, f"S_yaw_s:     {yaw_smooth:+.3f}",   (10, 68), 0.6, (220, 220, 220), 1)
                put_text(frame, f"S_pitch_s:   {pitch_smooth:+.3f}", (10, 88), 0.6, (220, 220, 220), 1)

                put_text(frame, f"Servo Yaw   : {yaw_angle:3d} deg",   (10, 118), 0.6, (0, 255, 255), 1)
                put_text(frame, f"Servo Pitch : {pitch_angle:3d} deg", (10, 140), 0.6, (0, 255, 255), 1)

            # Big label + help
            put_text(frame, label, (10, h - 20), 1.2, (0, 255, 255), 3)
            put_text(frame, "C: calibrate center   Q: quit",
                     (w - 360, 24), 0.6, (200, 200, 200), 1)

            # =========================
            # UDP Send
            # =========================
            now = time.time()
            if yaw_angle is not None and pitch_angle is not None:
                if now - last_send_ts >= SEND_MIN_INTERVAL:
                    send = (
                        last_sent_yaw is None or abs(yaw_angle - last_sent_yaw) >= MIN_DELTA_DEG or
                        last_sent_pitch is None or abs(pitch_angle - last_sent_pitch) >= MIN_DELTA_DEG
                    )
                    if send:
                        pkt = f"{yaw_angle},{pitch_angle}\n".encode("utf-8")
                        try:
                            sock.sendto(pkt, (TARGET_IP, TARGET_PORT))
                            print("[TX-UDP]", yaw_angle, pitch_angle)
                            last_sent_yaw = yaw_angle
                            last_sent_pitch = pitch_angle
                            last_send_ts = now
                        except Exception as e:
                            print("[UDP WARN]", e)

            cv2.imshow("Head → Servo Dual UDP", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('c'):
                yaw_off, pitch_off = smoother.get()
                print(f"[Calibrated] yaw_offset={yaw_off:+.3f}, pitch_offset={pitch_off:+.3f}")

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()

