# -*- coding: utf-8 -*-
"""
Tello human interaction: SEARCH -> LOCK -> 10s COUNTDOWN -> FAST APPROACH
with a HARD SAFE-STOP at a standoff distance.

STATE MACHINE:
  SEARCHING : no person -> slowly rotate in place to scan the room.
  LOCKED    : person found -> stop scanning, centre on them (yaw + up/down),
              and run a 10-second countdown while keeping them centred.
  APPROACH  : after countdown -> drive forward BRISKLY, but the distance
              controller (based on bounding-box size) DECELERATES and HARD
              STOPS at SAFE_STANDOFF. It never rams: forward speed is clamped
              to zero once the person's apparent size reaches the stop target,
              and reverses if it overshoots too close.
  DONE      : hold at standoff, then land.

Safety:
  - Forward motion is ALWAYS gated by the distance controller. If the person
    is lost during approach, forward thrust is cut immediately and it hovers.
  - Hard stop the instant box-size >= stop threshold (i.e. within standoff).
  - 'q' or Ctrl-C -> land at any time.

REQUIREMENTS: pip install djitellopy opencv-python torch torchvision
Connect PC to Tello WiFi first. Use in a clear, controlled space.
"""

import time
import cv2
import torch

from djitellopy import Tello

# ---------------------------------------------------------------------------
FRAME_W, FRAME_H = 640, 480
CONF_THRESHOLD = 0.45

# Centering deadzones / limits
DEAD_X_PX = 40
DEAD_Y_PX = 45
MAX_YAW = 60
MAX_UD  = 30

# Search behaviour
SEARCH_YAW_SPEED = 30       # deg/s rotate while scanning
LOCK_CONFIRM_FRAMES = 3     # consecutive detections needed to confirm a lock

# Countdown
COUNTDOWN_S = 5

# Approach / distance control (bounding-box height fraction as distance proxy)
# Larger fraction = closer. We approach until the person fills STOP_BOX_FRAC
# of the frame height, then HARD STOP.
STOP_BOX_FRAC   = 0.90      # <-- standoff: stop when box is this tall
BACKOFF_BOX_FRAC = 0.80     # if we overshoot closer than this, back off a touch
APPROACH_FWD_SPEED = 50     # cm/s brisk forward while far
APPROACH_MAX_TIME  = 6.0    # s safety cap on the approach phase

# ---------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, out_limit, integ_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit; self.integ_limit = integ_limit
        self.integral = 0.0; self.prev_err = 0.0; self.prev_t = None
    def reset(self):
        self.integral = 0.0; self.prev_err = 0.0; self.prev_t = None
    def update(self, err):
        now = time.time()
        dt = 1e-3 if self.prev_t is None else max(1e-3, now - self.prev_t)
        self.prev_t = now
        self.integral += err * dt
        self.integral = max(-self.integ_limit, min(self.integ_limit, self.integral))
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp*err + self.ki*self.integral + self.kd*deriv
        return max(-self.out_limit, min(self.out_limit, out))

yaw_pid = PID(0.22, 0.02, 0.06, MAX_YAW, 400)
ud_pid  = PID(0.25, 0.02, 0.05, MAX_UD,  300)


def pick_person(results):
    dets = results.xyxy[0].cpu().numpy()
    best = None; best_area = 0
    for x1, y1, x2, y2, conf, cls in dets:
        if int(cls) != 0 or conf < CONF_THRESHOLD:
            continue
        area = (x2-x1)*(y2-y1)
        if area > best_area:
            best_area = area
            cx = (x1+x2)/2.0; cy = (y1+y2)/2.0; bh = (y2-y1)
            best = (cx, cy, bh, float(conf), (int(x1),int(y1),int(x2),int(y2)))
    return best


def draw_person(frame, p):
    cx, cy, bh, conf, (x1,y1,x2,y2) = p
    cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)
    cv2.circle(frame,(int(cx),int(cy)),5,(0,0,255),-1)
    cv2.putText(frame,f"person {conf:.2f}",(x1,y1-8),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)


def hud(frame, state, extra=""):
    cv2.line(frame,(FRAME_W//2,0),(FRAME_W//2,FRAME_H),(80,80,80),1)
    cv2.line(frame,(0,FRAME_H//2),(FRAME_W,FRAME_H//2),(80,80,80),1)
    cv2.putText(frame,f"[{state}] {extra}",(10,FRAME_H-15),
                cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)


def main():
    print("Loading YOLOv5...")
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.classes = [0]; model.conf = CONF_THRESHOLD
    print("Model loaded.")

    t = Tello(); t.connect()
    print(f"Battery: {t.get_battery()}%")

    t.streamon()
    frame_read = t.get_frame_read()
    time.sleep(2.0)

    print("Taking off...")
    t.takeoff()
    time.sleep(2.0)
    t.move_up(30)
    print("SEARCHING. Press 'q' or Ctrl-C to land.")

    state = "SEARCHING"
    lock_frames = 0
    countdown_start = None
    approach_start = None

    try:
        while True:
            frame = frame_read.frame
            if frame is None:
                continue
            frame = cv2.resize(frame, (FRAME_W, FRAME_H))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            person = pick_person(model(rgb, size=640))

            yaw = ud = fb = lr = 0

            # ============================ SEARCHING ============================
            if state == "SEARCHING":
                if person is not None:
                    lock_frames += 1
                    draw_person(frame, person)
                    if lock_frames >= LOCK_CONFIRM_FRAMES:
                        state = "LOCKED"
                        countdown_start = time.time()
                        yaw_pid.reset(); ud_pid.reset()
                        print("LOCKED. Countdown starting.")
                else:
                    lock_frames = 0
                    yaw = SEARCH_YAW_SPEED          # rotate to scan
                hud(frame, "SEARCHING", f"scan  hits={lock_frames}")

            # ============================= LOCKED =============================
            elif state == "LOCKED":
                if person is None:
                    # lost during countdown -> back to searching
                    state = "SEARCHING"; lock_frames = 0
                    hud(frame, "SEARCHING", "target lost")
                else:
                    cx, cy, bh, conf, box = person
                    draw_person(frame, person)
                    err_x = cx - FRAME_W/2.0
                    err_y = FRAME_H/2.0 - cy
                    if abs(err_x) > DEAD_X_PX: yaw = int(yaw_pid.update(err_x))
                    else: yaw_pid.reset()
                    if abs(err_y) > DEAD_Y_PX: ud = int(ud_pid.update(err_y))
                    else: ud_pid.reset()

                    remaining = COUNTDOWN_S - (time.time() - countdown_start)
                    if remaining <= 0:
                        state = "APPROACH"
                        approach_start = time.time()
                        print("APPROACH.")
                    hud(frame, "LOCKED", f"T-{max(0,remaining):4.1f}s")
                    # big countdown number
                    cv2.putText(frame, f"{max(0,int(remaining)+1)}",
                                (FRAME_W//2-30, FRAME_H//2-40),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0,0,255), 5)

            # ============================ APPROACH ============================
            elif state == "APPROACH":
                # Safety cap on total approach time
                if time.time() - approach_start > APPROACH_MAX_TIME:
                    state = "DONE"
                if person is None:
                    # Lost target mid-approach -> CUT forward thrust, hover
                    fb = 0
                    hud(frame, "APPROACH", "target lost - HOLD")
                else:
                    cx, cy, bh, conf, box = person
                    draw_person(frame, person)
                    box_frac = bh / FRAME_H

                    # keep centred while approaching
                    err_x = cx - FRAME_W/2.0
                    err_y = FRAME_H/2.0 - cy
                    if abs(err_x) > DEAD_X_PX: yaw = int(yaw_pid.update(err_x))
                    if abs(err_y) > DEAD_Y_PX: ud = int(ud_pid.update(err_y))

                    # ---- DISTANCE GATE: the hard safe-stop logic ----
                    if box_frac >= BACKOFF_BOX_FRAC:
                        fb = -20                       # too close: back off
                        hud(frame, "APPROACH", f"too close {box_frac:.2f} - BACK")
                    elif box_frac >= STOP_BOX_FRAC:
                        fb = 0                         # within standoff: HARD STOP
                        state = "DONE"
                        print("SAFE STOP reached.")
                        hud(frame, "APPROACH", f"STOP {box_frac:.2f}")
                    else:
                        fb = APPROACH_FWD_SPEED        # far: brisk forward
                        hud(frame, "APPROACH", f"fwd  box={box_frac:.2f}")

            # ============================== DONE ==============================
            elif state == "DONE":
                fb = 0
                if person is not None:
                    draw_person(frame, person)
                    cx, cy, bh, conf, box = person
                    err_x = cx - FRAME_W/2.0
                    if abs(err_x) > DEAD_X_PX: yaw = int(yaw_pid.update(err_x))
                hud(frame, "DONE", "safe standoff - hold")

            # send command (also keepalive)
            t.send_rc_control(lr, fb, ud, yaw)

            cv2.imshow("Tello Search/Lock/Approach", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("Landing...")
        try:
            t.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            t.land()
        except Exception as e:
            print(f"Land error: {e}")
        t.streamoff()
        t.end()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == '__main__':
    main()
