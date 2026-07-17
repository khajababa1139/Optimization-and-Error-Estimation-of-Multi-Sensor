"""
DJI Tello — pure OpenCV HUD, keyboard only
No pygame. No Tkinter. No SDL.

Requirements:
    pip install djitellopy opencv-python numpy

Controls:
    C        connect
    T        takeoff
    L        land
    X        emergency stop
    ESC      quit

    W / S    forward / back
    A / D    left / right
    R / F    up / down
    Q / E    yaw left / right
"""

import os, time, math, threading, queue
import numpy as np
import cv2

try:
    from djitellopy import Tello
    TELLO_OK = True
except ImportError:
    TELLO_OK = False
    print("[WARN] djitellopy not installed — SIMULATION mode")

# ── Palette (BGR) ─────────────────────────────────────────────────────────────
C_GREEN = (80,  210, 80)
C_BLUE  = (220, 160, 60)
C_AMBER = (40,  180, 220)
C_RED   = (60,  60,  240)
C_WHITE = (220, 220, 220)
C_GRAY  = (110, 110, 110)
C_PANEL = (30,  34,  42)

RC_SPEED = 60   # rc value sent per axis when key held
FPS      = 30
W, H     = 960, 720

# ── Simulation drone ──────────────────────────────────────────────────────────
class SimDrone:
    def __init__(self):
        self._t0  = time.time()
        self._h   = 0
        self._yaw = 0.0
        self._vx = self._vy = self._vz = 0.0
    def connect(self):   pass
    def takeoff(self):   self._h = 80
    def land(self):      self._h = 0
    def emergency(self): self._h = 0
    def streamon(self):  pass
    def streamoff(self): pass
    def get_frame_read(self): return None
    def send_rc_control(self, lr, fb, ud, yaw):
        self._vx  = lr  * 0.1
        self._vy  = fb  * 0.1
        self._vz  = ud  * 0.1
        self._yaw = (self._yaw + yaw * 0.2) % 360
        self._h   = max(0, self._h + ud * 0.05)
    def get_battery(self):      return 87
    def get_temperature(self):  return 34 + int(math.sin(time.time()) * 2)
    def get_height(self):       return max(0, int(self._h))
    def get_speed_x(self):      return round(self._vx, 1)
    def get_speed_y(self):      return round(self._vy, 1)
    def get_speed_z(self):      return round(self._vz, 1)
    def get_pitch(self):        return round(math.sin(time.time() * 0.7) * 12, 1)
    def get_roll(self):         return round(math.cos(time.time() * 0.5) * 8,  1)
    def get_yaw(self):          return round(self._yaw, 1)
    def get_distance_tof(self): return int(self._h * 10)
    def get_flight_time(self):  return int(time.time() - self._t0)
    def set_speed(self, s):     pass

# ── HUD helpers ───────────────────────────────────────────────────────────────
def put(img, text, xy, color=C_WHITE, scale=0.55, thick=1):
    cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thick+2, cv2.LINE_AA)
    cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color,   thick,   cv2.LINE_AA)

def panel(img, x, y, w, h, alpha=0.55):
    roi = img[y:y+h, x:x+w]
    cv2.addWeighted(np.full_like(roi, C_PANEL), alpha, roi, 1-alpha, 0, roi)
    cv2.rectangle(img, (x, y), (x+w, y+h), C_GRAY, 1)

def vbar(img, cx, y, h, val, color):
    mid = y + h // 2
    cv2.rectangle(img, (cx-14, y), (cx+14, y+h), (50,50,55), -1)
    cv2.line(img, (cx-14, mid), (cx+14, mid), C_GRAY, 1)
    fill = int(h / 2 * abs(val) / 100)
    if val >= 0:
        cv2.rectangle(img, (cx-13, mid-fill), (cx+13, mid), color, -1)
    else:
        cv2.rectangle(img, (cx-13, mid), (cx+13, mid+fill), color, -1)
    cv2.rectangle(img, (cx-14, y), (cx+14, y+h), C_GRAY, 1)

# ── Artificial horizon ────────────────────────────────────────────────────────
def draw_horizon(canvas, cx, cy, r, pitch, roll):
    roll_r = math.radians(roll)
    shift  = int(pitch * r / 45)
    nx = math.sin(roll_r)
    ny = -math.cos(roll_r)
    y0, y1 = max(0, cy-r), min(canvas.shape[0], cy+r+1)
    x0, x1 = max(0, cx-r), min(canvas.shape[1], cx+r+1)
    ys, xs  = np.mgrid[y0:y1, x0:x1]
    in_c    = (xs-cx)**2 + (ys-cy)**2 < r*r
    dot     = (xs-cx)*nx + (ys-cy+shift)*ny
    canvas[y0:y1, x0:x1][in_c & (dot <= 0)] = (100, 60, 20)
    canvas[y0:y1, x0:x1][in_c & (dot  > 0)] = (30, 100, 60)
    cos_r, sin_r = math.cos(roll_r), math.sin(roll_r)
    dx = int(r*cos_r); ox = int(shift*sin_r); oy = int(shift*cos_r)
    cv2.line(canvas,
             (cx - dx + ox, cy - int(r*sin_r) - oy),
             (cx + dx + ox, cy + int(r*sin_r) - oy),
             C_WHITE, 2, cv2.LINE_AA)
    for deg in [-20, -10, 10, 20]:
        lw  = int(r * (0.38 if abs(deg) == 20 else 0.24))
        dpy = int((pitch - deg) * r / 45)
        rot = lambda sx, sy: (int(cx + sx*cos_r - sy*sin_r),
                               int(cy + sx*sin_r + sy*cos_r))
        cv2.line(canvas, rot(-lw, dpy), rot(0, dpy), (200,200,200), 1, cv2.LINE_AA)
        cv2.line(canvas, rot( lw, dpy), rot(0, dpy), (200,200,200), 1, cv2.LINE_AA)
    pr = r - 3
    px = int(cx + pr*math.sin(roll_r)); py = int(cy - pr*math.cos(roll_r))
    pts = np.array([[px, py],
                    [int(cx+(pr-10)*math.sin(roll_r)-6*math.cos(roll_r)),
                     int(cy-(pr-10)*math.cos(roll_r)-6*math.sin(roll_r))],
                    [int(cx+(pr-10)*math.sin(roll_r)+6*math.cos(roll_r)),
                     int(cy-(pr-10)*math.cos(roll_r)+6*math.sin(roll_r))]], np.int32)
    cv2.fillPoly(canvas, [pts], C_AMBER)
    cv2.line(canvas, (cx-24, cy), (cx-8, cy),  C_BLUE, 3, cv2.LINE_AA)
    cv2.line(canvas, (cx+8,  cy), (cx+24, cy), C_BLUE, 3, cv2.LINE_AA)
    cv2.circle(canvas, (cx, cy), 4, C_BLUE, -1)
    cv2.circle(canvas, (cx, cy), r, C_GRAY, 1)

# ── Compass strip ─────────────────────────────────────────────────────────────
def draw_compass(img, x, y, w, yaw):
    ch = 32
    cv2.rectangle(img, (x, y), (x+w, y+ch), (40,40,45), -1)
    cv2.rectangle(img, (x, y), (x+w, y+ch), C_GRAY, 1)
    cx_mid = x + w // 2
    for label, deg in [("N",0),("NE",45),("E",90),("SE",135),
                        ("S",180),("SW",225),("W",270),("NW",315)]:
        diff = (deg - yaw + 540) % 360 - 180
        px   = int(cx_mid + diff * w / 120)
        if x < px < x + w:
            scale = 0.55 if len(label) == 1 else 0.42
            col   = C_RED if label == "N" else C_WHITE
            tw, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0]
            put(img, label, (px - tw//2, y+20), col, scale)
    cv2.line(img, (cx_mid, y), (cx_mid, y+ch), C_AMBER, 2)

# ── Main app ──────────────────────────────────────────────────────────────────
class TelloCV:
    def __init__(self):
        self.drone     = None
        self.sim       = not TELLO_OK
        self.connected = False
        self.flying    = False
        self.running   = True
        self.telem     = dict(battery=0, height=0, spd_x=0.0, spd_y=0.0, spd_z=0.0,
                              pitch=0.0, roll=0.0, yaw=0.0, tof=0, temp=0, ftime=0)
        self.rc        = [0, 0, 0, 0]   # lr, fb, ud, yaw
        self.log_lines = []
        self.frame_q   = queue.Queue(maxsize=2)
        # held-key state — updated by _handle_key
        self._keys     = set()

    def log(self, msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        print(line)
        self.log_lines.append(line)
        if len(self.log_lines) > 8:
            self.log_lines.pop(0)

    # ── drone ─────────────────────────────────────────────────────────────────
    def connect(self):
        if self.connected:
            return
        self.log("Connecting...")
        try:
            self.drone = SimDrone() if self.sim else Tello()
            self.drone.connect()
            self.connected = True
            self.log("Connected" + (" [SIM]" if self.sim else " [LIVE]"))
            if not self.sim:
                self.drone.streamon()
            threading.Thread(target=self._telem_loop, daemon=True).start()
            threading.Thread(target=self._video_loop, daemon=True).start()
        except Exception as e:
            self.log(f"Connect failed: {e}")

    def takeoff(self):
        if not self.connected or self.flying:
            return
        self.log("Takeoff")
        def _do():
            try:
                self.drone.takeoff()
                self.flying = True
                self.log("Airborne")
            except Exception as e:
                self.log(f"Takeoff error: {e}")
        threading.Thread(target=_do, daemon=True).start()

    def land(self):
        if not self.connected or not self.flying:
            return
        self.log("Landing")
        def _do():
            try:
                self.drone.land()
                self.flying = False
                self.log("Landed")
            except Exception as e:
                self.log(f"Land error: {e}")
        threading.Thread(target=_do, daemon=True).start()

    def emergency(self):
        self.log("EMERGENCY STOP")
        self.flying = False
        try:
            self.drone.emergency()
        except Exception:
            pass

    # ── background threads ────────────────────────────────────────────────────
    def _telem_loop(self):
        while self.running and self.connected:
            try:
                d = self.drone
                self.telem.update(
                    battery=d.get_battery(),   height=d.get_height(),
                    spd_x=d.get_speed_x(),     spd_y=d.get_speed_y(),
                    spd_z=d.get_speed_z(),     pitch=d.get_pitch(),
                    roll=d.get_roll(),          yaw=d.get_yaw(),
                    tof=d.get_distance_tof(),  temp=d.get_temperature(),
                    ftime=d.get_flight_time()
                )
            except Exception:
                pass
            time.sleep(0.15)

    def _video_loop(self):
        if self.sim:
            self._sim_video()
            return
        fr = self.drone.get_frame_read()
        while self.running:
            f = fr.frame
            if f is not None:
                try:
                    self.frame_q.put_nowait(f)
                except queue.Full:
                    pass
            time.sleep(1/FPS)

    def _sim_video(self):
        while self.running:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            draw_horizon(frame, 320, 240, 220,
                         self.telem["pitch"], self.telem["roll"])
            try:
                self.frame_q.put_nowait(frame)
            except queue.Full:
                pass
            time.sleep(1/FPS)

    # ── keyboard → RC ─────────────────────────────────────────────────────────
    def _handle_key(self, key):
        """Process one cv2.waitKey result. Returns False to quit."""
        if key == 255 or key == -1:   # no key
            return True
        k = key & 0xFF
        if k == 27:                   # ESC
            return False
        if k == ord('c') or k == ord('C'):
            threading.Thread(target=self.connect, daemon=True).start()
        elif k == ord('t') or k == ord('T'):
            self.takeoff()
        elif k == ord('l') or k == ord('L'):
            self.land()
        elif k == ord('x') or k == ord('X'):
            self.emergency()
        return True

    def _rc_from_keys(self, key):
        """
        cv2.waitKey only fires once per press, so we maintain a held-key set.
        Keys toggle in on press (detected here via raw key), and we send RC
        every frame based on what's currently in the set.
        Movement keys are momentary — held means continuous RC, released → 0.
        We approximate hold by resending each frame and zeroing when not seen.
        """
        # Since cv2 has no key-up event, we use a simpler approach:
        # send the RC value for this frame based on current key, decay otherwise.
        k = key & 0xFF if key not in (255, -1) else None
        lr = fb = ud = yaw = 0
        if k == ord('d'): lr  =  RC_SPEED
        if k == ord('a'): lr  = -RC_SPEED
        if k == ord('w'): fb  =  RC_SPEED
        if k == ord('s'): fb  = -RC_SPEED
        if k == ord('r'): ud  =  RC_SPEED
        if k == ord('f'): ud  = -RC_SPEED
        if k == ord('e'): yaw =  RC_SPEED
        if k == ord('q'): yaw = -RC_SPEED
        self.rc = [lr, fb, ud, yaw]

    def _apply_rc(self):
        if not self.connected or not self.flying:
            return
        try:
            self.drone.send_rc_control(*self.rc)
        except Exception:
            pass

    # ── render ────────────────────────────────────────────────────────────────
    def _render(self, base_frame):
        t  = self.telem
        lr, fb, ud, yaw_rc = self.rc

        canvas = cv2.resize(base_frame, (W, H)) if base_frame is not None \
                 else np.zeros((H, W, 3), dtype=np.uint8)

        # top bar
        panel(canvas, 0, 0, W, 44)
        mode   = "SIM" if self.sim else "LIVE"
        status = "FLYING" if self.flying else ("CONNECTED" if self.connected else "DISCONNECTED")
        s_col  = C_GREEN if self.flying else (C_BLUE if self.connected else C_RED)
        put(canvas, f"TELLO  |  {mode}  |  {status}", (12, 28), s_col, 0.65, 2)
        batt  = int(t["battery"])
        b_col = C_GREEN if batt > 30 else C_RED
        put(canvas, f"BAT {batt:3d}%", (W-150, 28), b_col, 0.65, 2)

        # compass
        draw_compass(canvas, 200, 48, W-400, t["yaw"])

        # left panel — telemetry
        px, py, pw = 10, 90, 190
        panel(canvas, px, py, pw, 390)
        put(canvas, "TELEMETRY", (px+8, py+18), C_GRAY, 0.45)
        metrics = [
            ("HEIGHT",  f'{int(t["height"]):4d} cm',     C_BLUE),
            ("SPEED X", f'{t["spd_x"]:+6.1f} cm/s',     C_GREEN),
            ("SPEED Y", f'{t["spd_y"]:+6.1f} cm/s',     C_GREEN),
            ("SPEED Z", f'{t["spd_z"]:+6.1f} cm/s',     C_GREEN),
            ("PITCH",   f'{t["pitch"]:+6.1f} deg',       C_AMBER),
            ("ROLL",    f'{t["roll"]:+6.1f} deg',        C_AMBER),
            ("YAW",     f'{t["yaw"]:6.1f} deg',          C_AMBER),
            ("TOF",     f'{int(t["tof"]):4d} mm',        C_WHITE),
            ("TEMP",    f'{int(t["temp"]):4d} C',        C_WHITE),
            ("F-TIME",  f'{int(t["ftime"]):4d} s',       C_GRAY),
        ]
        for i, (label, val, col) in enumerate(metrics):
            ry = py + 36 + i*34
            put(canvas, label, (px+8, ry),    C_GRAY, 0.40)
            put(canvas, val,   (px+8, ry+16), col,    0.52, 1)

        # right panel — ADI + RC bars
        rp_x = W-210; rp_y = 90
        panel(canvas, rp_x, rp_y, 200, 390)
        put(canvas, "ATTITUDE", (rp_x+8, rp_y+18), C_GRAY, 0.45)
        adi_cx = rp_x+100; adi_cy = rp_y+115
        draw_horizon(canvas, adi_cx, adi_cy, 90, t["pitch"], t["roll"])
        put(canvas, f'{t["yaw"]:.0f} deg', (adi_cx-24, adi_cy+108), C_AMBER, 0.5, 1)

        put(canvas, "RC INPUT", (rp_x+8, rp_y+230), C_GRAY, 0.40)
        for i, (lbl, val, col) in enumerate([
                ("LR",  lr,     C_BLUE),
                ("FB",  fb,     C_GREEN),
                ("UD",  ud,     C_AMBER),
                ("YAW", yaw_rc, (180,100,220))]):
            ry2 = rp_y+248+i*34
            put(canvas, lbl,      (rp_x+8,   ry2+12), C_GRAY, 0.38)
            vbar(canvas, rp_x+130, ry2, 28, val, col)
            put(canvas, str(val), (rp_x+152, ry2+18), col,    0.42)

        # bottom cheatsheet
        cy_bot = H-72
        panel(canvas, 210, cy_bot, W-420, 66)
        put(canvas, "C=connect   T=takeoff   L=land   X=emergency   ESC=quit",
            (220, cy_bot+20), C_GRAY, 0.42)
        put(canvas, "W/S=fwd/back   A/D=left/right   R/F=up/down   Q/E=yaw",
            (220, cy_bot+44), C_GRAY, 0.42)

        # log
        n = len(self.log_lines)
        if n:
            log_top = cy_bot - n*17 - 6
            panel(canvas, 210, log_top, W-420, n*17+6, alpha=0.45)
            for i, line in enumerate(self.log_lines):
                put(canvas, line, (218, log_top+i*17+14), C_GRAY, 0.38)

        return canvas

    # ── main loop ─────────────────────────────────────────────────────────────
    def run(self):
        cv2.namedWindow("Tello", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Tello", W, H)
        self.log("Press C to connect  |  T=takeoff  L=land  ESC=quit")
        last_frame = None

        while self.running:
            key = cv2.waitKey(33) & 0xFFFF   # ~30 fps

            if not self._handle_key(key):
                break

            self._rc_from_keys(key)
            self._apply_rc()

            try:
                last_frame = self.frame_q.get_nowait()
            except queue.Empty:
                pass

            cv2.imshow("Tello", self._render(last_frame))

        # cleanup
        self.running = False
        if self.flying:
            try: self.drone.land()
            except Exception: pass
        if self.connected and not self.sim:
            try: self.drone.streamoff()
            except Exception: pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    TelloCV().run()
