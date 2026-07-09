# -*- coding: utf-8 -*-
"""
Mixed swarm shape drawing:
  - Crazyflie draws a FIGURE-8 using the high-level commander + Poly4D
    trajectory (precise, onboard-executed) at HIGH altitude.
  - Tello draws a RECTANGLE using relative move commands at LOW altitude.

Both take off together, draw their shapes in parallel, and land smoothly.
Sync barriers only at takeoff-ready and landing (their control paradigms
are too different to lockstep mid-shape).

REQUIREMENTS:  pip install cflib djitellopy
  - PC on Tello WiFi + Crazyradio dongle plugged in
"""

import sys
import time
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from djitellopy import Tello

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# ---------------------------------------------------------------------------
# Figure-8 trajectory (from Bitcraze example) — 8th order polys per segment
# Duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7
# ---------------------------------------------------------------------------
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

# ---------------------------------------------------------------------------
# Altitudes
CF_FIG8_Z    = 1.5    # Crazyflie draws figure-8 HIGH
TELLO_RECT_Z = 1.0    # Tello draws rectangle LOW (relative)

# Tello rectangle dimensions (cm) and speed
RECT_W   = 100   # width  (cm)
RECT_H   = 60    # height/depth (cm)
RECT_SPD = 20    # cm/s — slow & smooth

# ---------------------------------------------------------------------------
barrier = threading.Barrier(2)
def sync(msg):
    print(f"[SYNC] {msg}")
    barrier.wait()

# ---------------------------------------------------------------------------
# Tello keepalive
# ---------------------------------------------------------------------------
tello_rc = {'lr': 0, 'fb': 0, 'ud': 0, 'yaw': 0}
tello_rc_lock = threading.Lock()
tello_alive = threading.Event(); tello_alive.set()

def tello_keepalive(t):
    while tello_alive.is_set():
        with tello_rc_lock:
            v = (tello_rc['lr'], tello_rc['fb'], tello_rc['ud'], tello_rc['yaw'])
        try:
            t.send_rc_control(*v)
        except Exception as e:
            print(f"[TELLO] keepalive error: {e}")
        time.sleep(0.1)

def set_rc(lr=0, fb=0, ud=0, yaw=0):
    with tello_rc_lock:
        tello_rc.update(lr=lr, fb=fb, ud=ud, yaw=yaw)

def move_for(seconds, lr=0, fb=0, ud=0, yaw=0):
    set_rc(lr, fb, ud, yaw); time.sleep(seconds); set_rc(0, 0, 0, 0)


# ===========================================================================
# CRAZYFLIE — figure 8 via high-level commander
# ===========================================================================
def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []
    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration
    if not trajectory_mem.write_data_sync():
        print('[CF] Trajectory upload failed!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(
        trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def crazyflie_routine(ready_evt):
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1

        print("[CF] Uploading figure-8 trajectory...")
        duration = upload_trajectory(cf, trajectory_id, figure8)
        print(f"[CF] Trajectory is {duration:.1f}s long")

        print("[CF] Resetting estimator...")
        reset_estimator(scf)

        hlc = cf.high_level_commander
        cf.supervisor.send_arming_request(True)
        time.sleep(1.0)

        # Signal Tello the slow setup is done
        ready_evt.set()

        print(f"[CF] Takeoff to {CF_FIG8_Z}m")
        hlc.takeoff(CF_FIG8_Z, 2.0)
        time.sleep(3.0)
        sync("takeoff done")

        print("[CF] Drawing figure-8...")
        hlc.start_trajectory(trajectory_id, 1.0, relative_position=True)
        time.sleep(duration)

        print("[CF] Smooth landing...")
        hlc.land(0.0, 3.0)   # land over 3s for smoothness
        time.sleep(3.0)
        hlc.stop()
        sync("landed")


# ===========================================================================
# TELLO — rectangle via relative moves
# ===========================================================================
def tello_routine(ready_evt):
    t = Tello()
    t.connect()
    print(f"[TELLO] Battery: {t.get_battery()}%")

    print("[TELLO] Waiting for CF setup...")
    ready_evt.wait()

    print("[TELLO] Takeoff")
    t.takeoff()
    ka = threading.Thread(target=tello_keepalive, args=(t,), daemon=True)
    ka.start()

    # settle near rectangle altitude
    move_for(0.7, ud=15)
    sync("takeoff done")

    # Draw rectangle: forward -> right -> back -> left
    print("[TELLO] Drawing rectangle...")
    w_time = RECT_W / RECT_SPD    # 100/20 = 5s
    h_time = RECT_H / RECT_SPD    # 60/20  = 3s

    print("  edge 1: forward")
    move_for(h_time, fb=RECT_SPD)
    time.sleep(0.5)
    print("  edge 2: right")
    move_for(w_time, lr=RECT_SPD)
    time.sleep(0.5)
    print("  edge 3: back")
    move_for(h_time, fb=-RECT_SPD)
    time.sleep(0.5)
    print("  edge 4: left")
    move_for(w_time, lr=-RECT_SPD)
    time.sleep(0.5)
    print("[TELLO] Rectangle complete")

    # smooth landing
    print("[TELLO] Landing")
    set_rc(0, 0, 0, 0); time.sleep(0.3)
    tello_alive.clear(); time.sleep(0.3)
    try:
        t.land()
    except Exception as e:
        print(f"[TELLO] land error: {e}")
    sync("landed")
    t.end()


# ===========================================================================
if __name__ == '__main__':
    print("=" * 58)
    print("  MIXED SWARM SHAPE DRAWING")
    print(f"  Crazyflie: FIGURE-8   @ {CF_FIG8_Z}m  (high, precise HLC)")
    print(f"  Tello:     RECTANGLE  @ ~{TELLO_RECT_Z}m (low, open-loop)")
    print("  PC on Tello WiFi + Crazyradio plugged in")
    print("=" * 58)
    input("  Press ENTER when both drones are powered and ready...")

    cf_ready = threading.Event()
    cf_thread    = threading.Thread(target=crazyflie_routine, args=(cf_ready,))
    tello_thread = threading.Thread(target=tello_routine, args=(cf_ready,))

    cf_thread.start(); tello_thread.start()
    cf_thread.join();  tello_thread.join()

    print("\n[DONE] Both drones landed.")
