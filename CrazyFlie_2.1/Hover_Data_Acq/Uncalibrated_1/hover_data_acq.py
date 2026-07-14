import logging
import time
import threading
import csv
from datetime import datetime

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
HOVER_Z       = 1.5
WARMUP_S      = 30      # estimator warm-up before arming

logging.basicConfig(level=logging.ERROR)

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
pos_lock = threading.Lock()
pos      = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

t_buf  = []; x_buf  = []; y_buf  = []; z_buf  = []
ro_buf = []; pi_buf = []; ya_buf = []
t0     = None
recording = False   # only store to buffers during flight


def data_callback(timestamp, data, logconf):
    global t0
    now = time.time()
    with pos_lock:
        pos['x']     = data['stateEstimate.x']
        pos['y']     = data['stateEstimate.y']
        pos['z']     = data['stateEstimate.z']
        pos['roll']  = data['stabilizer.roll']
        pos['pitch'] = data['stabilizer.pitch']
        pos['yaw']   = data['stabilizer.yaw']

        if recording:
            if t0 is None:
                t0 = now
            t_buf.append(now - t0)
            x_buf.append(pos['x']);     y_buf.append(pos['y'])
            z_buf.append(pos['z']);     ro_buf.append(pos['roll'])
            pi_buf.append(pos['pitch']); ya_buf.append(pos['yaw'])


def start_logging(cf):
    lc = LogConfig(name='HoverLog', period_in_ms=100)
    lc.add_variable('stateEstimate.x',  'float')
    lc.add_variable('stateEstimate.y',  'float')
    lc.add_variable('stateEstimate.z',  'float')
    lc.add_variable('stabilizer.roll',  'float')
    lc.add_variable('stabilizer.pitch', 'float')
    lc.add_variable('stabilizer.yaw',   'float')
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(data_callback)
    lc.start()
    return lc

# ---------------------------------------------------------------------------
# Keypress listener
# ---------------------------------------------------------------------------
HOVER_DURATION = 30     # seconds to hover before landing

# ---------------------------------------------------------------------------
# Flight helpers
# ---------------------------------------------------------------------------
def takeoff(cf, target_z, duration=4.0):
    print(f"Taking off to {target_z}m...")
    steps = int(duration / 0.1)
    vz = target_z / duration
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(0.1)


def hover(cf, x, y, z, duration=30.0):
    print(f"Hovering for {duration}s...")
    steps = int(duration / 0.1)
    for i in range(steps):
        cf.commander.send_position_setpoint(x, y, z, 0)
        time.sleep(0.1)
        if (i + 1) % 50 == 0:
            print(f"  {(i+1)//10}s elapsed...")


def smooth_land(cf, x, y, current_height):
    print("\nLanding...")
    for i in range(60):
        z = current_height + (0.4 - current_height) * (i / 60)
        cf.commander.send_position_setpoint(x, y, z, 0)
        time.sleep(0.1)
    for _ in range(15):
        cf.commander.send_position_setpoint(x, y, 0.4, 0)
        time.sleep(0.1)
    for i in range(50):
        z = 0.4 + (0.15 - 0.4) * (i / 50)
        cf.commander.send_position_setpoint(x, y, z, 0)
        time.sleep(0.1)
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)
    print("Landed.")

# ---------------------------------------------------------------------------
# Save CSV + plots
# ---------------------------------------------------------------------------
def save_csv(filename, init_pos, t, x, y, z, ro, pi, ya):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg'])
        # First row = initial position at t=0
        writer.writerow([0.0,
                         init_pos['x'], init_pos['y'], init_pos['z'],
                         init_pos['roll'], init_pos['pitch'], init_pos['yaw']])
        for row in zip(t, x, y, z, ro, pi, ya):
            writer.writerow(row)
    print(f"  CSV saved -> {filename}")


def show_plots(session_label, t, x, y, z, ro, pi, ya):
    suffix = f"hover flight  ({session_label})"
    datasets = [
        (x,  'X',     'X (m)',      'tab:blue'),
        (y,  'Y',     'Y (m)',      'tab:orange'),
        (z,  'Z',     'Z (m)',      'tab:green'),
        (ro, 'Roll',  'Roll (°)',   'tab:red'),
        (pi, 'Pitch', 'Pitch (°)',  'tab:purple'),
        (ya, 'Yaw',   'Yaw (°)',    'tab:brown'),
    ]
    for vals, name, ylabel, color in datasets:
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t, vals, color=color, linewidth=0.9)
        ax.set_title(f"{name}  —  {suffix}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
    plt.show()

# ---------------------------------------------------------------------------
if __name__ == '__main__':
    session_label = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_filename  = f"hover_{session_label}.csv"

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Resetting estimator...")
        reset_estimator(scf)
        cf = scf.cf

        lc = start_logging(cf)

        # -- Warm-up: let estimator converge for 30s --
        print(f"\n{'='*52}")
        print(f"  Estimator warm-up  —  {WARMUP_S}s  (keep drone still)")
        print(f"  {'t(s)':>5}  {'X':>7}  {'Y':>7}  {'Z':>7}  {'Roll':>6}  {'Pitch':>6}  {'Yaw':>7}")
        print(f"  {'-'*55}")
        for i in range(WARMUP_S):
            time.sleep(1)
            with pos_lock:
                p = pos.copy()
            print(f"  {i+1:>5}  {p['x']:>7.3f}  {p['y']:>7.3f}  {p['z']:>7.3f}  "
                  f"{p['roll']:>6.2f}  {p['pitch']:>6.2f}  {p['yaw']:>7.2f}")
        print(f"{'='*52}\n")

        # Snapshot initial position
        with pos_lock:
            init_pos = pos.copy()
        print(f"Initial position locked:  X={init_pos['x']:.4f}  Y={init_pos['y']:.4f}  Z={init_pos['z']:.4f}")

        # Start recording
        recording = True

        print("Arming...")
        cf.supervisor.send_arming_request(True)
        time.sleep(2.0)

        takeoff(cf, HOVER_Z, duration=4.0)

        with pos_lock:
            hold_x = pos['x']
            hold_y = pos['y']

        print(f"Holding at  X={hold_x:.3f}  Y={hold_y:.3f}  Z={HOVER_Z}\n")

        hover(cf, hold_x, hold_y, HOVER_Z, duration=HOVER_DURATION)

        # Stop recording before landing
        recording = False
        print("Hover complete — landing now (not recorded)...")

        smooth_land(cf, hold_x, hold_y, HOVER_Z)
        lc.stop()

    # Snapshot buffers
    t  = list(t_buf);  x  = list(x_buf);  y  = list(y_buf)
    z  = list(z_buf);  ro = list(ro_buf); pi = list(pi_buf); ya = list(ya_buf)

    print(f"\n{len(t)} packets recorded.")
    save_csv(csv_filename, init_pos, t, x, y, z, ro, pi, ya)
    show_plots(session_label, t, x, y, z, ro, pi, ya)

    print("Done.")