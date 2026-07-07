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

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
TEST_DURATION = 60      # 1 minute
LOG_PERIOD_MS = 100     # 10 Hz

logging.basicConfig(level=logging.ERROR)

# ---------------------------------------------------------------------------
# Shared buffers
# ---------------------------------------------------------------------------
lock   = threading.Lock()
t_buf  = []
x_buf  = []
y_buf  = []
z_buf  = []
ro_buf = []
pi_buf = []
ya_buf = []
t0     = None


def data_callback(timestamp, data, logconf):
    global t0
    now = time.time()
    with lock:
        if t0 is None:
            t0 = now
        t_buf.append(now - t0)
        x_buf.append(data['stateEstimate.x'])
        y_buf.append(data['stateEstimate.y'])
        z_buf.append(data['stateEstimate.z'])
        ro_buf.append(data['stabilizer.roll'])
        pi_buf.append(data['stabilizer.pitch'])
        ya_buf.append(data['stabilizer.yaw'])


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    session_label = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_filename  = f"crazyflie_stationary_{session_label}.csv"

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Single log block — 6 variables at 10 Hz
        lc = LogConfig(name='Stationary', period_in_ms=LOG_PERIOD_MS)
        lc.add_variable('stateEstimate.x',  'float')
        lc.add_variable('stateEstimate.y',  'float')
        lc.add_variable('stateEstimate.z',  'float')
        lc.add_variable('stabilizer.roll',  'float')
        lc.add_variable('stabilizer.pitch', 'float')
        lc.add_variable('stabilizer.yaw',   'float')
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(data_callback)

        print(f"{'='*55}")
        print(f"  Stationary acquisition  —  {TEST_DURATION}s  @  {1000//LOG_PERIOD_MS} Hz")
        print(f"  Session: {session_label}")
        print(f"{'='*55}")
        print(f"  {'t(s)':>5}  {'pkts':>5}  {'X':>7}  {'Y':>7}  {'Z':>7}  {'Roll':>7}  {'Pitch':>7}  {'Yaw':>7}")
        print(f"  {'-'*63}")

        lc.start()

        for i in range(TEST_DURATION):
            time.sleep(1)
            with lock:
                n  = len(t_buf)
                if n > 0:
                    lx = x_buf[-1]; ly = y_buf[-1]; lz = z_buf[-1]
                    lr = ro_buf[-1]; lp = pi_buf[-1]; lyw = ya_buf[-1]
                else:
                    lx=ly=lz=lr=lp=lyw=0.0
            print(f"  {i+1:>5}  {n:>5}  {lx:>7.3f}  {ly:>7.3f}  {lz:>7.3f}  {lr:>7.2f}  {lp:>7.2f}  {lyw:>7.2f}")

        lc.stop()

    # Snapshot
    with lock:
        t  = list(t_buf)
        x  = list(x_buf)
        y  = list(y_buf)
        z  = list(z_buf)
        ro = list(ro_buf)
        pi = list(pi_buf)
        ya = list(ya_buf)

    print(f"\n{'='*55}")
    print(f"  {len(t)} packets recorded.")

    # -- CSV -----------------------------------------------------------------
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg'])
        for row in zip(t, x, y, z, ro, pi, ya):
            writer.writerow(row)
    print(f"  CSV saved -> {csv_filename}")
    print(f"{'='*55}\n")

    # -- Plots (6 separate) --------------------------------------------------
    suffix = f"stationary  ({session_label})"

    datasets = [
        (t, x,  'X',     'X (m)',       'tab:blue'),
        (t, y,  'Y',     'Y (m)',       'tab:orange'),
        (t, z,  'Z',     'Z (m)',       'tab:green'),
        (t, ro, 'Roll',  'Roll (°)',    'tab:red'),
        (t, pi, 'Pitch', 'Pitch (°)',   'tab:purple'),
        (t, ya, 'Yaw',   'Yaw (°)',     'tab:brown'),
    ]

    for t_data, vals, name, ylabel, color in datasets:
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t_data, vals, color=color, linewidth=0.9)
        ax.set_title(f"{name}  —  {suffix}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

    plt.show()