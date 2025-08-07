# Zig‑Zag Timer‑Based Motion
import numpy as np
from math import pi
import sim_interface
import localization
import covar_mat_sub
import visualization
import time
import signal
import sys  # for sys.exit
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import update_eq_sub

# Global Parameters
T_ZIG = 5.0           # seconds per zig or zag
V_FORWARD = 0.3       # constant forward speed (m/s)
W_ZIG = 0.8           # constant turn rate for zig (rad/s)
W_ZAG = -0.5          # constant turn rate for zag (rad/s)

# Signal handler for clean exit
def signal_handler(sig, frame):
    print('Shutting down...')
    plt.close('all')
    sim_interface.setvel_pioneers1(0.0, 0.0)
    sim_interface.sim_shutdown()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Main loop
if __name__ == '__main__':
    visualization.plot_initialization()
    mat_cal = covar_mat_sub.matrix_calculator()
    update_eq = update_eq_sub.update_eq()
    odom_cal = localization.Odometry_calculation(mat_cal,update_eq)

    # Initialize simulator
    if not sim_interface.sim_init():
        print('Failed to connect to simulator'); sys.exit(1)
    sim_interface.get_handles()
    if not sim_interface.start_simulation():
        print('Failed to start simulation'); sys.exit(1)

    # Timing and state
    phase_start = time.time()
    start = phase_start
    zig_phase = True  # start with zig
    realpose = sim_interface.localize_robot1()
    prev_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now
            t = now - start  # Run time since last phase start

            # 1) Check phase duration
            if now - phase_start >= T_ZIG:
                zig_phase = not zig_phase
                phase_start = now

            # 2) Set commands based on phase
            v_cmd = V_FORWARD
            w_cmd = W_ZIG if zig_phase else W_ZAG

            # 3) Send to robot
            sim_interface.setvel_pioneers1(v_cmd, w_cmd)

            # 4) EKF prediction
            dl, dr = localization.encoder_output(v_cmd, w_cmd, t)
            tl = dl / localization.ticks_to_millimeter
            tr = dr / localization.ticks_to_millimeter
            odom_cal.update_encoder_tick([tl, tr])
            localization.prediction(odom_cal, realpose)

            # 5) EKF update
            realpose = sim_interface.localize_robot1()
            localization.update(odom_cal, realpose)

            # 6) Visualization
            x, y, theta = odom_cal.pose
            sim_interface.change_pioneer2_pose(x, y, theta)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        sim_interface.setvel_pioneers1(0.0, 0.0)
        sim_interface.sim_shutdown()
        plt.show(block=True)
    print('Program ended')
