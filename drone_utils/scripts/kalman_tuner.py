#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import rospy

import seaborn as sns

sns.set()

from plot_utils import convert_state_to_array, convert_control_to_array, NP, NX, NU, var_indexes, plot_history, read_state_history

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') +"/"+ rospy.get_param('/log_file'))

time_init = 0
t_end = 0
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if time_init == 0:
        time_init = msg.header.stamp.to_sec()
    t_end = msg.header.stamp.to_sec()

kalman_state_history = read_state_history(bag, '/drone_state', time_init, t_end)

# time_init = -20
# t_end = 1

fig_kalman, axe_kalman = plt.subplots(7, 3, figsize=(20, 10))

plot_ranges = {
    "t": [0, t_end-time_init],
    "x": [-3, 3],
    "y": [-3, 3],
    "z": [-1, 2],
    "dx": [-10, 10],
    "dy": [-3, 3],
    "dz": [-1.5, 5],
    "yaw (x)": [-15, 15],
    "pitch (y)": [-15, 15],
    "roll (z)": [-90, 90],
    "dyaw (x)": [-4, 4],
    "dpitch (y)": [-4, 4],
    "droll (z)": [-2, 2],

    "thrust_scaling": [0, 1.5],
    "torque_scaling": [0, 1.5],
    "servo1_offset": [-0.3, 0.3],
    "servo2_offset": [-1, 1],
    "fx": [-3, 3],
    "fy": [-3, 3],
    "fz": [-3, 3],
    "mx": [-1, 1],
    "my": [-1, 1],
    "mz": [-0.1, 0.1],
}

kalman_plot_indexes = {
    (0, 0): ("t", "x"),
    (0, 1): ("t", "y"),
    (0, 2): ("t", "z"),

    (1, 0): ("t", "yaw (x)"),
    (1, 1): ("t", "pitch (y)"),
    (1, 2): ("t", "roll (z)"),

    (2, 0): ("t", "dx"),
    (2, 1): ("t", "dy"),
    (2, 2): ("t", "dz"),

    (3, 0): ("t", "dyaw (x)"),
    (3, 1): ("t", "dpitch (y)"),
    (3, 2): ("t", "droll (z)"),

    (4, 0): ("t", "thrust_scaling"),
    (4, 1): ("t", "servo1_offset"),
    (4, 2): ("t", "servo2_offset"),

    (5, 0): ("t", "mx"),
    (5, 1): ("t", "my"),
    (5, 2): ("t", "mz"),

    (6, 0): ("t", "fx"),
    (6, 1): ("t", "fy"),
    (6, 2): ("t", "fz"),
}

Q_names = ['x', 'x', 'x',
           'dx', 'dx', 'dx',
           'att', 'att', 'att', 'att',
           'datt', 'datt', 'datt',
           'thrust_scaling',
           'torque_scaling',
           'servo_offset', 'servo_offset',
           'disturbance_force', 'disturbance_force', 'disturbance_force_z',
           'disturbance_torque', 'disturbance_torque', 'disturbance_torque_z']

R_names = ['optitrack_x', 'optitrack_x', 'optitrack_x',
           'optitrack_att', 'optitrack_att', 'optitrack_att', 'optitrack_att']

# set plot ranges
for plot_idx, (x_name, y_name) in kalman_plot_indexes.items():
    axe_kalman[plot_idx].axis(xmin=plot_ranges[x_name][0],
                              xmax=plot_ranges[x_name][1],
                              ymin=plot_ranges[y_name][0],
                              ymax=plot_ranges[y_name][1])
    axe_kalman[plot_idx].set_xlabel(x_name)
    axe_kalman[plot_idx].set_ylabel(y_name)

try:
    from drone_gnc.srv import KalmanSimu
    from drone_gnc.msg import DroneTrajectory

    kalman_simu = rospy.ServiceProxy('/kalman_simu', KalmanSimu)
    kalman_line_list = plot_history(kalman_state_history, kalman_plot_indexes, axe_kalman, "kalman_state")

    # optitrack_state_history = read_state_history(bag, '/optitrack_client/Kite/optitrack_pose', time_init, t_end)
    # plot_history(optitrack_state_history, kalman_plot_indexes, axe_kalman, "optitrack")

    simu_state_history = read_state_history(bag, '/simu_drone_state', time_init, t_end)
    plot_history(simu_state_history, kalman_plot_indexes, axe_kalman, "simu")

    # plot "ground truth"
    Q = [1, 1, 1, 2000, 2000, 2000, 1, 1, 1, 1, 4000, 4000, 4000] + [0 for i in range(NP)]
    R = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    resp = kalman_simu(Q, R, False)
    ground_truth_state_history = np.empty((0, NX + NP + 1))
    for waypoint in resp.trajectory.trajectory:
        state_array = np.append(waypoint.state.header.stamp.to_sec() - time_init, convert_state_to_array(waypoint.state))
        ground_truth_state_history = np.vstack((ground_truth_state_history, state_array))
    plot_history(ground_truth_state_history, kalman_plot_indexes, axe_kalman, "ground_truth", 'r')

    # create sliders figure
    plt.figure(figsize=(9, 6))

    initial_Q = rospy.get_param('/navigation/predict_vars')
    initial_R = rospy.get_param('/navigation/update_vars')

    param_sliders = {}
    i = 0
    for var_name in set(Q_names): # convert to a set to get unique values
        ax_slider = plt.axes([0.25, 0.8 - i * 0.05, 0.65, 0.04], facecolor='white')
        param_sliders[var_name] = Slider(ax_slider, var_name, 0, max(initial_Q[var_name]*10, 100), valinit=initial_Q[var_name], valstep=0.001)
        i += 1
    for var_name in set(R_names): # convert to a set to get unique values
        ax_slider = plt.axes([0.25, 0.8 - i * 0.05, 0.65, 0.04], facecolor='white')
        param_sliders[var_name] = Slider(ax_slider, var_name, 0, max(initial_R[var_name]*10, 100), valinit=initial_R[var_name], valstep=0.001)
        i += 1

    def update_kalman(val):
        Q = [param_sliders[var_name].val for var_name in Q_names]
        R = [param_sliders[var_name].val for var_name in R_names]
        resp = kalman_simu(Q, R, True)

        new_kalman_state_history = np.empty((0, NX + NP + 1))
        for waypoint in resp.trajectory.trajectory:
            state_array = np.append(waypoint.state.header.stamp.to_sec() - time_init, convert_state_to_array(waypoint.state))
            new_kalman_state_history = np.vstack((new_kalman_state_history, state_array))

        for i in range(len(kalman_line_list)):
            l, plot_idx = kalman_line_list[i]
            x_name, y_name = kalman_plot_indexes[plot_idx]
            l.set_ydata(new_kalman_state_history[:, var_indexes[y_name]])
            l.set_xdata(new_kalman_state_history[:, var_indexes[x_name]])
        fig_kalman.canvas.draw_idle()


    for var_name, slider in param_sliders.items():
        slider.on_changed(update_kalman)

except rospy.ServiceException as e:
    print("c++ kalman_tuner node is not running")

plt.show()
