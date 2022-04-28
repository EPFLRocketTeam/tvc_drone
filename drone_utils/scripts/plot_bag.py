#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import re
from plot_utils import plot_df_multiple, parse_bag
import seaborn as sns
import os
import rospkg
import argparse
import numpy as np

rospack = rospkg.RosPack()

USE_LATEX = False
if USE_LATEX:
    plt.rc('font', family='serif', serif='cm10')
    plt.rc('text', usetex=True)
    plt.rc('text.latex', preamble=r'\usepackage{gensymb}\usepackage{amsmath}')

sns.set()
sns.set_style("whitegrid")

state_plot_indexes = {
    (0, 0): [("t", "x")],
    (0, 1): [("t", "y")],
    (0, 2): [("t", "z")],

    (1, 0): [("t", "vx")],
    (1, 1): [("t", "vy")],
    (1, 2): [("t", "vz")],

    (2, 0): [("t", "yaw")],
    (2, 1): [("t", "pitch")],
    (2, 2): [("t", "roll")],

    (3, 0): [("t", "wx")],
    (3, 1): [("t", "wy")],
    (3, 2): [("t", "wz")],
}

control_plot_indexes = {
    (4, 0): [("t", "bottom"), ("t", "top")],
    (4, 1): [("t", "servo1")],
    (4, 2): [("t", "servo2")],
}

if __name__ == "__main__":
    default_bag_path = os.path.join(rospack.get_path('drone_utils'), 'log/log.bag')

    parser = argparse.ArgumentParser(description="Plot a ROS bag")
    parser.add_argument("rosbag_filepath", nargs='?', type=str, default=default_bag_path, help="rosbag file path")

    args = parser.parse_args()

    df_dict = parse_bag(args.rosbag_filepath, ["/drone_state", "/simu_drone_state", "/drone_control"])

    # radians to degrees
    for topic in ["/drone_state", "/simu_drone_state"]:
        df_dict[topic][["yaw", "pitch", "roll", "wx", "wy", "wz"]] *= 180 / np.pi
    df_dict["/drone_control"][["servo1", "servo2"]] *= 180 / np.pi

    # set initial times to 0
    t0 = min([df['t'][0] for df in df_dict.values()])
    for df in df_dict.values():
        df['t'] -= t0

    fig, axe = plt.subplots(5, 3, figsize=(15, 10))
    fig.subplots_adjust(wspace=0.4, hspace=0.5)

    plot_df_multiple(df_dict["/drone_state"], state_plot_indexes, axe, label=r"EKF state", use_latex=USE_LATEX)
    plot_df_multiple(df_dict["/simu_drone_state"], state_plot_indexes, axe, label=r"Simulation state",
                     use_latex=USE_LATEX)
    plot_df_multiple(df_dict["/drone_control"], control_plot_indexes, axe, label=r"Control", use_latex=USE_LATEX)

    axe[0][0].legend(loc='upper left')

    PLOT_FULL_STATE = False
    if PLOT_FULL_STATE:
        kalman_plot_indexes = {
            (0, 0): [("t", "vx")],
            (0, 1): [("t", "vy")],
            (0, 2): [("t", "vz")],

            (1, 0): [("t", "wx")],
            (1, 1): [("t", "wy")],
            (1, 2): [("t", "wz")],

            (2, 0): [("t", "thrust_scaling")],
            (2, 1): [("t", "servo1_offset")],
            (2, 2): [("t", "servo2_offset")],

            (3, 0): [("t", "disturbance_torque_x")],
            (3, 1): [("t", "disturbance_torque_y")],
            (3, 2): [("t", "disturbance_torque_z")],

            (4, 0): [("t", "disturbance_force_x")],
            (4, 1): [("t", "disturbance_force_y")],
            (4, 2): [("t", "disturbance_force_z")],
        }

        fig_kalman, axe_kalman = plt.subplots(5, 3, figsize=(20, 10))
        plot_df_multiple(df_dict["/drone_state"], kalman_plot_indexes, axe_kalman, label=r"EKF state",
                         use_latex=USE_LATEX)

        axe_kalman[0][0].legend(loc='upper left')

    plt.show()