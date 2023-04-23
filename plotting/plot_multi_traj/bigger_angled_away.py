import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
import matplotlib.patheffects as pe
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import sys
# setting path
sys.path.append('../')

from plotting_settings import *
from robot import fk
from robot import rot_z

from angle_annotation import AngleAnnotation

# Function for guessing ros message tupe
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


def get_ellipse_dist(x, y, theta):
    return x*y/np.sqrt((y*np.sin(theta))**2 + (x*np.cos(theta))**2)


def get_normal_error(std, forward):
    forward = forward / np.linalg.norm(forward)
    dx = forward[0]
    dy = forward[1]
    angle = np.arctan2(dy, dx)

    return get_ellipse_dist(std[0], std[1], angle)



def draw_error_band(ax, x, y, err, **kwargs):
    # Calculate normals via centered finite differences (except the first point
    # which uses a forward difference and the last point which uses a backward
    # difference).
    dx = np.concatenate([[x[1] - x[0]], x[2:] - x[:-2], [x[-1] - x[-2]]])
    dy = np.concatenate([[y[1] - y[0]], y[2:] - y[:-2], [y[-1] - y[-2]]])
    l = np.hypot(dx, dy)
    nx = dy / l
    ny = -dx / l

    # end points of errors
    xp = x + nx * err
    yp = y + ny * err
    xn = x - nx * err
    yn = y - ny * err

    vertices = np.block([[xp, xn[::-1]],
                         [yp, yn[::-1]]]).T

    ax.fill(vertices[:,0], vertices[:,1], **kwargs)

# Install non-standard types
add_types = {}

for pathstr in [
    '/home/anton/Desktop/cloned_software/px4_ros_com_ros2/src/px4_msgs/msg/VehicleOdometry.msg'
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)



# The time interval we want to plot
times = [(18, 41), (37.5, 62.5), (18, 40)]

# File path to rosbag
paths = ['/home/anton/Desktop/rosbags/2023_04_06/angled_away/rosbag2-07_51_50-success6-bigger-angle',
         '/home/anton/Desktop/rosbags/2023_04_06/angled_away/rosbag2-07_56_10-success7',
         '/home/anton/Desktop/rosbags/2023_04_06/angled_away/rosbag2-07_59_51-success8']

# Topics to collect data from
topics=['/fmu/out/vehicle_odometry',
        '/joint_states',
        '/wall_pose']

####################################################
##### Init the Collection of Trajectories ##########
####################################################
ee_col = []
t_ee_col = []

####################################################
fig_t, axs_t = plt.subplots(3)

####################################################
############## GT Plot  ############################
####################################################
fig, axs = plt.subplots()
axs.set_aspect('equal')
axs.tick_params(axis='both', which='major', labelsize=0.8*text_size)
axs.xaxis.set_major_locator(MultipleLocator(1))
axs.xaxis.set_major_formatter('{x:.0f}')
axs.xaxis.set_minor_locator(MultipleLocator(0.2))

axs.yaxis.set_major_locator(MultipleLocator(1))
axs.yaxis.set_major_formatter('{x:.0f}')
axs.yaxis.set_minor_locator(MultipleLocator(0.2))

axs.spines['top'].set_visible(False)
axs.spines['right'].set_visible(False)
#axs.spines['bottom'].set_visible(False)
#axs.spines['left'].set_visible(False)

axs.set_xlabel("x[m]",fontsize=text_size)
axs.set_ylabel("y[m]",fontsize=text_size)
axs.set_xlim([-2, 0.5])
axs.set_ylim([-0.2, 2.3])
#axs.set_facecolor('xkcd:light grey')

##############################################################
############## Load all the data #############################
##############################################################

# create reader instance and open for reading
for idx, path in enumerate(paths[:6]):
    with Reader(path) as reader:

        # Init the arrays for plotting
        t_odom = []
        odom = []
        odom_q = []

        t_joint_state = []
        joint_state = []

        t_wall = []
        wall = []
        wall_q = []

        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            pass

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
 
            if connection.topic == topics[0]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                t_odom += [timestamp]
                odom += [msg.position]
                odom_q += [msg.q]

            if connection.topic == topics[1]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                t_joint_state += [timestamp]
                joint_state += [msg.position]
            
            if connection.topic == topics[2]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                t_wall += [timestamp]
                wall += [[msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z]]
                wall_q += [[msg.pose.orientation.w,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z]]


    # Make all the arrays np arrays
    t_odom = np.array(t_odom, dtype=float)
    odom = np.array(odom, dtype=float)
    odom_q = np.array(odom_q, dtype=float)

    t_joint_state = np.array(t_joint_state, dtype=float)
    joint_state = np.array(joint_state, dtype=float)

    t_wall = np.array(t_wall, dtype=float)
    wall = np.array(wall, dtype=float)
    wall_q = np.array(wall_q, dtype=float)

    #############################################################
    ################ Transformations ############################
    #############################################################

    # Transform from ned to enu
    T = np.array([[0,1,0],[1,0,0],[0,0,-1]])
    T_q = np.array([[1, 0, 0, 0],
                    [0, 0, 1, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, -1]])


    odom = np.array([T @ odom[i,:].transpose() for i in range(len(odom))])
    odom_q = np.array([T_q @ odom_q[i,:].transpose() for i in range(len(odom))])
    

    wall = np.array([T @ wall[i,:].transpose() for i in range(len(wall))])
    wall_q = np.array([T_q @ wall_q[i,:].transpose() for i in range(len(wall_q))])

    # Find the odometry points that most closely match the joint_state messages
    odom_subsample = np.array([odom[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
    odom_q_subsample = np.array([odom_q[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
    q0 = odom_q_subsample[:,0]
    q1 = odom_q_subsample[:,1]
    q2 = odom_q_subsample[:,2]
    q3 = odom_q_subsample[:,3] 
    yaw = np.arctan2(
            2 * ((q1 * q2) + (q0 * q3)),
            q0**2 + q1**2 - q2**2 - q3**2
        )


    # Compute the EE positions
    ee = np.array([fk(odom_subsample[i,:], odom_q_subsample[i,:], joint_state[i,:]) for i in range(len(t_joint_state))])

    # Find start time and shift time scale and turn it into seconds
    start_t = min(np.concatenate((t_odom, t_joint_state, t_wall)))
    t_odom = (t_odom - start_t) * 1e-9
    t_joint_state = (t_joint_state - start_t) * 1e-9
    t_wall = (t_wall - start_t) * 1e-9


    # Find the indeces for the specified time range
    odom_idx = ((np.abs(times[idx][0] - t_odom)).argmin(), (np.abs(times[idx][1] - t_odom)).argmin())
    joint_state_idx = ((np.abs(times[idx][0] - t_joint_state)).argmin(), (np.abs(times[idx][1] - t_joint_state)).argmin())
    wall_idx = ((np.abs(times[idx][0] - t_wall)).argmin(), (np.abs(times[idx][1] - t_wall)).argmin())

    # Cut down to indeces
    wall = wall[wall_idx[0]:wall_idx[1],:]
    wall_q = wall_q[wall_idx[0]:wall_idx[1],:]
    t_wall = t_wall[wall_idx[0]:wall_idx[1]]
    ee = ee[joint_state_idx[0]:joint_state_idx[1], :]
    yaw = yaw[joint_state_idx[0]:joint_state_idx[1]]
    t_ee = t_joint_state[joint_state_idx[0]: joint_state_idx[1]]

    ##############################################################
    ############## Synchronize the Trajectories ##################
    # For size we find the maximal change in y (moving forward)  #
    # to imply the beginning of the trajectory and move that to  #
    # t = 0                                                      #
    ##############################################################
    start_idx  = np.abs(np.diff(ee[:,1])).argmax()
    t_ee -= t_ee[start_idx]


    #############################################################
    ################ Wall Patch Definition ######################
    #############################################################
    wall_pos =  [1.66, 0]
    y_diff = wall_pos[0] - 1.5
    wall_pos[0] -= y_diff
    q0_wall = wall_q[:,0]
    q1_wall = wall_q[:,1]
    q2_wall = wall_q[:,2]
    q3_wall = wall_q[:,3] 
    wall_yaw = -np.arctan2(
            2 * ((q1_wall * q2_wall) + (q0_wall * q3_wall)),
            q0_wall**2 + q1_wall**2 - q2_wall**2 - q3_wall**2
        )

    wall_angle = np.mean(wall_yaw)
    wall_fun = lambda x: np.tan(wall_angle) * x + (wall_pos[0] - np.tan(wall_angle) * wall_pos[1])

    # Only add the first wall
    if idx == 0: 
        wall_angle = -20.1 * np.pi/180 
        
        wall_side=np.array([-1.9, 0.0])
        wall_up = np.array([0, 0.25])
        rot = np.array([[np.cos(wall_angle), -np.sin(wall_angle)],
                        [np.sin(wall_angle),  np.cos(wall_angle)]])
        wall_side = rot @ wall_side
        wall_up = rot @ wall_up

        wall_bottom_right = np.array([0.25, wall_fun(0.25)])
        wall_bottom_left = wall_bottom_right + wall_side
 
        wall_top_right = wall_bottom_right + wall_up
        wall_top_left = wall_bottom_right + wall_side + wall_up
        corners = np.stack((wall_bottom_left, wall_bottom_right,
                            wall_top_right, wall_top_left))
        axs.add_patch(Polygon(corners,
                              facecolor='xkcd:light grey', edgecolor='black', alpha=1, label='_nolegend_',
                              zorder=1))
        upper_bound = max(wall_top_right[1], wall_top_left[1])
        axs.plot(np.linspace(-10,10,2), np.ones(2) * upper_bound, color='black', lw=0.8*lw, linestyle='--', zorder=0)
       
        wall_str = "{\\mathrm{Wall}}"
        angle_points = np.stack((wall_top_left,
                                 wall_top_right,
                                 wall_top_left + np.array([.1, 0])))

        AngleAnnotation(xy=angle_points[0,:],
                        p1=angle_points[1,:],
                        p2=angle_points[2,:],
                        ax=axs,
                        size=2000,
                        lw=0.8*lw,
                        linestyle=":")
        text_position = np.mean(angle_points[1:,:],axis=0) + np.array([-.38, 0.2])
        axs.annotate(rf"$\psi_{wall_str} = {180/np.pi * wall_angle:.1f}^\circ$", text_position, fontsize=0.8*text_size)


    # Transform such that all walls are at the same spot
    ee[:, 1] -= y_diff


    #############################################################
    ############### Add to collection ###########################
    #############################################################
    ee_col += [ee[:, :3]]
    t_ee_col += [t_ee]

    ###########################################################
    ############# Plot the current data #######################
    ###########################################################
    axs.plot(ee[:, 0], ee[:, 1], color="black", alpha=0.4, zorder=20, lw=lw)

    ###########################################################
    axs_t[0].plot(t_ee, ee[:, 0], color="grey")
    axs_t[1].plot(t_ee, ee[:, 1], color="grey")
    axs_t[2].plot(t_ee, ee[:, 2], color="grey")


####################################################
########## Find the mean trajectory ################
####################################################
ts = np.concatenate([t for t in t_ee_col])
t_mean = np.arange(start=min(ts),
                   stop=max(ts),
                   step=1)

mean = []
std = []
for t in t_mean:
    points = []
    for i in range(len(paths[:6])):
        idx = np.abs(t - t_ee_col[i]).argmin()
        points += [ee_col[i][idx, :]]
    mean += [np.mean(points, axis=0)]
    std += [np.std(points, axis=0)]


mean = np.array(mean, dtype=float)
std = np.array(std, dtype=float)

axs_t[0].plot(t_mean, mean[:, 0], color="orange", label=r"$\mu_x$")
axs_t[1].plot(t_mean, mean[:, 1], color="orange", label=r"$\mu_y$")
axs_t[2].plot(t_mean, mean[:, 2], color="orange", label=r"$\mu_z$")
draw_error_band(axs_t[0], t_mean, mean[:,0], std[:,0], facecolor="orange", alpha=.3 ,label=r"$\sigma_x$")
draw_error_band(axs_t[1], t_mean, mean[:,1], std[:,1], facecolor="orange", alpha=.3,label=r"$\sigma_x$")
draw_error_band(axs_t[2], t_mean, mean[:,2], std[:,2], facecolor="orange", alpha=.3,label=r"$\sigma_x$")

axs.plot(mean[:, 0], mean[:,1], color=colors["ee"], label=r"$\mu_{EE}$", path_effects=[pe.Stroke(linewidth=lw, foreground='black'), pe.Normal()], zorder=21)

forward = np.concatenate(([[1, 0]], np.diff(mean[:,:2], axis=0)), axis=0)
err = [get_normal_error(std[i, :], forward[i,:]) for i in range(len(std))]
draw_error_band(axs, mean[:, 0], mean[:,1], err, facecolor=colors["ee"], edgecolor="none", alpha=.5, zorder=15, label=r"$\sigma_{EE}$")

# Set Legend
axs.legend(loc="lower left", prop={'size': text_size}, ncol=1, labelspacing=0.1, columnspacing=0.5)

axs_t[0].legend()
axs_t[1].legend()
axs_t[2].legend()


# Save the Figures
fig.set_size_inches(size)
fig.savefig("../plots/multi_groundtrack.png", dpi=dpi, bbox_inches='tight')

fig_t.set_size_inches(size)
fig_t.savefig("../plots/test.png", dpi=dpi, bbox_inches='tight')

