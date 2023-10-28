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
times = [(21, 45),
         (21, 43),
         (21, 44),
         (21, 50),
         (21, 44),
         (21, 50),
         (26, 50)
         ]

# File path to rosbag
paths = ['/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-11_43_42-success2',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-11_52_34-success3',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-16_53_32-success5',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-17_01_14-success6',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-17_04_53-success7',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-17_09_21-success8',
         '/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-17_23_12-success9'
         ]

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
############## GT Plot  ############################
####################################################
fig, axs = plt.subplots()
#axs.set_aspect('equal')
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
axs.set_xlim([-2.4, 0.2])
axs.set_ylim([-0.4, 1.8])
#axs.set_facecolor('xkcd:light grey')

##############################################################
############## Load all the data #############################
##############################################################

err = 0
num = 0


# create reader instance and open for reading
for idx, path in enumerate(paths):
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
    wall_pos =  np.mean(wall, axis=0) + np.array([0.03, 0, 0])
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

    wall_angle = np.mean(wall_yaw) + np.pi/2


    ##############################################
    ###### Wall Length Definitions ###############
    #                      l                     #
    #               ________________             #
    #               |__    __    __| d           #
    #                  \__/  \__/    d_b         #
    #              off2  a mid b off1            #
    #                                            #
    ##############################################

    # Only add the first wall
    if idx == 0: 
        #wall_angle = 0.0 * np.pi/180
        d = 0.25
        l = 2.4
        off1 = 0.15
        off2 = 0.29
        mid = 0.4
        a = 0.35
        b = 0.08
        d_b1 = 0.09
        d_b2 = 0.09
        assert(np.abs(l - (off1 + off2 + mid + 4*a + 2*b)) < 0.001)

        corners = np.stack(([-l, 0],
                            [-l, d],
                            [0, d],
                            [0, 0],
                            [-off1, 0],
                            [-off1-a, -d_b1],
                            [-off1-a-b, -d_b1],
                            [-off1-2*a-b, 0],
                            [-off1-2*a-b-mid, 0],
                            [-off1-3*a-b-mid, -d_b2],
                            [-off1-3*a-2*b-mid, -d_b2],
                            [-off1-4*a-2*b-mid, 0.0]
                            ))
        rot = np.array([[np.cos(wall_angle), -np.sin(wall_angle)],
                        [np.sin(wall_angle),  np.cos(wall_angle)]])
        corners = np.array([rot @ corners[i,:] for i in range(len(corners))]) + np.array([0.1, wall_pos[0]])
        
        def wall_fun(x):
            x = np.array(x)
            result = np.zeros_like(x)
            result[x < corners[11,0]] = corners[11,1]
            result[x >= corners[11,0]] =  (x[x >= corners[11,0]]*(corners[10,1] - corners[11,1])/(corners[10,0] - corners[11,0]) 
                                                + (corners[10,1] - (corners[10,1] - corners[11,1])/(corners[10,0] - corners[11,0]) * corners[10,0]))
            result[x >= corners[10,0]] =  corners[10,1]
            result[x >= corners[9,0]] =  (x[x >= corners[9,0]]*(corners[8,1] - corners[9,1])/(corners[8,0] - corners[9,0]) 
                                                + (corners[8,1] - (corners[8,1] - corners[9,1])/(corners[8,0] - corners[9,0]) * corners[8,0]))
            result[x >= corners[8,0]] =  corners[8,1]
            result[x >= corners[7,0]] =  (x[x >= corners[7,0]]*(corners[6,1] - corners[7,1])/(corners[6,0] - corners[7,0]) 
                                                + (corners[6,1] - (corners[6,1] - corners[7,1])/(corners[6,0] - corners[7,0]) * corners[6,0]))
            result[x >= corners[6,0]] =  corners[6,1]
            result[x >= corners[5,0]] =  (x[x >= corners[5,0]]*(corners[4,1] - corners[5,1])/(corners[4,0] - corners[5,0]) 
                                                + (corners[4,1] - (corners[4,1] - corners[5,1])/(corners[4,0] - corners[5,0]) * corners[4,0]))
            result[x >= corners[4,0]] =  corners[4,1]
           
            return result 
            

        axs.plot(np.arange(-3, 1, 0.01), wall_fun(np.arange(-3, 1, 0.01)))
        


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
    axs.plot(ee[:, 0], ee[:, 1], color="black", alpha=0.2, zorder=10, lw=0.7*lw)

    ##########################################################
    # Compute the normal distance to the wall and accumulate #
    ##########################################################
    wall = wall_fun(ee[:, 0])
    for k in range(len(ee[:,0])):
        err += np.linalg.norm(ee[k,:2] - np.array([ee[k,0], wall[k]]))
        x = ee[k, 0] 
        y = wall[k] 
        axs.plot([ee[k,0], x], [ee[k,1], y], color="red")
        
        num += 1

print(f"Mean orthogonal error to wall {err / num:.02} over {len(paths)} trajectories and a total of {num} data points")

# Save the Figures
fig.set_size_inches(size)

plt.show()