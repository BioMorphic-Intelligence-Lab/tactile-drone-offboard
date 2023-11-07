import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
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
times = [(27, 45), (27, 52.5),(27, 42.5),(27, 52.5),(27, 47.5)]

# File path to rosbag
paths = ['/home/anton/Desktop/rosbags/2023_04_28/superimposed_altitude/rosbag2-19_48_30-success1',
         '/home/anton/Desktop/rosbags/2023_04_28/superimposed_altitude/rosbag2-20_25_31-success2',
         '/home/anton/Desktop/rosbags/2023_04_28/superimposed_altitude/rosbag2-20_28_17-success3',
         '/home/anton/Desktop/rosbags/2023_04_28/superimposed_altitude/rosbag2-20_31_00-success4',
         '/home/anton/Desktop/rosbags/2023_04_28/superimposed_altitude/rosbag2-20_33_47-success5'
         ]

# Topics to collect data from
topics=['/fmu/out/vehicle_odometry',
        '/joint_states',
        '/wall_pose', 
        '/ee_reference',
        '/wrench']

####################################################
############## GT Plot  ############################
####################################################
fig, axs = plt.subplots()
axs.set_aspect('equal')
axs.tick_params(axis='both', which='major', labelsize=0.8*text_size)
axs.xaxis.set_major_locator(MultipleLocator(1))
axs.xaxis.set_major_formatter('{x:.0f}')
axs.xaxis.set_minor_locator(MultipleLocator(0.2))

#axs.yaxis.set_major_locator(MultipleLocator(1))
#axs.yaxis.set_major_formatter('{x:.0f}')
#axs.yaxis.set_minor_locator(MultipleLocator(0.2))

axs.spines['top'].set_visible(False)
axs.spines['right'].set_visible(False)
#axs.spines['bottom'].set_visible(False)
#axs.spines['left'].set_visible(False)

axs.set_xlabel("x[m]",fontsize=text_size)
axs.set_ylabel("y[m]",fontsize=text_size)
axs.set_xlim([-2.5, 0.5])
axs.set_ylim([0.5, 1.6])
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

        t_ee_ref = []
        ee_ref_q = []
        ee_ref = []

        t_wrench = []
        wrench = []

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

            if connection.topic == topics[3]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                t_ee_ref += [timestamp]
                ee_ref += [[msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]]
                ee_ref_q += [[msg.pose.orientation.w,
                              msg.pose.orientation.x,
                              msg.pose.orientation.y,
                              msg.pose.orientation.z]]
            if connection.topic == topics[4]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                t_wrench += [timestamp]
                wrench += [[msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]]


    # Make all the arrays np arrays
    t_odom = np.array(t_odom, dtype=float)
    odom = np.array(odom, dtype=float)
    odom_q = np.array(odom_q, dtype=float)

    t_joint_state = np.array(t_joint_state, dtype=float)
    joint_state = np.array(joint_state, dtype=float)

    t_wall = np.array(t_wall, dtype=float)
    wall = np.array(wall, dtype=float)
    wall_q = np.array(wall_q, dtype=float)

    t_ee_ref = np.array(t_ee_ref, dtype=float)
    ee_ref = np.array(ee_ref, dtype=float)
    ee_ref_q = np.array(ee_ref_q, dtype=float)

    t_wrench = np.array(t_wrench, dtype=float)
    wrench = np.array(wrench, dtype=float)

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

    # Find the reference yaws
    ee_ref_yaw = np.arctan2(2*((ee_ref_q[:,1]*ee_ref_q[:,2]) + (ee_ref_q[:,0]*ee_ref_q[:,3])),
                             ee_ref_q[:,0]**2 + ee_ref_q[:,1]**2 - ee_ref_q[:,2]**2 - ee_ref_q[:,3]**2)

    # Find start time and shift time scale and turn it into seconds
    start_t = min(np.concatenate((t_odom, t_joint_state, t_wall, t_ee_ref, t_wrench)))
    t_odom = (t_odom - start_t) * 1e-9
    t_joint_state = (t_joint_state - start_t) * 1e-9
    t_wall = (t_wall - start_t) * 1e-9
    t_ee_ref = (t_ee_ref - start_t) * 1e-9
    t_wrench = (t_wrench - start_t) * 1e-9


    # Find the odometry points that most closely match the joint_state messages
    odom_subsample = np.array([odom[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
    odom_q_subsample = np.array([odom_q[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
    # Compute the EE positions
    ee = np.array([fk(odom_subsample[i,:], odom_q_subsample[i,:], joint_state[i,:]) for i in range(len(t_joint_state))])

    # Find the indeces for the specified time range
    odom_idx  = ((np.abs(times[idx][0] - t_odom)).argmin(), (np.abs(times[idx][1] - t_odom)).argmin())
    joint_state_idx = ((np.abs(times[idx][0] - t_joint_state)).argmin(), (np.abs(times[idx][1] - t_joint_state)).argmin())
    wall_idx   = ((np.abs(times[idx][0] - t_wall)).argmin(), (np.abs(times[idx][1] - t_wall)).argmin())
    ee_ref_idx = ((np.abs(times[idx][0] - t_ee_ref)).argmin(), (np.abs(times[idx][1] - t_ee_ref)).argmin())
    wrench_idx = ((np.abs(times[idx][0] - t_wrench)).argmin(), (np.abs(times[idx][1] - t_wrench)).argmin())


    # Transform wrench to body frame
    T = np.array([[0, -1, 0], 
                [-1, 0, 0],
                [0, 0, -1]])
    wrench = np.array([T @ f for f in wrench])

    T_world = [Rotation.from_quat(
            np.append(odom_q_subsample[i, 1:4], odom_q_subsample[i,0])
            ).as_matrix() for i in range(len(joint_state))]

    wrench_world = [np.eye(3) @ wrench[i,:]
                        for i in range(len(wrench))]    

    # Cut down to indeces
    wall = wall[wall_idx[0]:wall_idx[1],:]
    wall_q = wall_q[wall_idx[0]:wall_idx[1],:]
    t_wall = t_wall[wall_idx[0]:wall_idx[1]]
    t_ee = t_joint_state[joint_state_idx[0]: joint_state_idx[1]]


    # Get the indeces where the reference was adjusted
    decision_idx = []
    for i in range(ee_ref_idx[0], ee_ref_idx[1]):
        if np.linalg.norm(ee_ref[i, :] - ee_ref[i+1, :]) > 0.1:
            decision_idx += [i]


    ##############################################################
    ############## Synchronize the Trajectories ##################
    # For size we find the maximal change in y (moving forward)  #
    # to imply the beginning of the trajectory and move that to  #
    # t = 0                                                      #
    ##############################################################
    start_idx  = np.abs(np.diff(ee[:,1])).argmax()
    #t_ee -= t_ee[start_idx]

    #############################################################
    ################ Wall Patch Definition ######################
    #############################################################
    wall_pos =  np.mean(wall, axis=0) + 0.04
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

    wall_angle = -0.6 * np.pi/180
    wall_fun = lambda x: np.tan(wall_angle) * x + (wall_pos[0] - np.tan(wall_angle) * wall_pos[1])

    # Only add the first wall
    if idx == 0: 
        
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

        upper_bound = max(wall_top_right[1], wall_top_left[1])
       
        axs.plot(axs.get_xlim(), wall_fun(np.array(axs.get_xlim())), color="black", lw=lw, linestyle="--")


    # Transform such that all walls are at the same spot
    ee[:, 1] -= y_diff


    ###########################################################
    ############# Plot the current data #######################
    ###########################################################
    axs.plot(ee[joint_state_idx[0]:joint_state_idx[1], 0],
             ee[joint_state_idx[0]:joint_state_idx[1], 1],
             color="black", alpha=0.2, zorder=10, lw=0.7*lw)

    for i in decision_idx:
        idx_js = np.abs(t_joint_state - t_ee_ref[i]).argmin()
        idx_wrench = np.abs(t_wrench - t_ee_ref[i]).argmin() + 1


        axs.arrow(ee[idx_js, 0], ee[idx_js, 1],
                wrench_world[idx_wrench][0], wrench_world[idx_wrench][1], color=colors["force"],width=0.01, label=r"$f$")

        ##########################################################
        # Compute the normal distance to the wall and accumulate #
        ##########################################################
        angle = 0.5*np.pi - np.arccos(wrench_world[idx_wrench][0] / np.sqrt(wrench_world[idx_wrench][0]**2 + wrench_world[idx_wrench][1]**2)) 
        print(np.rad2deg(angle))
        err += np.abs(angle - wall_angle)
        num += 1

print(f"Mean alignement error to wall {np.rad2deg(err) / num:.2} over {len(paths)} trajectories and a total of {num} data points")

# Save the Figures
fig.set_size_inches(size)

plt.show()
