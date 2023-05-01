import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
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


# Install non-standard types
add_types = {}

for pathstr in [
    '/home/anton/Desktop/cloned_software/px4_ros_com_ros2/src/px4_msgs/msg/VehicleOdometry.msg',
    '/home/anton/Desktop/cloned_software/px4_ros_com_ros2/src/px4_msgs/msg/TrajectorySetpoint.msg'
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)



# The time interval we want to plot
time = (15, 49)

# File path to rosbag
path ='/home/anton/Desktop/rosbags/2023_04_28/wavy_surface/rosbag2-16_53_32-success5'


# Topics to collect data from
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/wrench',
        '/joint_states',
        '/ee_reference',
        '/wall_pose']

##############################################################
############## Load all the data #############################
##############################################################

# create reader instance and open for reading
with Reader(path) as reader:

    # Init the arrays for plotting
    t_ref = []
    reference = []
    reference_yaw = []

    t_odom = []
    odom = []
    odom_q = []

    t_mocap = []
    mocap = []
    mocap_q = []

    t_wrench = []
    wrench = []

    t_joint_state = []
    joint_state = []

    t_ee_reference = []
    ee_reference = []
    ee_reference_q = []

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
            t_ref += [timestamp]
            reference += [msg.position] 
            reference_yaw += [msg.yaw]
            
        if connection.topic == topics[1]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_odom += [timestamp]
            odom += [msg.position]
            odom_q += [msg.q]

        if connection.topic == topics[2]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_mocap += [timestamp]
            mocap += [msg.position]
            mocap_q += [msg.q]

        if connection.topic == topics[3]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_wrench += [timestamp]
            wrench += [[msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]]

        if connection.topic == topics[4]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_joint_state += [timestamp]
            joint_state += [msg.position]
        
        if connection.topic == topics[5]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_ee_reference += [timestamp]
            ee_reference += [[msg.pose.position.x,
                              msg.pose.position.y,
                              msg.pose.position.z]]
            ee_reference_q += [[msg.pose.orientation.w,
                                msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z]]
            
        if connection.topic == topics[6]:
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
t_ref = np.array(t_ref, dtype=float)
reference = np.array(reference, dtype=float)
reference_yaw = np.array(reference_yaw, dtype=float)

t_odom = np.array(t_odom, dtype=float)
odom = np.array(odom, dtype=float)
odom_q = np.array(odom_q, dtype=float)

t_mocap = np.array(t_mocap, dtype=float)
mocap = np.array(mocap, dtype=float)
mocap_q = np.array(mocap_q, dtype=float)

t_wrench = np.array(t_wrench, dtype=float)
wrench = np.array(wrench, dtype=float)

t_joint_state = np.array(t_joint_state, dtype=float)
joint_state = np.array(joint_state, dtype=float)

t_ee_reference = np.array(t_ee_reference, dtype=float)
ee_reference = np.array(ee_reference, dtype=float)
ee_reference_q = np.array(ee_reference_q, dtype=float)

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
reference = np.array([T @ reference[i,:].transpose() for i in range(len(reference))])
reference_yaw = -reference_yaw;
odom = np.array([T @ odom[i,:].transpose() for i in range(len(odom))])
odom_q = np.array([T_q @ odom_q[i,:].transpose() for i in range(len(odom))])
mocap = np.array([T @ mocap[i,:].transpose() for i in range(len(mocap))])
mocap_q = np.array([T_q @ mocap_q[i,:].transpose() for i in range(len(mocap))])
wall = np.array([T @ wall[i,:].transpose() for i in range(len(wall))])
wall_q = np.array([T_q @ wall_q[i,:].transpose() for i in range(len(wall_q))])

# Find the odometry points that most closely match the joint_state messages
odom_subsample = np.array([odom[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
odom_q_subsample = np.array([odom_q[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
# Compute the EE positions
ee = np.array([fk(odom_subsample[i,:], odom_q_subsample[i,:], joint_state[i,:]) for i in range(len(t_joint_state))])

# Find start time and shift time scale and turn it into seconds
start_t = min(np.concatenate((t_ref, t_odom, t_mocap, t_joint_state, t_ee_reference, t_wall)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_wrench = (t_wrench - start_t) * 1e-9
t_joint_state = (t_joint_state - start_t) * 1e-9
t_ee_reference = (t_ee_reference - start_t) * 1e-9
t_wall = (t_wall - start_t) * 1e-9

# Find the indeces for the specified time range
ref_idx = ((np.abs(time[0] - t_ref)).argmin(), (np.abs(time[1] - t_ref)).argmin())
odom_idx = ((np.abs(time[0] - t_odom)).argmin(), (np.abs(time[1] - t_odom)).argmin())
#mocap_idx = ((np.abs(time[0] - t_mocap)).argmin(), (np.abs(time[1] - t_mocap)).argmin())
wrench_idx = ((np.abs(time[0] - t_wrench)).argmin(), (np.abs(time[1] - t_wrench)).argmin())
joint_state_idx = ((np.abs(time[0] - t_joint_state)).argmin(), (np.abs(time[1] - t_joint_state)).argmin())
ee_ref_idx = ((np.abs(time[0] - t_ee_reference)).argmin(), (np.abs(time[1] - t_ee_reference)).argmin())
wall_idx = ((np.abs(time[0] - t_wall)).argmin(), (np.abs(time[1] - t_wall)).argmin())

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


# Get the indeces where the reference was adjusted
decision_idx = []
for i in range(ee_ref_idx[0], ee_ref_idx[1]):
    if np.linalg.norm(ee_reference[i, :] - ee_reference[i+1, :]) > 0.1:
        decision_idx += [i]


 #############################################################
################ Wall Patch Definition ######################
#############################################################
wall_pos =  np.mean(wall, axis=0)
y_diff = wall_pos[0] - 1.5
#wall_pos[0] -= y_diff
q0_wall = wall_q[:,0]
q1_wall = wall_q[:,1]
q2_wall = wall_q[:,2]
q3_wall = wall_q[:,3] 
wall_yaw = -np.arctan2(
        2 * ((q1_wall * q2_wall) + (q0_wall * q3_wall)),
        q0_wall**2 + q1_wall**2 - q2_wall**2 - q3_wall**2
    )

wall_angle = np.mean(wall_yaw)


##############################################
###### Wall Length Definitions ###############
#                      l                     #
#               ________________             #
#               |__    __    __| d           #
#                  \__/  \__/    d_b         #
#              off2   a mid b off1           #
#                                            #
##############################################
wall_angle = 0.0 * np.pi/180
d = 0.25
l = 2.4
off1 = 0.23
off2 = 0.21
mid = 0.16
a = 0.4
b = 0.1
d_b = 0.09
print(off1 + off2 + mid + 4*a + 2*b)
assert(np.abs(l - (off1 + off2 + mid + 4*a + 2*b)) < 0.001)

corners = np.stack(([-l, 0],
                    [-l, d],
                    [0, d],
                    [0, 0],
                    [-off1, 0],
                    [-off1-a, -d_b],
                    [-off1-a-b, -d_b],
                    [-off1-2*a-b, 0],
                    [-off1-2*a-b-mid, 0],
                    [-off1-3*a-b-mid, -d_b],
                    [-off1-3*a-2*b-mid, -d_b],
                    [-off1-4*a-2*b-mid, 0.0]
                    ))
rot = np.array([[np.cos(wall_angle), -np.sin(wall_angle)],
                [np.sin(wall_angle),  np.cos(wall_angle)]])
corners = np.array([rot @ corners[i,:] for i in range(len(corners))]) + np.array([0.1, wall_pos[0]])

q0 = odom_q[:,0]
q1 = odom_q[:,1]
q2 = odom_q[:,2]
q3 = odom_q[:,3] 
yaw = np.arctan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0**2 + q1**2 - q2**2 - q3**2
    )

####################################################
############## GT Plot  ############################
####################################################
fig2, axs2 = plt.subplots()
axs2.set_aspect('equal')
axs2.plot(odom[odom_idx[0]:odom_idx[1], 0], odom[odom_idx[0]:odom_idx[1], 1], label=r"$GT_{Base}$", color=colors["base"], lw=lw)
axs2.plot(ee[joint_state_idx[0]:joint_state_idx[1], 0], ee[joint_state_idx[0]:joint_state_idx[1], 1], color=colors["ee"], label=r"$GT_{EE}$", lw=lw)
axs2.scatter(ee_reference[ee_ref_idx[0]:ee_ref_idx[1], 0], ee_reference[ee_ref_idx[0]:ee_ref_idx[1], 1], marker="x", color=colors["ee"], label=r"$p_{EE,ref}$")
#axs2.scatter(reference[ref_idx[0]:ref_idx[1], 0], reference[ref_idx[0]:ref_idx[1], 1], marker="x", color="blue", label=r"$p_{ref}$")

for i in decision_idx[1:]:
    idx_js = np.abs(t_joint_state - t_ee_reference[i]).argmin()
    idx_wrench = np.abs(t_wrench - t_ee_reference[i]).argmin() + 1

    arrow = T_world[idx_js] @  np.array([0, 0.1, 0])

    axs2.arrow(ee[idx_js, 0], ee[idx_js, 1],
               arrow[0], arrow[1],
               color=colors["ee"], linestyle=":", width = 0.01, label=r"$\psi_{EE}$")
    axs2.arrow(ee[idx_js, 0], ee[idx_js, 1],
            wrench_world[idx_wrench][0], wrench_world[idx_wrench][1], color=colors["force"],width=0.01, label=r"$f$")
    
    projection = np.cross(wrench_world[idx_wrench],
                                np.array([0,0,1]))
    
    projection = 0.4 * projection / np.linalg.norm(projection)
    #axs2.arrow(ee_reference[i, 0], ee_reference[i, 1],
    #           projection[0], projection[1],
    #          color="grey")

axs2.add_patch(Polygon(corners,
                        facecolor='xkcd:light grey', edgecolor='black', alpha=1, label='_nolegend_',
                        zorder=0))

axs2.plot([corners[-1,0], corners[-4,0]],
            [corners[-1,1], corners[-4,1]], color='black', lw=0.8*lw, linestyle='--', zorder=3)
axs2.plot([corners[4,0], corners[7,0]],
            [corners[4,1], corners[7,1]], color='black', lw=0.8*lw, linestyle='--', zorder=3)


wall_str = "{\\mathrm{Wall}}"

AngleAnnotation(xy=corners[-1],
                p1=corners[-2],
                p2=corners[-4],
                ax=axs2,
                size=420,
                lw=0.5*lw,
                linestyle=":")
text_position = corners[-1] + np.array([0.05, 0.05])
axs2.annotate(rf"${180/np.pi * np.arctan2(-d_b,a):.1f}^\circ$", text_position, fontsize=0.8*text_size)

AngleAnnotation(xy=corners[4],
                p1=corners[7],
                p2=corners[5],
                ax=axs2,
                size=420,
                lw=0.8*lw,
                linestyle=":")
text_position = corners[4] + np.array([-0.15, 0.05])
axs2.annotate(rf"${180/np.pi * np.arctan2(d_b, a):.1f}^\circ$", text_position, fontsize=0.8*text_size)
axs2.xaxis.set_major_locator(MultipleLocator(1))
axs2.xaxis.set_major_formatter('{x:.0f}')
axs2.xaxis.set_minor_locator(MultipleLocator(0.2))

axs2.yaxis.set_major_locator(MultipleLocator(1))
axs2.yaxis.set_major_formatter('{x:.0f}')
axs2.yaxis.set_minor_locator(MultipleLocator(0.2))

axs2.spines['top'].set_visible(False)
axs2.spines['right'].set_visible(False)
#axs2.spines['bottom'].set_visible(False)
#axs2.spines['left'].set_visible(False)

axs2.set_xlabel("x [m]", fontsize=text_size)
axs2.set_ylabel("y [m]", fontsize=text_size)
axs2.set_xlim([-2.5, 0.3])
axs2.set_ylim([-0.75, 2.5])

handles, labels = fig2.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
axs2.legend(by_label.values(), by_label.keys(), loc="lower left", prop={'size': text_size}, ncol=2, labelspacing=0.1, columnspacing=0.5)

####################################################
############## EE Position and Reference ###########
####################################################
fig4, axs4 = plt.subplots(2, sharex="all")

for i in range(2):
    axs4[i].spines['top'].set_visible(False)
    axs4[i].spines['right'].set_visible(False)
    #axs4[i].spines['bottom'].set_visible(False)
    #axs4[i].spines['left'].set_visible(False)
    axs4[i].xaxis.set_major_locator(MultipleLocator(10))
    axs4[i].xaxis.set_major_formatter('{x:.0f}')
    axs4[i].xaxis.set_minor_locator(MultipleLocator(1))
    axs4[i].tick_params(axis='both', which='major', labelsize=0.8*text_size)


axs4[0].yaxis.set_major_locator(MultipleLocator(1))
axs4[0].yaxis.set_major_formatter('{x:.0f}')
axs4[0].yaxis.set_minor_locator(MultipleLocator(0.2))

axs4[1].yaxis.set_major_locator(MultipleLocator(5))
axs4[1].yaxis.set_major_formatter('{x:.0f}')
axs4[1].yaxis.set_minor_locator(MultipleLocator(1))




axs4[0].plot(t_ee_reference[ee_ref_idx[0]:ee_ref_idx[1]] - t_ee_reference[ee_ref_idx[0]],
             ee_reference[ee_ref_idx[0]:ee_ref_idx[1],:2], '--', color="grey", lw=lw)
axs4[0].plot(t_joint_state[joint_state_idx[0]:joint_state_idx[1]] - t_joint_state[joint_state_idx[0]],
             ee[joint_state_idx[0]:joint_state_idx[1], 0], '-', color=colors["x"], lw=lw)
axs4[0].plot(t_joint_state[joint_state_idx[0]:joint_state_idx[1]] - t_joint_state[joint_state_idx[0]],
             ee[joint_state_idx[0]:joint_state_idx[1], 1], '-', color=colors["y"], lw=lw)
axs4[0].legend([r"$x_{EE,ref}$", r"$y_{EE,ref}$",
                r"$x_{EE}$",r"$y_{EE}$"],
                loc="lower left", prop={'size': text_size}, ncol=2,
                labelspacing=0.1, columnspacing=0.5)
axs4[0].set_ylabel("Position [m]", fontsize=text_size)
axs4[0].set_xlim([0, time[1]-time[0]])

q0_ref = ee_reference_q[:,0]
q1_ref = ee_reference_q[:,1]
q2_ref = ee_reference_q[:,2]
q3_ref = ee_reference_q[:,3] 
ee_yaw_ref = np.arctan2(
        2 * ((q1_ref * q2_ref) + (q0_ref * q3_ref)),
        q0_ref**2 + q1_ref**2 - q2_ref**2 - q3_ref**2
    )


axs4[1].plot(t_ee_reference[ee_ref_idx[0]:ee_ref_idx[1]] - t_ee_reference[ee_ref_idx[0]],
             180.0 / np.pi  * ee_yaw_ref[ee_ref_idx[0]:ee_ref_idx[1]], '--', color="grey", label=r"$\psi_{EE,ref}$", lw=lw)
axs4[1].plot(t_wall[wall_idx[0]:wall_idx[1]] - t_wall[wall_idx[0]],
             180.0 / np.pi * np.ones_like(t_wall[wall_idx[0]:wall_idx[1]])*wall_angle, '--', color=colors["wall_yaw"], label= r"$\psi_{Wall}$", lw=lw)
axs4[1].plot(t_odom[odom_idx[0]:odom_idx[1]] - t_odom[odom_idx[0]],
             180.0 / np.pi  * yaw[odom_idx[0]:odom_idx[1]], '-', color=colors["yaw"], label=r"$\psi_{EE}$", lw=lw)
axs4[1].legend(loc="upper left", prop={'size': text_size}, ncol=3,
                labelspacing=0.1, columnspacing=0.5)
axs4[1].set_xlabel("Time [s]", fontsize=text_size)
axs4[1].set_ylabel(r"Yaw [$^\circ$]", fontsize=text_size)
axs4[1].set_xlim([0, time[1]-time[0]])
#plt.show()

# Save the Figures
fig2.set_size_inches(size)
fig4.set_size_inches(size)
fig2.savefig("../plots/groundtrack.png", dpi=dpi, bbox_inches='tight')
fig4.savefig("../plots/ee_pose.png", dpi=dpi, bbox_inches='tight')
