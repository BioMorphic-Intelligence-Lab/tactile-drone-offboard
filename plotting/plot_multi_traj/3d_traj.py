import matplotlib.pyplot as plt 
import numpy as np 
from pathlib import Path
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import sys
# setting path
sys.path.append('../')
from plotting_settings import *
from robot import fk
from robot import rot_z, rot_x

def cuboid_data(o, size=(1,1,1)):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1], o[1], o[1]],          
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]   
    z = [[o[2], o[2], o[2], o[2], o[2]],                       
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],   
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],               
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]               
    return np.array(x), np.array(y), np.array(z)

def plotCubeAt(pos=(0,0,0), size=(1,1,1), ax=None,**kwargs):
    # Plotting a cube element at position pos
    if ax !=None:
        X, Y, Z = cuboid_data( pos, size )
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, **kwargs)


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
    '/home/anton/Desktop/cloned_software/px4_ros_com_ros2/src/px4_msgs/msg/VehicleOdometry.msg'
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)

# The time interval we want to plot
times = [(7.5, 45),(7.5, 52.5),(7.5, 42.5),(7.5, 52.5),(7.5, 47.5)]

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
        '/wall_pose']

####################################################
##### Init the Collection of Trajectories ##########
####################################################
ee_col = []
t_ee_col = []

#####################################################
################ Init the Plots #####################
#####################################################
fig = plt.figure()
axs = plt.axes(projection='3d')

axs.set_xlim([-2, 0.5])
axs.set_ylim([-0.1, 2])
axs.set_zlim([0.0, 2.5])

axs.set_xlabel("x[m]",fontsize=text_size)
axs.set_ylabel("y[m]",fontsize=text_size)
axs.set_zlabel("z[m]",fontsize=text_size)

axs.tick_params(axis='both', which='major', labelsize=0.8*text_size)
axs.xaxis.set_major_locator(MultipleLocator(1))
axs.xaxis.set_major_formatter('{x:.0f}')
axs.xaxis.set_minor_locator(MultipleLocator(0.5))

axs.yaxis.set_major_locator(MultipleLocator(1))
axs.yaxis.set_major_formatter('{x:.0f}')
axs.yaxis.set_minor_locator(MultipleLocator(0.5))

axs.zaxis.set_major_locator(MultipleLocator(1))
axs.zaxis.set_major_formatter('{x:.0f}')
axs.zaxis.set_minor_locator(MultipleLocator(0.5))

fig2, ax2 = plt.subplots()
ax2.set_xlim([-2.4, 0.5])
ax2.set_ylim([1.2, 2.0])

ax2.set_xlabel("x[m]",fontsize=text_size)
ax2.set_ylabel("z[m]",fontsize=text_size)

ax2.tick_params(axis='both', which='major', labelsize=0.8*text_size)
ax2.xaxis.set_major_locator(MultipleLocator(1))
ax2.xaxis.set_major_formatter('{x:.0f}')
ax2.xaxis.set_minor_locator(MultipleLocator(0.2))

ax2.yaxis.set_major_locator(MultipleLocator(1))
ax2.yaxis.set_major_formatter('{x:.0f}')
ax2.yaxis.set_minor_locator(MultipleLocator(0.2))

ax2.set_aspect('equal')


fig3, ax3 = plt.subplots()
ax3.set_xlabel("x[m]",fontsize=text_size)
ax3.set_ylabel("z[m]",fontsize=text_size)

ax3.tick_params(axis='both', which='major', labelsize=0.8*text_size)
ax3.xaxis.set_major_locator(MultipleLocator(1))
ax3.xaxis.set_major_formatter('{x:.0f}')
ax3.xaxis.set_minor_locator(MultipleLocator(0.2))

ax3.yaxis.set_major_locator(MultipleLocator(1))
ax3.yaxis.set_major_formatter('{x:.0f}')
ax3.yaxis.set_minor_locator(MultipleLocator(0.2))


ax3.set_xlim([0.0, 30.0])
ax3.set_ylim([1.4,1.8])

ax3.set_xlabel("Time [s]",fontsize=text_size)
ax3.set_ylabel("z[m]",fontsize=text_size)

ax3.tick_params(axis='both', which='major', labelsize=0.8*text_size)
ax3.xaxis.set_major_locator(MultipleLocator(10))
ax3.xaxis.set_major_formatter('{x:.0f}')
ax3.xaxis.set_minor_locator(MultipleLocator(1))

ax3.yaxis.set_major_locator(MultipleLocator(1))
ax3.yaxis.set_major_formatter('{x:.0f}')
ax3.yaxis.set_minor_locator(MultipleLocator(0.2))


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
    start_idx  = np.abs(np.diff(ee[:,1])).argmax() - 50
    t_ee -= t_ee[start_idx]


    #############################################################
    ################ Wall Patch Definition ######################
    #############################################################
    wall_pos =  np.mean(wall, axis=0)
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
    if idx == 0:
        pass
        plotCubeAt(pos=(-2,1.55,1.3), size=(2.4,0.1,0.6), ax=axs, color='xkcd:light grey',alpha=0.5)
        axs.plot3D([-2,0.4],[1.5,1.5],[1.7,1.7], color="black",linestyle="--", lw=0.5*lw)
        axs.plot3D([-2,0.4],[1.5,1.5],[1.5,1.5], color="black",linestyle="--", lw=0.5*lw)

        ax2.add_patch(Polygon(([-2,1.3], [-2,1.9], [0.4,1.9], [0.4,1.3]), facecolor='xkcd:light grey', edgecolor="black", lw=1.2*lw, alpha=1))
        ax2.plot([-2,0.4],[1.7,1.7], color="black",linestyle="--", lw=0.7*lw)
        ax2.plot([-2,0.4],[1.5,1.5], color="black",linestyle="--", lw=0.7*lw)

        ax3.plot([0,30],[1.7,1.7], color="black",linestyle="--", lw=0.7*lw)
        ax3.plot([0,30],[1.5,1.5], color="black",linestyle="--", lw=0.7*lw)


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
    
    d2_idx = np.argwhere(ee[:, 1] > 1.4)[0][0]
    ax2.plot(ee[d2_idx:joint_state_idx[1], 0],
             ee[d2_idx:joint_state_idx[1], 2],
                color="black", alpha=0.3, lw=0.7*lw)
    ax3.plot(t_ee[joint_state_idx[0]:joint_state_idx[1]],
                ee[joint_state_idx[0]:joint_state_idx[1], 2],
                color="black", alpha=0.3, lw=0.7*lw)
    

    axs.plot3D(ee[joint_state_idx[0]:joint_state_idx[1], 0],
               ee[joint_state_idx[0]:joint_state_idx[1], 1],
               ee[joint_state_idx[0]:joint_state_idx[1], 2],
                color="black", alpha=0.5, lw=lw)



####################################################
########## Find the mean trajectory ################
####################################################
ts = np.concatenate([t for t in t_ee_col])
t_mean = np.arange(start=-5,
                   stop=40,
                   step=1)

mean = []
std = []
for t in t_mean:
    points = []
    for i in range(len(paths)):
        idx = np.abs(t - t_ee_col[i]).argmin()
        points += [ee_col[i][idx, :]]
    mean += [np.mean(points, axis=0)]
    std += [np.std(points, axis=0)]


mean = np.array(mean, dtype=float)
std = np.array(std, dtype=float)

axs.plot3D(mean[:, 0], mean[:,1], mean[:,2], color=colors["ee"], label=r"$\mu_{EE}$", zorder=21)
d2_idx = np.argwhere(mean[:, 1] > 1.4)[0][0]
ax2.plot(mean[d2_idx:, 0], mean[d2_idx:,2], color=colors["ee"], lw=lw, label=r"$\mu_{EE}$", zorder=21)
ax3.plot(t_mean, mean[:,2], color=colors["ee"], lw=lw, label=r"$\mu_{EE,z}$", zorder=21)
draw_error_band(ax3, t_mean, mean[:,2], std[:,2], facecolor=colors["ee"], alpha=.3,label=r"$\sigma_z$")

ax2.legend(loc="lower left", prop={'size': text_size}, ncol=1, labelspacing=0.1, columnspacing=0.5)
ax3.legend(loc="lower left", prop={'size': text_size}, ncol=1, labelspacing=0.1, columnspacing=0.5)

# Set Legend
axs.legend(prop={'size': text_size}, ncol=1, labelspacing=0.1, columnspacing=0.5)
#ax2.legend()

fig2.set_size_inches(size)
fig2.savefig("../plots/wall_painting.png", dpi=dpi, bbox_inches='tight')


fig3.set_size_inches(size)
fig3.savefig("../plots/altitude.png", dpi=dpi, bbox_inches='tight')


#plt.show()