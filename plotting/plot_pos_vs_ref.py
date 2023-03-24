import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle 
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from robot import fk

dpi = 250
size = (20, 10)

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


add_types = {}

for pathstr in [
    '/home/anton/Desktop/px4_ros_com_ros2/src/px4_msgs/msg/VehicleOdometry.msg',
    '/home/anton/Desktop/px4_ros_com_ros2/src/px4_msgs/msg/TrajectorySetpoint.msg'
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)


path = '/home/anton/Desktop/rosbags/rosbag2_2023_03_23-13_57_45_reference_pushed_into_wall'
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/wrench',
        '/joint_states',
        '/ee_reference']

time = (12, 45)

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

# Transform from ned to enu
T = np.array([[0,1,0],[1,0,0],[0,0,-1]])
reference = np.array([T @ reference[i,:].transpose() for i in range(len(reference))])
odom = np.array([T @ odom[i,:].transpose() for i in range(len(odom))])
mocap = np.array([T @ mocap[i,:].transpose() for i in range(len(mocap))])

# Find the odometry points that most closely match the joint_state messages
odom_subsample = np.array([odom[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
odom_q_subsample = np.array([odom_q[(np.abs(t_i - t_odom)).argmin(), :] for t_i in t_joint_state])
# Compute the EE positions
ee = np.array([fk(odom_subsample[i,:], odom_q_subsample[i,:], joint_state[i,:]) for i in range(len(t_joint_state))])

# Find start time and shift time scale and turn it into seconds
start_t = min(np.concatenate((t_ref, t_odom, t_mocap)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_wrench = (t_wrench - start_t) * 1e-9
t_joint_state = (t_joint_state - start_t) * 1e-9
t_ee_reference = (t_ee_reference - start_t) * 1e-9

# Find the indeces for the specified time range
ref_idx = ((np.abs(time[0] - t_ref)).argmin(), (np.abs(time[1] - t_ref)).argmin())
odom_idx = ((np.abs(time[0] - t_odom)).argmin(), (np.abs(time[1] - t_odom)).argmin())
mocap_idx = ((np.abs(time[0] - t_mocap)).argmin(), (np.abs(time[1] - t_mocap)).argmin())
wrench_idx = ((np.abs(time[0] - t_wrench)).argmin(), (np.abs(time[1] - t_wrench)).argmin())
joint_state_idx = ((np.abs(time[0] - t_joint_state)).argmin(), (np.abs(time[1] - t_joint_state)).argmin())
ee_ref_idx = ((np.abs(time[0] - t_ee_reference)).argmin(), (np.abs(time[1] - t_ee_reference)).argmin())


####################################################
############ Base Position and Reference ###########
####################################################
fig1, axs1 = plt.subplots(3)
axs1[0].plot(t_ref[ref_idx[0]:ref_idx[1]], reference[ref_idx[0]:ref_idx[1],2], '--')
axs1[0].plot(t_odom[odom_idx[0]:odom_idx[1]], odom[odom_idx[0]:odom_idx[1],2], '-')
axs1[0].legend([r"$z_{ref}$", r"$z_{odom}$",r"$z_{mocap}$"])
axs1[0].grid()
axs1[0].set_xlabel("Time [s]")
axs1[0].set_ylabel("Height [m]")
axs1[0].set_xlim(time)

axs1[1].plot(t_ref[ref_idx[0]:ref_idx[1]], reference[ref_idx[0]:ref_idx[1],:2], '--')
axs1[1].plot(t_odom[odom_idx[0]:odom_idx[1]], odom[odom_idx[0]:odom_idx[1],:2], '-')
axs1[1].legend([r"$x_{ref}$", r"$y_{ref}$",
                r"$x_{odom}$",r"$y_{odom}$",
                r"$x_{mocap}$",r"$y_{mocap}$"])
axs1[1].grid()
axs1[1].set_xlabel("Time [s]")
axs1[1].set_ylabel("Position [m]")
axs1[1].set_xlim(time)

q0 = odom_q[:,0]
q1 = odom_q[:,1]
q2 = odom_q[:,2]
q3 = odom_q[:,3] 
yaw = np.arctan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0**2 + q1**2 - q2**2 - q3**2
    )

axs1[2].plot(t_ref[ref_idx[0]:ref_idx[1]], 180.0 / np.pi * reference_yaw[ref_idx[0]:ref_idx[1]], '--')
axs1[2].plot(t_odom[odom_idx[0]:odom_idx[1]], 180.0 / np.pi  * yaw[odom_idx[0]:odom_idx[1]], '-')
axs1[2].legend([r"$\psi_{ref}$", r"$\psi$"])
axs1[2].grid()
axs1[2].set_xlabel("Time [s]")
axs1[2].set_ylabel("Yaw [degree]")
axs1[2].set_xlim(time)

####################################################
############## GT Plot  ############################
####################################################
fig2, axs2 = plt.subplots()
fig2.gca().set_aspect('equal')
axs2.plot(odom[odom_idx[0]:odom_idx[1],0], odom[odom_idx[0]:odom_idx[1],1])
axs2.plot(ee[joint_state_idx[0]:joint_state_idx[1],0], ee[joint_state_idx[0]:joint_state_idx[1],1])
axs2.add_patch(Rectangle((-0.5,1.25), 2, 2, color='gray', alpha=0.4))
axs2.grid()
axs2.set_xlabel("x [m]")
axs2.set_ylabel("y [m]")
axs2.set_xlim([-0.5, 0.5])
axs2.set_ylim([-0.5, 1.5])
axs2.legend([r"$GT_{base}$",r"$GT_{EE}$"])


# Contact Force
fig3, axs3 = plt.subplots()
axs3.plot(t_wrench[wrench_idx[0]:wrench_idx[1]], wrench[wrench_idx[0]:wrench_idx[1],:])
axs3.grid()
axs3.set_xlabel("Time [s]")
axs3.set_ylabel("Force")
axs3.legend([r"$f_x$",r"$f_y$",r"$f_z$"])
axs3.set_xlim(time)

####################################################
############## EE Position and Reference ###########
####################################################
fig4, axs4 = plt.subplots(3)
axs4[0].plot(t_ee_reference[ee_ref_idx[0]:ee_ref_idx[1]], ee_reference[ee_ref_idx[0]:ee_ref_idx[1],2], '--')
axs4[0].plot(t_joint_state[joint_state_idx[0]:joint_state_idx[1]], ee[joint_state_idx[0]:joint_state_idx[1],2], '-')
axs4[0].legend([r"$z_{EE,ref}$", r"$z_{EE,odom}$"])
axs4[0].grid()
axs4[0].set_xlabel("Time [s]")
axs4[0].set_ylabel("Height [m]")
axs4[0].set_xlim(time)

axs4[1].plot(t_ee_reference[ee_ref_idx[0]:ee_ref_idx[1]], ee_reference[ee_ref_idx[0]:ee_ref_idx[1],:2], '--')
axs4[1].plot(t_joint_state[joint_state_idx[0]:joint_state_idx[1]], ee[joint_state_idx[0]:joint_state_idx[1],:2], '-')
axs4[1].legend([r"$x_{EE,ref}$", r"$y_{EE,ref}$",
                r"$x_{EE,odom}$",r"$y_{EE,odom}$"])
axs4[1].grid()
axs4[1].set_xlabel("Time [s]")
axs4[1].set_ylabel("Position [m]")
axs4[1].set_xlim(time)


q0_ref = ee_reference_q[:,0]
q1_ref = ee_reference_q[:,1]
q2_ref = ee_reference_q[:,2]
q3_ref = ee_reference_q[:,3] 
ee_yaw_ref = np.pi / 2 - np.arctan2(
        2 * ((q1_ref * q2_ref) + (q0_ref * q3_ref)),
        q0_ref**2 + q1_ref**2 - q2_ref**2 - q3_ref**2
    )


axs4[2].plot(t_ee_reference[ee_ref_idx[0]:ee_ref_idx[1]], 180.0 / np.pi  * ee_yaw_ref[ee_ref_idx[0]:ee_ref_idx[1]], '--')
axs4[2].plot(t_odom[odom_idx[0]:odom_idx[1]], 180.0 / np.pi  * yaw[odom_idx[0]:odom_idx[1]], '-')
axs4[2].legend([r"$\psi_{EE,ref}$", r"$\psi_{EE}$"])
axs4[2].grid()
axs4[2].set_xlabel("Time [s]")
axs4[2].set_ylabel("Yaw [degree]")
axs4[2].set_xlim(time)


# Save the Figures
fig1.set_size_inches(size)
fig2.set_size_inches(size)
fig3.set_size_inches(size)
fig4.set_size_inches(size)
fig1.savefig("base_pose.png", dpi=dpi, bbox_inches='tight')
fig2.savefig("groundtrack.png", dpi=dpi, bbox_inches='tight')
fig3.savefig("force.png", dpi=dpi, bbox_inches='tight')
fig4.savefig("ee_pose.png", dpi=dpi, bbox_inches='tight')
