import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

dpi = 250
size = (20, 10)

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)



path = '/home/anton/Desktop/rosbags/rosbag2_2023_03_22-14_40_20_angle_validation'
topics=['/wrench', '/mocap_pose', '/wall_pose']

time = (15, 51)

# create reader instance and open for reading
with Reader(path) as reader:

    # Init the arrays for plotting
    t_wrench = []
    wrench = []

    t_mocap = []
    mocap = []
    mocap_q = []

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
            t_wrench += [timestamp]
            wrench += [[msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]]

        if connection.topic == topics[1]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_mocap += [timestamp]
            mocap += [[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]]
            mocap_q += [[msg.pose.orientation.w, msg.pose.orientation.x,
                         msg.pose.orientation.y, msg.pose.orientation.z]]
        
        if connection.topic == topics[2]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_wall += [timestamp]
            wall += [[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]]
            wall_q += [[msg.pose.orientation.w, msg.pose.orientation.x,
                         msg.pose.orientation.y, msg.pose.orientation.z]]


# Make all the arrays np arrays
t_wrench = np.array(t_wrench, dtype=float)
wrench = np.array(wrench, dtype=float)

t_mocap = np.array(t_mocap, dtype=float)
mocap = np.array(mocap, dtype=float)
mocap_q = np.array(mocap_q, dtype=float)

t_wall = np.array(t_wall, dtype=float)
wall = np.array(wall, dtype=float)
wall_q = np.array(wall_q, dtype=float)

# Find start time and shift time scale and turn it into seconds
start_t = min(np.concatenate((t_wrench, t_mocap, t_wall)))
t_wrench = (t_wrench - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_wall = (t_wall - start_t) * 1e-9

# Find the indeces for the specified time range
wrench_idx = ((np.abs(time[0] - t_wrench)).argmin(), (np.abs(time[1] - t_wrench)).argmin())
mocap_idx = ((np.abs(time[0] - t_mocap)).argmin(), (np.abs(time[1] - t_mocap)).argmin())
wall_idx = ((np.abs(time[0] - t_wall)).argmin(), (np.abs(time[1] - t_wall)).argmin())


fig1, axs1 = plt.subplots()
base_q0 = mocap_q[:,0]
base_q1 = mocap_q[:,1]
base_q2 = mocap_q[:,2]
base_q3 = mocap_q[:,3]
base_yaw = np.arctan2(
        2 * ((base_q1 * base_q2) + (base_q0 * base_q3)),
        base_q0**2 + base_q1**2 - base_q2**2 - base_q3**2
    )

wall_q0 = wall_q[:,0]
wall_q1 = wall_q[:,1]
wall_q2 = wall_q[:,2]
wall_q3 = wall_q[:,3]
wall_yaw = np.arctan2(
        2 * ((wall_q1 * wall_q2) + (wall_q0 * wall_q3)),
        wall_q0**2 + wall_q1**2 - wall_q2**2 - wall_q3**2
    )

estm_yaw =  (wrench[:,1] > 0 * (-1)) * np.arccos(wrench[:,0] / np.sqrt(wrench[:,0]**2 + wrench[:,1]**2))

contact_phases = ((np.abs(wrench[:,0]) > 0.15) | (np.abs(wrench[:,1]) > 0.15)) * 1
time_temp = np.linspace(t_wrench[0], t_wrench[-1], len(contact_phases));

axs1.plot(t_mocap[mocap_idx[0]:mocap_idx[1]], 180.0 / np.pi * (base_yaw[mocap_idx[0]:mocap_idx[1]] -wall_yaw[wall_idx[0]:wall_idx[1]]))
axs1.plot(t_wrench[wrench_idx[0]:wrench_idx[1]], 180.0 / np.pi * estm_yaw[wrench_idx[0]:wrench_idx[1]] * contact_phases[wrench_idx[0]:wrench_idx[1]])
axs1.fill_between(time_temp[wrench_idx[0]:wrench_idx[1]], 100 * contact_phases[wrench_idx[0]:wrench_idx[1]], -100 * contact_phases[wrench_idx[0]:wrench_idx[1]], alpha=0.4)
axs1.grid()
axs1.legend(["Motion Capture Relative Angle", "Estimated Angle"])
axs1.set_xlabel("Time [s]")
axs1.set_ylabel("Angle [Degree]")
axs1.set_xlim(time)
axs1.set_ylim([-30,110])
fig1.set_size_inches(size)

fig3, axs3 = plt.subplots()
axs3.plot(t_wrench[wrench_idx[0]:wrench_idx[1]], wrench[wrench_idx[0]:wrench_idx[1],:])
axs3.fill_between(time_temp[wrench_idx[0]:wrench_idx[1]], 0.4 * contact_phases[wrench_idx[0]:wrench_idx[1]], - 0.2 * contact_phases[wrench_idx[0]:wrench_idx[1]], alpha=0.4)
axs3.grid()
axs3.set_xlabel("Time [s]")
axs3.set_ylabel("Force")
axs3.legend([r"$f_x$",r"$f_y$",r"$f_z$"])
axs3.set_xlim(time)
axs3.set_ylim([-0.2,0.4])


plt.show()