import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


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


path = '/home/anton/Desktop/rosbags/rosbag2_2023_03_16-17_19_22_FirstSuccessfullPositionReference'
topics=['/fmu/in/trajectory_setpoint', '/fmu/out/vehicle_odometry', '/fmu/in/vehicle_visual_odometry']

time = (10, 100)


# create reader instance and open for reading
with Reader(path) as reader:

    # Init the arrays for plotting
    t_ref = []
    reference = []

    t_odom = []
    odom = []
    odom_q = []


    t_mocap = []
    mocap = []
    mocap_q = []

    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topics[0]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_ref += [timestamp]
            reference += [msg.position] 
            
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

# Make all the arrays np arrays
t_ref = np.array(t_ref, dtype=float)
reference = np.array(reference, dtype=float)

t_odom = np.array(t_odom, dtype=float)
odom = np.array(odom, dtype=float)
odom_q = np.array(odom_q, dtype=float)


t_mocap = np.array(t_mocap, dtype=float)
mocap = np.array(mocap, dtype=float)
mocap_q = np.array(mocap_q, dtype=float)


# Find start time and shift time scale and turn it into seconds
start_t = min(np.concatenate((t_ref, t_odom, t_mocap)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9

# Find the indeces for the specified time range
ref_idx = ((np.abs(time[0] - t_ref)).argmin(), (np.abs(time[1] - t_ref)).argmin())
odom_idx = ((np.abs(time[0] - t_odom)).argmin(), (np.abs(time[1] - t_odom)).argmin())
mocap_idx = ((np.abs(time[0] - t_mocap)).argmin(), (np.abs(time[1] - t_mocap)).argmin())


fig1, axs1 = plt.subplots(2)
axs1[0].plot(t_ref[ref_idx[0]:ref_idx[1]], reference[ref_idx[0]:ref_idx[1],2], '--')
axs1[0].plot(t_odom[odom_idx[0]:odom_idx[1]], odom[odom_idx[0]:odom_idx[1],2], '-')
#axs1[0].plot(t_mocap[mocap_idx[0]:mocap_idx[1]], mocap[mocap_idx[0]:mocap_idx[1],2], '-')
axs1[0].legend([r"$z_{ref}$", r"$z_{odom}$",r"$z_{mocap}$"])
axs1[0].grid()
axs1[0].set_xlabel("Time [s]")
axs1[0].set_ylabel("Height [m]")

axs1[1].plot(t_ref[ref_idx[0]:ref_idx[1]], reference[ref_idx[0]:ref_idx[1],:2], '--')
axs1[1].plot(t_odom[odom_idx[0]:odom_idx[1]], odom[odom_idx[0]:odom_idx[1],:2], '-')
#axs1[1].plot(t_mocap[mocap_idx[0]:mocap_idx[1]], mocap[mocap_idx[0]:mocap_idx[1],:2], '-')
axs1[1].legend([r"$x_{ref}$", r"$y_{ref}$",
                r"$x_{odom}$",r"$y_{odom}$",
                r"$x_{mocap}$",r"$y_{mocap}$"])
axs1[1].grid()
axs1[1].set_xlabel("Time [s]")
axs1[1].set_ylabel("Position [m]")

#axs1[2].plot(t_odom, odom_q)
#axs1[2].plot(t_mocap, mocap_q)
#axs1[2].legend([r"$w_{odom}$",r"$x_{odom}$",r"$y_{odom}$",r"$z_{odom}$",
#               r"$w_{mocap}$",r"$x_{mocap}$",r"$y_{mocap}$",r"$z_{mocap}$"])
#axs1[2].grid()
#axs1[2].set_xlabel("Time [s]")
#axs1[2].set_ylabel("Quaternion Value")

fig2, axs2 = plt.subplots()
fig2.gca().set_aspect('equal')
axs2.plot(odom[odom_idx[0]:odom_idx[1],0], odom[odom_idx[0]:odom_idx[1],1])
#axs2.plot(mocap[mocap_idx[0]:mocap_idx[1],0], mocap[mocap_idx[0]:mocap_idx[1],1])
axs2.grid()
axs2.set_xlabel("x [m]")
axs2.set_ylabel("y [m]")
axs2.legend([r"$GT_{odom}$",r"$GT_{mocap}$"])


plt.show()