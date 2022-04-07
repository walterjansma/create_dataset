from os import error
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import os
from scipy.spatial.transform import Rotation as R
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import matplotlib as mpl


home = os.path.expanduser("~")
pose_key = "odometry_enu"
traj_key = "trajectory"
map_key = "map"
rectangle_key = "rectangle"

bag_name = 'test'

class Map:
    size = [1715, 881]
    data = plt.imread(home + '/catkin_ws/src/roboat_social_simulator/config/maps/herengracht_3_way.png')
    origin = [-78, -40, 0.0]
    resolution = 0.081


def main():

    fig, ax = plt.subplots()
    #map = plt.imread(home + '/catkin_ws/src/roboat_social_simulator/config/maps/herengracht_3_way.png')
    #bag = bagreader(home + '/mppi_bags/' + bag_name + '.bag')

    bag = rosbag.Bag(home + '/mppi_bags/' + bag_name + '.bag')

    # Load roboat odometry topics
    pose_topics = load_topics(bag, pose_key)

    pose_data = read_pose_data(bag, pose_topics)

    print(pose_data['roboat_0'][0]) # output = [ x, y, heading, np.datetime64()]
    step_size = 30

    plot_map(Map, ax)
    plot_poses(pose_data, ax, step_size = step_size)
    plt.show()

    """
    #target1 = bag.message_by_topic('waypoint_target/roboat_1')
    #boat1 = bag.message_by_topic('roboat_1/arrow')# 'waypoint_target/roboat_0')

    #df0 = pd.read_csv(target1)
    #df1 = pd.read_csv(boat1)

    print(df0.head())
    print(len(df0))
    print(len(df1))
    print(df1.head())

    print(df1['header.seq'])

    #Plot
    ax.imshow(map, extent=[-15, 15, -15, 15],cmap='Greys' )
    ax.scatter(df0['point.x'][::10], df0['point.y'][::10], c='r')
    ax.scatter(df1['pose.position.x'][::10], df1['pose.position.y'][::10], c='b')
    plt.xlim([-15, 15])
    plt.ylim([-15,15])
    plt.show()
    """

def load_topics(bag, key = None):
    if key is None:
        return bag.get_type_and_topic_info()[1]
    else:
        topics = bag.get_type_and_topic_info()[1]
        topics = [topic for topic in topics if key in topic]
        return topics

def read_pose_topic(bag, topic):
    print('Reading topic: '+topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic not found, skipping...")
        msg_type = "not_found_in_bag"

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = np.datetime64(msg.header.stamp.secs, 's')+np.timedelta64(msg.header.stamp.nsecs, 'ns')
        if msg_type=='geometry_msgs/PoseWithCovarianceStamped' or msg_type=='nav_msgs/Odometry':
            q0, q1, q2, q3 = (msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z,
                              msg.pose.pose.orientation.w)

            rot_mat = R.from_quat([q0, q1, q2, q3])
            yaw = rot_mat.as_euler('zyx')[0]

            entry = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     yaw, time)
        else:
            print("Unkown msg type: ", msg_type)
            continue
        data.append(entry)
        topic_name = extract_ns_from_topic(topic)
    data = np.asarray(data)
    #print(data.shape)

    return topic_name, data

def read_pose_data(bag, topic_list):
    data_dict = {}
    for topic in topic_list:
        (name, data) = read_pose_topic(bag, topic)
        data_dict[name] = data

    return data_dict

def extract_ns_from_topic(topic):
    return topic.replace("/","",1).split("/", 1)[0]

def read_map_data(bag, topic):
    map = Map()
    for topic, msg, t in bag.read_messages(topics=[topic]):
        map.size = (msg.info.height, msg.info.width)
        map.data = np.asarray(msg.data).reshape(map.size)
        map.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        map.resolution = msg.info.resolution

def plot_poses(data, ax, vessel_colors = ['r', 'g', 'b', 'm'], path_colors = None, step_size = 1):
    if path_colors is None:
        path_colors = ['r', 'g', 'b', 'm']
    verts = [(-2, 1),
          (-2, -1),
          (1, -1),
          (2, 0),
          (1, 1),
          (-2, 1)]
    codes = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]

    path = Path(verts, codes)
    marker = Path(verts, codes)

    #patch = patches.PathPatch(path, facecolor='orange', lw=2)

    iterator = 0
    for (key, poses) in data.items():
        #print(data[key].data)
        plot_data = []
        #for i in range(len(data[key].data[0])):
        #    plot_data.append([element[i] for element in data[key].data])

        stamps_for_boats = np.arange(0, len(poses), step_size)
        #print(stamps_for_boats)
        t_start = poses[0, 3]
        duration = poses[-1, 3] - t_start
        for (x,y, phi, t) in poses[stamps_for_boats]:
            #print(x,y, phi, t)
            marker_transformed = marker.transformed(mpl.transforms.Affine2D().rotate(phi).translate(x, y))
            patch = PathPatch(marker_transformed,
                              lw=2,
                              alpha = 0.1 + 0.9 *(t - t_start) / duration)
            patch.set_linewidth(1)
            ax.add_patch(patch)

        #ax.scatter([],[], marker = path, s = 500, )
        ax.scatter(poses[:, 0], poses[:, 1], s = 2)
        #ax.scatter(poses[stamps_for_boats, 0], poses[stamps_for_boats, 1], color = colors[key], s = 50)

        iterator = iterator + 1

def get_type(color):
    #print(olor)
    if (color == [1.0, 0.4000000059604645, 0.0]):
        return "MPPI "
    elif (color == [0.0, 0.699999988079071, 0.0]):
        return "JS "
    else:
        return ""

def plot_map(map, ax):
    ax.imshow(map.data, vmin = 0, vmax = 100,
             extent = (map.origin[0], map.origin[0] + map.size[0] * map.resolution,
                       map.origin[1], map.origin[1] + map.size[1] * map.resolution),
             cmap='gray_r')
    ax.set_facecolor('#262626')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.yaxis.set_label_coords(-0.08, .5)
    ax.xaxis.set_label_coords(0.5, -0.09)


if __name__ == "__main__":
    main()
