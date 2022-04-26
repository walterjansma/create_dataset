from os import error
import matplotlib.pyplot as plt
import numpy as np
#import pandas as pd
import rosbag
import os
from scipy.spatial.transform import Rotation as R
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import matplotlib as mpl
import csv

home = os.path.expanduser("~")
pose_key = "odometry_enu"
traj_key = "trajectory"
map_key = "map"
rectangle_key = "rectangle"
directory = "/data_bags/"
folder = "/create_dataset/"
bag_name = 'test'

class Map:
    size = [1715, 881]
    data = plt.imread(home + '/catkin_ws/src/roboat_social_simulator/config/maps/herengracht_3_way.png')
    origin = [-78, -40, 0.0]
    resolution = 0.081


def main():
    """
    # Get names of all bag files
    bag_files = getBagNames()

    #print(bag_files)

    data = []
    agents = 0

    for file in bag_files:

        # Load data
        bag = rosbag.Bag(home + '/data_bags/' + file)
        pose_topics = load_topics(bag, pose_key)
        num_agents, pos_data = read_pose_data_new(bag, pose_topics)

        pos_data = reindex_agents(pos_data, agents)

        agents = agents + num_agents

        data.append(pos_data)

    save_to_file(data)
    """
    data = np.genfromtxt(home + folder + "datafile.csv", delimiter=",")[1:,:]

    print(data[20000,:])

    #topics = bag.get_type_and_topic_info()[1].keys()

    #print(topics)

    #messages = []
    #for topic, msg, t in bag.read_messages(topics=topics[0]):
    #	messages.append(msg)
    """

    diff = []
    diff_ = []
    for i in range(10):

        time1 = pos_data['roboat_0'][i][5]
        time2 = pos_data['roboat_0'][i+1][5]
        dt = time2 - time1
        diff.append(dt)

        time1_ = pos_data['roboat_1'][i][5]
        time2_ = pos_data['roboat_1'][i+1][5]
        dt_ = time2_ - time1_
        diff_.append(dt_)

    print(diff)
    print(diff_)

    print(pos_data['roboat_0'][10][5])
    print(pos_data['roboat_1'][10][5])
    """

def reindex_agents(data, agents):

    for i in range(len(data)):
        if data[i][0] == 0:
            data[i][0] = agents + 1
        elif data[i][0] == 1:
            data[i][0] = agents + 2
        elif data[i][0] == 2:
            data[i][0] = agents + 3

    return data


def save_to_file(data):
    """
    Save data to csv
    """
    fields = ['id', 'timestep_s', 'timestep_ns', 'pos_x', 'pox_y', 'yaw', 'vel_x', 'vel_y', 'omega', 'goal_x', 'goal_y' ]

    with open('datafile.csv', 'w') as f:

        write = csv.writer(f)
        write.writerow(fields)

        for i in range(len(data)):
            write.writerows(data[i])

def getBagNames():
    """
    Retrieve file names of all bags in specified directory
    """
    bag_files = []
    for root, dirs, files in os.walk(home + directory):
        for file in files:
            if file.endswith('.bag'):
                bag_files.append(file)

    return bag_files

def load_topics(bag, key = None):
    """
    Return relevant topics according to key
    """
    if key is None:
        return bag.get_type_and_topic_info()[1]
    else:
        topics = bag.get_type_and_topic_info()[1]
        topics = [topic for topic in topics if key in topic]
        return topics

def read_pose_topic_new(bag, topic):
    """
    Return the topic name ('roboat_0') and data = [x, y, yaw, vx, vy, time]. Yaw is in radians
    [frame_number pedestrian_ID pos_x pos_z pos_y v_x v_z v_y ]
    """
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
            yaw = rot_mat.as_euler('zyx')[0] # This is in radians

            entry = (int(extract_ns_from_topic(topic)[7]), # id, if 'roboat_0', id = 0
                    msg.header.stamp.secs, # timestep (s)
                    msg.header.stamp.nsecs, # timestep (ns)
                    msg.pose.pose.position.x, # pos x
                    msg.pose.pose.position.y, # pos y
                    yaw, # yaw
                    msg.twist.twist.linear.x, # vel x
                    msg.twist.twist.linear.y, # vel y
                    0, # omega
                    0, # goal x
                    0) # goal y
        else:
            print("Unkown msg type: ", msg_type)
            continue
        data.append(entry)
        topic_name = extract_ns_from_topic(topic)
    data = np.asarray(data)
    #print(data.shape)

    return topic_name, data


def read_pose_topic(bag, topic):
    """
    Return the topic name ('roboat_0') and data = [x, y, yaw, vx, vy, time]. Yaw is in radians
    """
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
            yaw = rot_mat.as_euler('zyx')[0] # This is in radians

            entry = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     yaw,
                     msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     time)
        else:
            print("Unkown msg type: ", msg_type)
            continue
        data.append(entry)
        topic_name = extract_ns_from_topic(topic)
    data = np.asarray(data)
    #print(data.shape)

    return topic_name, data

def read_pose_data_new(bag, topic_list):
    """
    Return a dictionairy of all data on the topics in the input
    """
    datalist = []
    agents = []
    for topic in topic_list:
        topic, data = read_pose_topic_new(bag, topic)
        agents.append(topic)
        datalist.append(data)

    flat_list = [item for sublist in datalist for item in sublist]

    return len(agents), flat_list

def read_pose_data(bag, topic_list):
    """
    Return a dictionairy of all data on the topics in the input
    """
    data_dict = {}
    for topic in topic_list:
        (name, data) = read_pose_topic(bag, topic)
        data_dict[name] = data

    return data_dict

def extract_ns_from_topic(topic):
    """
    Extract topic name string
    """
    return topic.replace("/","",1).split("/", 1)[0]

if __name__ == "__main__":
    main()
