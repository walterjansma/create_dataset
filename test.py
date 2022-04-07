from os import error
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import bagpy
from bagpy import bagreader

class Map:
    size = None
    data = None
    origin = None
    resolution = None

def main():

    fig, ax = plt.subplots()
    map = plt.imread('/home/walter/Downloads/tight_crossing.png')
    bag = bagreader('/home/walter/bagfiles/test.bag')

    target1 = bag.message_by_topic('waypoint_target/roboat_1')
    boat1 = bag.message_by_topic('roboat_1/arrow')# 'waypoint_target/roboat_0')

    df0 = pd.read_csv(target1)
    df1 = pd.read_csv(boat1)
    
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

def read_map_data(bag, topic):
    map = Map()
    for topic, msg, t in bag.read_messages(topics=[topic]):
        map.size = (msg.info.height, msg.info.width)
        map.data = np.asarray(msg.data).reshape(map.size)
        map.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        map.resolution = msg.info.resolution

def plot_map(map, ax):
    ax.imshow(map.data, vmin = 0, vmax = 100,
             extent = (map.origin[0], map.origin[0] + map.size[1] * map.resolution,
                       map.origin[1], map.origin[1] + map.size[0] * map.resolution),
             origin = 'lower', cmap='gray_r')
    ax.set_facecolor('#262626')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.yaxis.set_label_coords(-0.08, .5)
    ax.xaxis.set_label_coords(0.5, -0.09)


if __name__ == "__main__":
    main()
