from os import error
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import bagpy
from bagpy import bagreader

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

if __name__ == "__main__":
    main()
