import numpy as np
import rosbag
import os
import pickle


home = os.path.expanduser("~")
path = "/social_vrnn/data/real_world/ewap_dataset/seq_eth/"
file_name = "data12_1_0.pickle" 

infile = open(home + path + file_name,'rb')
new_dict = pickle.load(infile)
infile.close()