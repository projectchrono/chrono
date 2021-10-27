import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

import glob

input_dir = "output/iros/lidar/"
output_dir = "output/iros/lidar_imgs/"

file_list = glob.glob(input_dir+"*.csv")

for f in range(1000,len(file_list)):
    filename = input_dir + "frame_" + str(f) + ".csv"
    # data = np.random.rand(10,4)*180 - 90
    data = np.loadtxt(filename,delimiter=',',usecols=range(4))

    x = data[:,0][data[:,3]>0.01]
    y = data[:,1][data[:,3]>0.01]
    z = data[:,2][data[:,3]>0.01]

    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x,y,z, c=z, marker='o',s=.02)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0,20)
    ax.set_ylim(-5,5)
    ax.set_zlim(0,5)

    ax.view_init(30, -165)

    plt.savefig(output_dir+"img_"+str(f)+".png")
    print(filename)

# filename = "frame_1.csv"
#
# # data = np.random.rand(10,4)*180 - 90
# data = np.loadtxt(filename,delimiter=',',usecols=range(3))
# print(data)
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(data[:,0],data[:,1],data[:,2], c=data[:,2]/100, marker='o',s=2)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_xlim(0,100)
# ax.set_ylim(-50,50)
# ax.set_zlim(0,100)
#
# ax.view_init(20, -60)
#
# plt.savefig("savedimg.png")
