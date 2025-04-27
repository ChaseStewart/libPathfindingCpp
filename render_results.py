#!/usr/bin/python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv

fig, ax = plt.subplots(1)

with open("results.csv", newline="") as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if row == []:
            continue
        if "1" == row[0]:
            idx = float(row[1])
            x_0 = float(row[2])
            x_1 = float(row[4])
            y_0 = float(row[3])
            y_1 = float(row[5])
            path = eval(row[6])
            x_coords, y_coords = zip(*path)
            print(f"[Node {int(idx)}] {x_0},{y_0}->{x_1},{y_1}")
            plt.plot(x_coords, y_coords, label=f"node:{int(idx)}", zorder=1)
            plt.scatter(x_0, y_0, marker='o', color="r", zorder=2)
            plt.scatter(x_1, y_1, marker='o', color="b", zorder=3)
            ax.annotate(f"{x_0},{y_0}", (x_0, y_0))
            ax.annotate(f"{x_1},{y_1}", (x_1, y_1))
        elif "2" == row[0]:
            x_0 = float(row[10])
            x_1 = float(row[11])
            y_0 = float(row[12])
            y_1 = float(row[13])
            print(f"[Boundary] {x_0},{y_0} {x_1},{y_1}")
            rect = patches.Rectangle((x_0, y_0), (x_1- x_0), (y_1 - y_0), fill=False)
            ax.add_patch(rect)
            plt.xlim(x_0 - 1, x_1 + 1)
            plt.ylim(y_0 - 1, y_1 + 1)
        elif "3" == row[0]:
            c_x_0 = float(row[7])
            c_y_0 = float(row[8]) 
            rad   = float(row[9])
            print(f"[Obstacle] {c_x_0},{c_y_0} with radius {rad}")
            circ = patches.Circle((c_x_0, c_y_0), rad, zorder=0)
            ax.add_patch(circ)
        else:
            pass

plt.legend(loc="lower left")
plt.title("pathfinding results, red=agent, blue=target")
plt.show()
