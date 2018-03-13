import matplotlib.pyplot as plt
import numpy
import seaborn as sns
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection



obstacleFile = '2DForest.txt'


obs = []
with open(obstacleFile) as file:
    for line in file:
        data = map(float, line.split())
        rect = patches.Rectangle([data[0],data[1]], data[2]-data[0],data[3]-data[1])
        obs.append(rect)

ax = plt.gca()
ax.tick_params(labelsize=16)
pc = PatchCollection(obs, facecolor='darkslategrey',edgecolor='darkslategrey')
ax.add_collection(pc)


# Mark the Source and Target
plt.plot([0.1], [0.1], marker='o', markersize=9, color="black", label='source')
plt.plot([0.9], [0.9], marker='s', markersize=9, color="black", label='target')


plt.xlim([0,1])
plt.ylim([0,1])
plt.axis('scaled')
plt.show()