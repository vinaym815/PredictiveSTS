#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from fileReader import file
from fileReader import computePhaseTimes


# In[2]:


plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": True,
    "font.sans-serif": ["Helvetica"]})
## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
})


# In[4]:


yDataNames = ["time", "com_x", "com_y", "com_vel_x", "com_vel_y"]
file1 = file(yDataNames, "Data/00pct_com.mot", 14)


# In[5]:


time = file1.getColumn("time")
com_x = file1.getColumn("com_x")
com_y = file1.getColumn("com_y")
com_vel_x = file1.getColumn("com_vel_x")
com_vel_y = file1.getColumn("com_vel_y")


# In[6]:


points = np.array([com_x, com_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, axs = plt.subplots(1, 1, figsize=(5.5, 5.5), dpi=300)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(0.0, 1.6)
lc = LineCollection(segments, cmap='jet', norm=norm)
# Set the values used for colormapping
lc.set_array(time)
lc.set_linewidth(2)
line = axs.add_collection(lc)
colorbar = fig.colorbar(line, ax=axs)

axs.axis('equal')
axs.set_title("comX vs comY") 
colorbar.set_label("time (s)")
axs.set_xlabel("x(m)")
axs.set_ylabel("y(m)")
axs.set_ylim(0.65, 0.96)
axs.set_xlim(-0.15, 0.16)

plt.savefig("Figures/comXY_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[7]:


plt.figure(figsize=(5.5, 3.5), dpi=300)
plt.plot(time, com_vel_x, label="com velocity x")
plt.plot(time, com_vel_y, label="com velocity y")

plt.xlim([0.0, 1.6])
plt.ylim([-0.1, 1.22])
plt.xlabel("time (sec)")
plt.ylabel("vel (m/s)")
plt.legend(prop={'size': 8})
plt.savefig("Figures/comVel_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[8]:


plt.figure(figsize=(5.5, 3.5), dpi=300)
plt.plot(time, com_x, label="com x")
plt.plot(time, com_y, label="com y")

plt.xlim([0.0, 1.6])
plt.xlabel("time (sec)")
plt.ylabel("m")
plt.legend(prop={'size': 8})
plt.savefig("Figures/comxyt_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[ ]:




