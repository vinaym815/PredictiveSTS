#!/usr/bin/env python
# coding: utf-8

# In[4]:


from matplotlib import pyplot as plt
import numpy as np

plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": True,
    "font.sans-serif": ["CMU Sans Serif"],
    "font.size" : 10})

## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "CMU Serif",
    "font.serif": ["Palatino"],
})


# In[5]:


class exponential:
    def __init__(self, tau, tf):
        self.tau = tau
        self.tf = tf
        
    def computeWeight(self, t):
        return np.exp(t/self.tau)/(self.tau*(np.exp(self.tf/self.tau)-1))
        
    def computeWeights(self, inputVec):
        return np.array([self.computeWeight(xi) for xi in inputVec])

    def computeArea(self, t1, t2):
        return (np.exp(t2/self.tau) - np.exp(t1/self.tau))/ (np.exp(self.tf/self.tau) -1)


# In[6]:


tf = 1.6
tauChair = tf/8.0
tVec = np.linspace(0,tf,50)

expChair = exponential(tauChair, tf) 

plt.plot(tVec, expChair.computeWeights(tVec), label=r'$\tau=0.2s, T=1.6s$')
plt.xlabel(r'$t(s)$')
plt.ylabel(r'$exp(t|\tau,T)$')
plt.legend()
plt.savefig("./figures/expFunc.png", dpi=300, format="png")
#plt.show()
