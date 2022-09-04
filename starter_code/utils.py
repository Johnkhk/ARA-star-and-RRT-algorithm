import matplotlib.pyplot as plt
from numpy import loadtxt
import math
import numpy as np
import time 


def dist(robotpos,targetpos):
  """heursitc function is just euclidean distance"""
  return math.sqrt((robotpos[0]-targetpos[0])**2 + (robotpos[1]-targetpos[1])**2)

def h(robotpos,targetpos):
  """heursitc function is just euclidean distance"""
  return math.sqrt((robotpos[0]-targetpos[0])**2 + (robotpos[1]-targetpos[1])**2)

def gis_init(pos, envmap):
  gis={}
  for i in range(envmap.shape[0]):
    for j in range(envmap.shape[1]):
      if(envmap[i, j] == 0):
        gis[(i,j)]=float("inf")
  gis[pos]=0
  return gis
def vis_init(envmap):
  vis={}
  for i in range(envmap.shape[0]):
    for j in range(envmap.shape[1]):
      if(envmap[i, j] == 0):
        vis[(i,j)]=float("inf")
  return vis
def tic():
      return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

def plot_graph(opt, pos, goal, envmap):
    """
      Useful for seeing how one run did
      Will pause the main loop in main
    """
    robotpos = pos
    targetpos = goal
    
    # draw the environment
    # transpose because imshow places the first dimension on the y-axis
    f, ax = plt.subplots()
    ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
                extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
    ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
    ax.set_xlabel('x')
    ax.set_ylabel('y')  
    hr = ax.plot(robotpos[0], robotpos[1], 'bs')
    ht = ax.plot(targetpos[0], targetpos[1], 'rs')
    f.canvas.flush_events()


    for x,y in opt:
        # draw positions
        hr[0].set_xdata(x)
        hr[0].set_ydata(y)
        ht[0].set_xdata(goal[0])
        ht[0].set_ydata(goal[1])
        f.canvas.flush_events()
        plt.show()
        plt.pause(0.05)
    # plt.show(block=True)

def plot_graph2(opt, pos, goal, envmap):
    robotpos = pos
    targetpos = goal
    
    # environment
    # envmap = loadtxt(mapfile)
        
    # draw the environment
    # transpose because imshow places the first dimension on the y-axis
    f, ax = plt.subplots()
    ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
                extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
    ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
    ax.set_xlabel('x')
    ax.set_ylabel('y')  
    hr = ax.plot(robotpos[0], robotpos[1], 'bs')
    ht = ax.plot(targetpos[0], targetpos[1], 'rs')
    f.canvas.flush_events()

    x,y=[],[]
    for a,b in opt:
          x.append(a)
          y.append(b)
    for i in range(0, len(opt), 2):
        # plt.plot(x[i:i+2], y[i:i+2], 'ro-')
        # plt.plot(x[i:i+2], y[i:i+2], 'g-',linewidth=0.5 )
        plt.plot(x[i:i+2], y[i:i+2], 'g-')


    plt.show(block=True)


def bresenham2D(sx, sy, ex, ey):
      
  sx = int(round(sx))
  sy = int(round(sy))
  ex = int(round(ex))
  ey = int(round(ey))
  dx = abs(ex-sx)
  dy = abs(ey-sy)
  steep = abs(dy)>abs(dx)
  if steep:
    dx,dy = dy,dx # swap 

  if dy == 0:
    q = np.zeros((dx+1,1))
  else:
    q = np.append(0,np.greater_equal(np.diff(np.mod(np.arange( np.floor(dx/2), -dy*dx+np.floor(dx/2)-1,-dy),dx)),0))
  if steep:
    if sy <= ey:
      y = np.arange(sy,ey+1)
    else:
      y = np.arange(sy,ey-1,-1)
    if sx <= ex:
      x = sx + np.cumsum(q)
    else:
      x = sx - np.cumsum(q)
  else:
    if sx <= ex:
      x = np.arange(sx,ex+1)
    else:
      x = np.arange(sx,ex-1,-1)
    if sy <= ey:
      y = sy + np.cumsum(q)
    else:
      y = sy - np.cumsum(q)
  return np.vstack((x,y))
