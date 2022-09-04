import numpy as np
from pqdict import minpq
from utils import *
from Astar import *

def Astar(envmap, robotpos, targetpos, epsilon=1):
  """
    Weighted Astar
  """
  pos = tuple(robotpos)
  goal = tuple(targetpos)
  ### initiate gs=0, gi to inf ###
  g = gis_init(pos, envmap)
  open = minpq({pos:epsilon*h(pos,goal)}) # stores fi = gi+hi
  closed = set()
  dX = [-1, -1, -1, 0, 0, 1, 1, 1]
  dY = [-1,  0,  1, -1, 1, -1, 0, 1]
  childrend = [(x,y) for x,y in zip(dX,dY)]
  
  while goal not in closed:
    i,_ = open.popitem() # remove i with smallest f
    closed.add(i) # inset i into closed
    ### expand state ###
    for cx,cy in childrend: 
      j = (i[0]+cx,i[1]+cy)
      if j in closed:
        continue
      newx,newy = j[0],j[1]
      if (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1]):
        if(envmap[newx, newy] == 0):
          cij = dist(i,j)
          if g[j] > g[i] + cij:
            g[j] = g[i] + cij
            ### parent ### wut
            if j in open:
              open[j] = g[j] + epsilon*h(j,goal)
            else:
              open[j] = g[j] + epsilon*h(j,goal)

  ### backtrack to start ###
  path=[]
  i=goal
  nextstate = goal
  while True:
    ming=[]
    for cx,cy in childrend: 
      j = (i[0]+cx,i[1]+cy)
      newx,newy = j[0],j[1]
      if not (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1]):
        continue
      if not (envmap[newx, newy] == 0):
        continue
      if j not in g:
        continue
      ming.append((g[j],j))

    i = min(ming)[1]
    # print("huehue",i)
    # print(i,pos,goal)
    if i == pos:
      break
    path.append(i)
  # plot_graph2(path, pos,goal,envmap)
  return path[-1]


