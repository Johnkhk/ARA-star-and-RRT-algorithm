from re import L
import numpy as np
import math
import time
from pqdict import minpq
from utils import *
from Astar import *
from arastar import *
from rrtstar import *



def robotplanner(envmap, robotpos, targetpos, algo, obj):
  # # return robotplannerog(envmap, robotpos, targetpos)
  # if algo == 1:
  #   return Astar(envmap, robotpos, targetpos, 1)
  # elif algo == 2:  
  #   # ara=arastar(robotpos,targetpos,envmap,32)
  #   return ara.ARAstar(envmap, robotpos, targetpos)
  # elif algo == 3:
  #     robotstart,targetstart = tuple(robotstart),tuple(targetstart)
  #     rrt = rrt(envmap,robotstart,targetstart)
  #   pass  
  #   # return rrt
  # return robotplannerog(envmap, robotpos, targetpos)
  if algo == 1:
    return Astar(envmap, robotpos, targetpos, 1)
  elif algo == 2:  
    # ara=arastar(robotpos,targetpos,envmap,32)
    return obj.ARAstar(envmap, robotpos, targetpos)
  elif algo == 3:
    obj.robotstart = tuple(robotpos)
    obj.target = tuple(targetpos)
    # print("aaaa"*20,type(robotpos[0]),type(targetpos[0]))
    # obj.robotstart = tuple(i.item() for i in robotpos)
    # obj.target = tuple(i.item() for i in targetpos)
    return obj.rrtstarmain()
      # robotstart,targetstart = tuple(robotstart),tuple(targetstart)
      # rrt = rrt(envmap,robotstart,targetstart)
    # pass  
    # return rrt


