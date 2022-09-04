import numpy as np
import math
from numpy import loadtxt
# import matplotlib
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
plt.ion()
import time
# from utilsyay import *
# from robotplanner import robotplanner
from robotplanner import *

from targetplanner import targetplanner
import sys
sys.path.append("/Users/john/Downloads/ECE276B_PR2/rrt-algorithms/examples/rrt_star")
sys.path.append("/Users/john/Downloads/ECE276B_PR2/rrt-algorithms")

from yuhyuh import *

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


def runtest(mapfile, robotstart, targetstart):
  # current positions of the target and robot
  robotpos = np.copy(robotstart);
  # print("robot start: ", robotstart)
  # print("target start: ", targetstart)
  
  targetpos = np.copy(targetstart);

  robotpos = tuple(robotpos)
  targetpos = tuple(targetpos)
  
  # environment
  envmap = loadtxt(mapfile)
  X, Q, x_init, x_goal, max_samples, r, prc, rewire_count = init(robotpos,targetpos,envmap)
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
  # plt.pause(0.05)

  plt.show()
  # plt.show(block=True)

  # now comes the main loop
  numofmoves = 0
  caught = False
  for i in range(20000):
    # call robot planner
    t0 = tic()
    # newrobotpos = robotplanner(envmap, robotpos, targetpos)
    # newrobotpos = robotplanner(envmap, robotpos, targetpos)
    # newrobotpos = run(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
    newrobotpos = run(X, Q, tuple(robotpos), tuple(targetpos), max_samples, r, prc, rewire_count, envmap)
    newrobotpos = tuple(newrobotpos)
    break

    toc(t0, "planner")
    # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
    movetime = max(1, math.ceil((tic()-t0)/2.0))
    
    #check that the new commanded position is valid
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
         newrobotpos[1] < 0 or newrobotpos[1] >= envmap.shape[1] ):
      print('ERROR: out-of-map robot position commanded\n')
      break
    elif ( envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
      print('ERROR: invalid robot position commanded\n')
      break
    elif (abs(newrobotpos[0]-robotpos[0]) > 1 or abs(newrobotpos[1]-robotpos[1]) > 1):
      print('ERROR: invalid robot move commanded\n')
      break

    # call target planner to see how the target moves within the robot planning time
    newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime)
    
    # make the moves
    robotpos = tuple(newrobotpos)
    targetpos = newtargetpos
    numofmoves += 1
    
    # draw positions
    hr[0].set_xdata(robotpos[0])
    hr[0].set_ydata(robotpos[1])
    ht[0].set_xdata(targetpos[0])
    ht[0].set_ydata(targetpos[1])
    f.canvas.flush_events()
    plt.show()
    plt.pause(0.01)
    # plt.show(block=True)

    # check if target is caught
    if (abs(robotpos[0]-targetpos[0]) <= 1 and abs(robotpos[1]-targetpos[1]) <= 1):
      print('robotpos = (%d,%d)' %(robotpos[0],robotpos[1]))
      print('targetpos = (%d,%d)' %(targetpos[0],targetpos[1]))
      caught = True
      break
    # break
  return caught, numofmoves


def test_map0():
  robotstart = np.array([0, 2])
  targetstart = np.array([5, 3])
  return runtest('maps/map0.txt', robotstart, targetstart)

def test_map1():
  robotstart = np.array([699, 799])
  targetstart = np.array([699, 1699])
  return runtest('maps/map1.txt', robotstart, targetstart)

def test_map2():
  robotstart = np.array([0, 2])
  targetstart = np.array([7, 9])
  return runtest('maps/map2.txt', robotstart, targetstart)
  
def test_map3():
  robotstart = np.array([249, 249])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)

def test_map4():
  robotstart = np.array([0, 0])
  targetstart = np.array([5, 6])
  return runtest('maps/map4.txt', robotstart, targetstart)

def test_map5():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 59])
  return runtest('maps/map5.txt', robotstart, targetstart)

def test_map6():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 36])
  return runtest('maps/map6.txt', robotstart, targetstart)

def test_map7():
  robotstart = np.array([0, 0])
  targetstart = np.array([4998, 4998])
  return runtest('maps/map7.txt', robotstart, targetstart)


def test_map1b():
  robotstart = np.array([249, 1199])
  targetstart = np.array([1649, 1899])
  return runtest('maps/map1.txt', robotstart, targetstart)

def test_map3b():
  robotstart = np.array([74, 249])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)

def test_map3c():
  robotstart = np.array([4, 399])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)
  

if __name__ == "__main__":
  # you should change the following line to test different maps
  caught, numofmoves = test_map7()
  print('Number of moves made: {}; Target caught: {}.\n'.format(numofmoves, caught))
  plt.ioff()
  # plt.show()

  plt.show(block=True)



