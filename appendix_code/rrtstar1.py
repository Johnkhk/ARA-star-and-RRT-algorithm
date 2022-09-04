from cmath import inf
from pqdict import minpq
from collections import defaultdict
import random
import math
from utilsyay import *
import numpy as np

        
class rrt:
    def __init__(self, envmap, robot, target,eps=30):
        # robot = tuple(i.item() for i in robot)
        # target = tuple(i.item() for i in target)

        self.robotstart = robot
        self.target = target
        self.envmap=envmap
        self.init(envmap)
        self.graph = defaultdict(list)
        self.graph[self.robotstart] = []
        self.eps=eps
        self.cost = defaultdict(float)
    def init(self,envmap):
        self.cfree=set()
        self.cblock=set()
        self.c = set()
        for i in range(envmap.shape[0]):
            for j in range(envmap.shape[1]):
                if envmap[i,j]==0:
                    self.cfree.add((i,j))
                else:
                    self.cblock.add((i,j))
                self.c.add((i,j))
    def dist(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def sample(self):
        return random.choice(list(self.c))
    def samplefree(self):
        return random.choice(list(self.cfree))
    def nearest(self,x):
        # i,_ = self.neighbors[x].topitem()
        # return i
        mi=float("inf")
        for k in self.graph:
            if self.dist(k,x) < mi:
                mi=self.dist(k,x)
                res = k
        return res
    def near(self,x,r):
        ret=[]
        for node in self.graph:
            if self.dist(node[0],node[1]) < r:
                ret.append(node)
        return ret
    def steer(self,x,y):
        # print(x,y)
        pts = bresenham2D(x[0],x[1],y[0],y[1])
        pts = list((i,j) for i,j in zip(list(pts[0]),list(pts[1])))
        i=0
        # i = len(pts)-1
        # while i>0 and self.dist(pts[i],x)<self.eps::

            # i-=1

        while i<len(pts) and self.dist(pts[i],x)<self.eps:
            # print("plus1")
            i+=1
        return pts[i-1]
        # return pts[1]
    def collisionfree(self,x,y):
        pts = bresenham2D(x[0],x[1],y[0],y[1])
        pts = list((i,j) for i,j in zip(list(pts[0]),list(pts[1])))
        # print(pts)
        for pt in pts:
            if pt not in self.cfree:
                # print("unforunate")
                return False
        # print("free")
        return True
    def checkgoal(self):
        # i,d = self.graph.topitem()
        # print("dist: ", d, "node: ", i)
        # if d < self.eps:
        #     return True
        # return False
        mi = float("inf")
        for k in self.graph:
            if self.dist(k,self.target)<self.eps and self.collisionfree(k,self.target):
                self.graph[self.target].append(k)
                self.graph[k].append(self.target)

                return True
        return False
    def Astar(self,epsilon=1):
        """weighted astar"""
        pos = self.robotstart
        goal = self.target
        ### initiate gs=0, gi to inf ###
        g = gis_init(pos, self.envmap)
        open = minpq({pos:epsilon*h(pos,goal)}) # stores fi = gi+hi
        closed = set()
        
        while goal not in closed:
            i,_ = open.popitem() # remove i with smallest f
            closed.add(i) # inset i into closed
            ### expand state ###
            for j in self.graph[i]: 
                if j in closed:
                    continue                
                cij = dist(i,j)
                # print("what")
                # print(len(self.graph[goal]))

                # print(i,j)
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
            for cx,cy in self.graph[i]: 
                j = (cx,cy)
                # newx,newy = j[0],j[1]
                
                # if j not in g:
                    # continue
                ming.append((g[j],j))

            i = min(ming)[1]
            # print("huehue",i)
            # print(i,pos,goal)
            if i == pos:
                break
            path.append(i)

        ### return for 2 seconds ###
        # return path[-1]
        # return self.returnmove(path[-1])
        return self.returnmove(path[-1])


        ### return current path for plot ###
        path.append(self.robotstart)
        path.insert(0,goal)
        return path
    def returnmove(self,path):
        # tmp = bresenham2D(*self.robotstart,*path)
        tmp2 = bresenham2D(*self.robotstart,*path)
        tmp=[]
        for i,j in zip(list(tmp2[0]),list(tmp2[1])):
            tmp.append((i.item(),j.item()))

        # tmp = list(tuple(i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
        # tmp = [tuple((i,j)) for i,j in zip(list(tmp[0]),list(tmp[1]))]
        # print("tmp"*50, tmp)
        # print("retmove",tmp[1],type(tmp[1][0]))
        # print("ples",tmp[1],self.robotstart)
        # print(path)

        self.graph[tmp[1]].append(self.robotstart)
        self.graph[self.robotstart].append(tmp[1])

        return tuple(int(i) for i in tmp[1])

    def rrtstarmain(self):
        path = self.robotstart
        for i in range(200000):
            # print(i, self.graph)
            xrand = self.samplefree()
            xnearest = self.nearest(xrand)
            xnew = self.steer(xnearest,xrand)   

            if self.collisionfree(xnearest,xnew):
                self.graph[xnew].append(xnearest)
                self.graph[xnearest].append(xnew)
            

            if i%100==0:
                # print("checking")
                if self.checkgoal():
                    print("goal found")
                    path=self.Astar()
                    break
        return path
        

if __name__ == "__main__":
    pass
    # envmap = np.loadtxt("maps/map1.txt")
    # robotstart = np.array([249, 249])
    # targetstart = np.array([399, 399])
    # robotstart = np.array([74, 249])
    # targetstart = np.array([399, 399])
    # #1
    # robotstart = np.array([699, 799])
    # targetstart = np.array([699, 1699])
    # #1b
    # robotstart = np.array([249, 1199])
    # targetstart = np.array([1649, 1899])


    # robotstart,targetstart = tuple(robotstart),tuple(targetstart)
    # rrt = rrt(envmap,robotstart,targetstart)
    # path=[]
    # for i in range(200000):
    #     print(i)
    #     xrand = rrt.samplefree()
    #     xnearest = rrt.nearest(xrand)
    #     xnew = rrt.steer(xnearest,xrand)   

    #     if rrt.collisionfree(xnearest,xnew):
    #         rrt.graph[xnew].append(xnearest)
    #         rrt.graph[xnearest].append(xnew)
    #     rrt.cost[xrand] = rrt.dist(xrand,xnearest)



    #     if i%100==0:
    #         # print("checking")
    #         if rrt.checkgoal():
    #             print("yes")
    #             path=rrt.Astar()
    #             break
    #     # break
    # # print(path)
    # # print(rrt.graph.keys())
    # ### plot ###

    # tmpl = []
    # for j in range(1,len(path)):
    #     # tmp += raytrace(path[j-1],path[j])
    #     tmp = bresenham2D(*path[j-1],*path[j])
    #     tmp = list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
    #     tmpl += tmp

    # # print("path: ",tmp)
    # plot_graph2(tmpl, rrt.robotstart, rrt.target, envmap)




    

