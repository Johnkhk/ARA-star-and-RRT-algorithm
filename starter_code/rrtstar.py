from cmath import inf
from pqdict import minpq
from collections import defaultdict
import random
import math
from utils import *
import numpy as np

        
class rrt:
    def __init__(self, envmap, robot, target,eps=30):
        self.robotstart = robot
        self.target = target
        self.envmap=envmap
        self.init(envmap)
        self.graph = defaultdict(set)
        self.graph[self.robotstart] = set()
        self.eps=eps
        self.cost = defaultdict(float)
        self.storedpath=[]
        self.cursample = ()
    def init(self,envmap):
        self.cfree=set()
        self.cfreelist=[]
        # self.cblock=set()
        # self.c = set()
        for i in range(envmap.shape[0]):
            for j in range(envmap.shape[1]):
                if envmap[i,j]==0:
                    self.cfree.add((i,j))
                    self.cfreelist.append((i,j))

                # else:
                    # self.cblock.add((i,j))
                # self.c.add((i,j))
    def dist(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def sample(self):
        return random.choice(tuple(self.c))
    def samplefree(self):
        # return random.choice(tuple(self.cfree))
        return random.choice(self.cfreelist)

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
        # pts = list((int(i),int(j)) for i,j in zip(list(pts[0]),list(pts[1])))
        tmp=[]
        for i,j in zip(list(pts[0]),list(pts[1])):
            tmp.append((int(i.item()),int(j.item())))
        pts=tmp


        # pts = list((i.item(),j.item()) for i,j in list(zip(list(pts[0]),list(pts[1]))))

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
        # pts = list((int(i),(j)) for i,j in zip(list(pts[0]),list(pts[1])))
        pts = bresenham2D(x[0],x[1],y[0],y[1])
        # pts = list((int(i),int(j)) for i,j in zip(list(pts[0]),list(pts[1])))
        tmp=[]
        for i,j in zip(list(pts[0]),list(pts[1])):
            # tmp.append((i.item(),j.item()))
            tmp.append((int(i.item()),int(j.item())))

        pts=tmp


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
        if not isinstance(self.target[0],int):
            self.target = tuple(i.item() for i in self.target)
        mi = float("inf")
        for k in self.graph:
            if self.dist(k,self.target)<self.eps and self.collisionfree(k,self.target):
            # if self.collisionfree(k,self.target):

                self.graph[self.target].add(k)
                self.graph[k].add(self.target)

                return True
        return False
    def Astar(self,epsilon=1, once=False):
        """weighted astar"""
        pos = self.robotstart
        goal = self.target
        if not isinstance(pos[0],int):
            pos = tuple(i.item() for i in pos)
        if not isinstance(goal[0],int):
            goal = tuple(i.item() for i in goal)
        ### initiate gs=0, gi to inf ###
        g = gis_init(pos, self.envmap)
        # g = {pos:0}
        open = minpq({pos:epsilon*h(pos,goal)}) # stores fi = gi+hi
        closed = set()
        
        while goal not in closed:
            i,_ = open.popitem() # remove i with smallest f
            closed.add(i) # inset i into closed
            ### expand state ###
            for j in self.graph[i]: 
                if j not in g: continue
                # if not g[j]: continue
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
            for j in self.graph[i]:
            # for cx,cy in self.graph[i]: 
                # j = (cx,cy)
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
        
        if once:
            # print("HERE RETURN"*20)
            ### return current path for plot ###
            path.append(self.robotstart)
            path.insert(0,goal)
            return path
        # print("PATHASTAR", path)
        # self.storedpath = path

        if not path:
            return self.returnmove(self.target)

        return self.returnmove(path[-1])


        
    def returnmove(self,path):
        # tmp = bresenham2D(*self.robotstart,*path)
        if not isinstance(self.robotstart[0],int):
            self.robotstart = tuple(i.item() for i in self.robotstart)
        print("path in returnmove: ",path)
        tmp2 = bresenham2D(*self.robotstart,*path)

        tmp=[]
        for i,j in zip(list(tmp2[0]),list(tmp2[1])):
            # tmp.append((i.item(),j.item()))
            tmp.append((int(i.item()),int(j.item())))
        self.storedpath = tmp

        ### option 2 ###
        for j in range(len(tmp)):
            for k in tmp[:j] + tmp[j+1:]: #mylist[:x] + mylist[x+1:]
                self.graph[tmp[j]].add(k)
                self.graph[k].add(tmp[j])


        # self.graph[tmp[len(tmp)//2]].add(self.robotstart)
        # self.graph[self.robotstart].add(tmp[len(tmp)//2])

        ### option 1 ###        
        self.graph[tmp[1]].add(self.robotstart)
        self.graph[self.robotstart].add(tmp[1])
        return tuple(int(i) for i in tmp[1])
    
    def rrtstarmain(self):
        ## option 2 add all guys in path to new node ###

        ### option 1 stored path ###
        # if self.storedpath:
        #     n=self.storedpath[0]
        #     self.storedpath.pop(0)
        #     return n

        path = self.robotstart
        for i in range(1,10000000):
            # print(i, self.graph)
            xrand = self.samplefree()
            xnearest = self.nearest(xrand)
            xnew = self.steer(xnearest,xrand)   

            if not isinstance(xnew[0],int):
                xnew = tuple(i.item() for i in xnew)
            
            if isinstance(xnearest[0],float):
                xnearest = tuple(int(i) for i in xnearest)
            if not isinstance(xnearest[0],int):
                xnearest = tuple(i.item() for i in xnearest)

            if self.collisionfree(xnearest,xnew):
                
                self.graph[xnew].add(xnearest)
                self.graph[xnearest].add(xnew)
            

            if i%100==0:
                # print("checking")
                if self.checkgoal():
                    print("goal found")
                    path=self.Astar()
                    break
        return path
        

if __name__ == "__main__":
    once = True
    envmap = np.loadtxt("maps/map3.txt")
    robotstart = np.array([249, 249])
    targetstart = np.array([399, 399])
    # robotstart = np.array([74, 249])
    # targetstart = np.array([399, 399])
    #1
    # robotstart = np.array([699, 799])
    # targetstart = np.array([699, 1699])
    #1b
    # robotstart = np.array([249, 1199])
    # targetstart = np.array([1649, 1899])

    #map7
    # robotstart = np.array([0, 0])
    # targetstart = np.array([4998, 4998])


    robotstart,targetstart = tuple(robotstart),tuple(targetstart)
    rrt = rrt(envmap,robotstart,targetstart,10)
    path=[]
    term=False
    for i in range(1000000000):
        print(i)
        t1 = tic()
        xrand = rrt.samplefree()
        self.cursample = xrand
        t2=toc(t1,"samplefree")

        t1 = tic()
        xnearest = rrt.nearest(xrand)
        t2=toc(t1,"nearest")
        t1 = tic()
        xnew = rrt.steer(xnearest,xrand)  
        t2=toc(t1,"steer")
        


        t1=tic()
        if rrt.collisionfree(xnearest,xnew):
            rrt.graph[xnew].add(xnearest)
            rrt.graph[xnearest].add(xnew)
        toc(t1, "collisionfree")



        if i%100==0:
            # print("checking",rrt.graph)
            if rrt.checkgoal():
                print("yes")
                path=rrt.Astar(once=True)
                if once:
                    term = True
                break
        if term:
            break
        # break
    # print(path)
    # print(rrt.graph.keys())
    ### plot ###

    tmpl = []
    for j in range(1,len(path)):
        # tmp += raytrace(path[j-1],path[j])
        tmp = bresenham2D(*path[j-1],*path[j])
        tmp = list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
        tmpl += tmp

    # print("path: ",tmp)
    plot_graph2(tmpl, rrt.robotstart, rrt.target, envmap)




    

