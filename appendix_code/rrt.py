# from cmath import inf
# from pqdict import minpq
# from collections import defaultdict
# import random
# import math
# from utilsyay import *
# import numpy as np


# class rrt:
#     def __init__(self, envmap, robot, target,eps=10):
#         self.robotstart = tuple(i.item() for i in robot)
#         self.target = tuple(i.item() for i in target)
#         self.envmap=envmap
#         self.init(envmap)
#         # self.neighbors = minpq({robot:float(inf)}) # dict of minpq used for nearest
#         # self.nodes
#         # self.neighbors =defaultdict(tuple)
#         # self.neighbors[robot]=minpq()

#         # self.neighbors = {robot:minpq()}

#         # self.graph = defaultdict(list) # dict of lists of keys for more dicts
#         # self.graph[robot]=[]
#         self.graph = minpq({self.robotstart:self.dist(self.robotstart,self.target)})
#         self.eps=eps
#     def init(self,envmap):
#         self.cfree=set()
#         self.cblock=set()
#         self.c = set()
#         for i in range(envmap.shape[0]):
#             for j in range(envmap.shape[1]):
#                 if envmap[i,j]==0:
#                     self.cfree.add((i,j))
#                 else:
#                     self.cblock.add((i,j))
#                 self.c.add((i,j))
#     def dist(self,p1,p2):
#         return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

#     def sample(self):
#         return random.choice(list(self.c))
#     def samplefree(self):
#         return random.choice(list(self.cfree))
#     def nearest(self,x):
#         # i,_ = self.neighbors[x].topitem()
#         # return i
#         mi=float("inf")
#         for k in self.graph:
#             if self.dist(k,x) < mi:
#                 mi=self.dist(k,x)
#                 res = k
#         return res
#     def near(self,x,r):
#         ret=[]
#         for node in self.graph:
#             if self.dist(node[0],node[1]) < r:
#                 ret.append(node)
#         return ret
#     def steer(self,x,y):
#         # print(x,y)
#         pts = bresenham2D(x[0],x[1],y[0],y[1])
#         pts = list((i,j) for i,j in zip(list(pts[0]),list(pts[1])))
#         i=0
#         # i = len(pts)-1
#         # while i>0 and self.dist(pts[i],x)<self.eps::

#             # i-=1

#         while i<len(pts) and self.dist(pts[i],x)<self.eps:
#             # print("plus1")
#             i+=1
#         return pts[i-1]
#         # return pts[1]
#     def collisionfree(self,x,y):
#         pts = bresenham2D(x[0],x[1],y[0],y[1])
#         pts = list((i,j) for i,j in zip(list(pts[0]),list(pts[1])))
#         # print(pts)
#         for pt in pts:
#             if pt not in self.cfree:
#                 # print("unforunate")
#                 return False
#         # print("free")
#         return True
#     def checkgoal(self):
#         i,d = self.graph.topitem()
#         print("dist: ", d, "node: ", i)
#         if d <= self.eps:
#             return True
#         return False
#     def connect(self):
#         pass
#         # # pathdist=[]
#         # prev = self.target
#         # path = [prev]
#         # while prev != self.robotstart:
#         #     node,d = self.graph.popitem()
#         #     if self.collisionfree(prev,node):
#         #         path.append(node)
#         #     prev = node
#         # return path


#         # nodes = list(self.graph.keys()) # closest to farthest nodes from goal
#         # l,r=0,len(nodes)
#         # while True:
#             # if self.collisionfree(start,node):

#         # print("aaaaa"*30)
#         # print(list(self.graph.keys())) # closest to farthest nodes from goal
#         # nodes = []
#         # while self.graph:
#         #     node,d = self.graph.popitem()
#         # nodes.append(node) # closest to farthest nodes from goal
#         # l,r=0,len(nodes)
#         # print("aaaaa"*30)
#         # print(nodes)
#         # while True:
#             # if self.collisionfree(start,node):
#     def Astar(self,epsilon=1):
#         """weighted astar"""
#         pos = self.robotstart
#         goal = self.target
#         ### initiate gs=0, gi to inf ###
#         g = gis_init(pos, envmap)
#         open = minpq({pos:epsilon*h(pos,goal)}) # stores fi = gi+hi
#         closed = set()
#         dX = [-1, -1, -1, 0, 0, 1, 1, 1]
#         dY = [-1,  0,  1, -1, 1, -1, 0, 1]
#         childrend = [(x,y) for x,y in zip(dX,dY)]
        
#         while goal not in closed:
#             i,_ = open.popitem() # remove i with smallest f
#             closed.add(i) # inset i into closed
#             ### expand state ###
#             # for cx,cy in childrend: 
#             for cx,cy in self.graph[goal]: 
#                 j = (i[0]+cx,i[1]+cy)
#                 if j in closed:
#                     continue                
#                 cij = dist(i,j)
#                 if g[j] > g[i] + cij:
#                     g[j] = g[i] + cij
#                     ### parent ### wut
#                     if j in open:
#                         open[j] = g[j] + epsilon*h(j,goal)
#                     else:
#                         open[j] = g[j] + epsilon*h(j,goal)
        

# if __name__ == "__main__":
#     envmap = np.loadtxt("maps/map3.txt")
#     robotstart = np.array([249, 249])
#     targetstart = np.array([399, 399])
#     # robotstart = np.array([74, 249])
#     # targetstart = np.array([399, 399])
#     robotstart,targetstart = tuple(robotstart),tuple(targetstart)
#     rrt = rrt(envmap,robotstart,targetstart)
#     path=[]
#     for i in range(20000):
#         # print(i)
#     # while True:
#         xrand = rrt.samplefree()
#         # print("rand",xrand)
#         xnearest = rrt.nearest(xrand)
#         xnew = rrt.steer(xnearest,xrand)   
#         # print("xrand",xrand,"xnearest",xnearest, "xnew", xnew)
#         if rrt.collisionfree(xnearest,xnew):
#             rrt.graph[xnew] = rrt.dist(xnew,rrt.target)
#             # rrt.graph[xnearest] = rrt.dist(xnearest,rrt.target)
#             # rrt.graph[xnew].append(xnearest)
#             # rrt.graph[xnearest].append(xnew)
#         # print("e")

#         # path = rrt.connect()
#     # print(path)
#         if i%100==0:
#             # print("checking")
#             if rrt.checkgoal():
#                 print("yes")
#                 path=rrt.Astar()
#                 break
#         # break
#     print(path)
#     # print(rrt.graph.keys())

#     ### plot ###
#     tmpl = []
#     for j in range(1,len(path)):
#         # tmp += raytrace(path[j-1],path[j])
#         tmp = bresenham2D(*path[j-1],*path[j])
#         tmp = list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
#         tmpl += tmp

#     # print("path: ",tmp)
#     plot_graph2(tmpl, rrt.robotstart, rrt.target, envmap)




    

