from pqdict import minpq
from utils import *
from collections import defaultdict

class arastar():
    def __init__(self,robotpos,targetpos,envmap, epsilon):
        self.robotpos=robotpos
        self.targetpos=targetpos
        self.envmap=envmap
        self.epsilon=epsilon
        self.e=epsilon
        pos = tuple(self.robotpos)
        goal = tuple(self.targetpos)
        self.reinit(pos,goal)
        self.g =  defaultdict(lambda: float('inf'))
        self.g[pos]=0
        # self.g = gis_init(pos, self.envmap)
        # self.v = vis_init(self.envmap)
        self.open = minpq({pos:self.e*h(pos,goal)}) # stores fi = gi+hi
        self.INCONS = minpq()
        self.closed=set()

    def reinit(self, pos, goal):
        self.g =  defaultdict(lambda: float('inf'))
        self.g[pos]=0
        self.open = minpq({pos:self.e*h(pos,goal)}) # stores fi = gi+hi
        self.INCONS = minpq()
        self.closed=set()


        # self.open = minpq({pos:self.e*h(pos,goal)}) # stores fi = gi+hi
        # self.INCONS = minpq()
        # self.closed=set()
        pass

        # self.g = gis_init(pos, self.envmap)
        # self.v = vis_init(self.envmap)
        # self.times=0

    
    def computepath(self, envmap, goal):
        # print("*"*30)
        # print(len(self.open),len(self.INCONS))
        # print(self.open.topitem()[1])
        """v keeps track of inconsistent nodes"""
        dX = [-1, -1, -1, 0, 0, 1, 1, 1]
        dY = [-1,  0,  1, -1, 1, -1, 0, 1]
        childrend = [(x,y) for x,y in zip(dX,dY)]
        # INCONS = minpq()
        # print(g[goal],h(goal,goal),open.top())
        # print("###"*30)
        # print(len(self.open), self.open.topitem()[1])
        while self.open and self.g[goal] + self.e*h(goal,goal) > self.open.topitem()[1]:
            # print("@@@"*30)
            # print(len(self.open))
            i,_=self.open.popitem()
            self.closed.add(i) # inset i into closed
            # self.v[i]=self.g[i]

            for cx,cy in childrend: 
                j = (i[0]+cx,i[1]+cy)
                newx,newy = j[0],j[1]
                if not (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1]):
                    continue
                if  not (envmap[newx, newy] == 0):
                    continue
                # if j not in self.g:
                    # continue
                cij = dist(i,j)
                if self.g[j]>self.g[i]+cij:
                    self.g[j]=self.g[i]+cij
                    if j not in self.closed:
                        self.open[j] = self.g[j] + self.e*h(j, goal)
                    else:
                        self.INCONS[j]=self.g[j] + self.e*h(j, goal)

    def ARAstar(self, envmap, robotpos, targetpos):
        self.reinit(robotpos,targetpos)
        pos = tuple(robotpos)
        goal = tuple(targetpos)
        self.e = self.epsilon

        while self.e>1:

            self.reinit(pos,goal)
            # print("@@@"*30)
            # print(len(self.open))

            dX = [-1, -1, -1, 0, 0, 1, 1, 1]
            dY = [-1,  0,  1, -1, 1, -1, 0, 1]
            childrend = [(x,y) for x,y in zip(dX,dY)]

            self.computepath(envmap,goal)

            ### publish subopt sol ###
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
                    if j not in self.g:
                        continue
                    ming.append((self.g[j],j))

                i = min(ming)[1]
                # print(path)
                if i == pos:
                    break
                path.append(i)
                # print("i: ",i,"pos: ",pos,"goal: ",goal, "path[-1]: ",path[-1])

            self.open = minpq({**self.open,**self.INCONS})
            self.e/=2
            # break
            # self.times+=1
            # if self.times==self.recalctimes:
                # self.times=0
                # self.reinit(pos,i)

            # plot_graph2(path, pos,goal,envmap)

        print("i: ",i,"pos: ",pos,"goal: ",goal, "path[-1]: ",path[-1])
        return path[-1]

                        