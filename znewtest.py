from importlib.resources import path
from math import floor
import numpy as np
def sign(n):
    print(type(n))
    return (n > 0) - (n < 0)

def raytrace(A, B):
    """ Return all cells of the unit grid crossed by the line segment between
        A and B.
    """

    (xA, yA) = A
    (xB, yB) = B
    (dx, dy) = (xB - xA, yB - yA)
    (sx, sy) = (sign(dx), sign(dy))

    grid_A = (floor(A[0]), floor(A[1]))
    grid_B = (floor(B[0]), floor(B[1]))
    (x, y) = grid_A
    traversed=[grid_A]

    tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
    tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")

    while (x,y) != grid_B:
        print("wot ",(x,y),grid_B)
        # NB if tIx == tIy we increment both x and y
        (movx, movy) = (tIx <= tIy, tIy <= tIx)

        if movx:
            # intersection is at (x + sx, yA + tIx / dx^2)
            x += sx
            tIx = dy * (x + sx - xA)

        if movy:
            # intersection is at (xA + tIy / dy^2, y + sy)
            y += sy
            tIy = dx * (y + sy - yA)

        traversed.append( (x,y) )

    return traversed
# a = raytrace((0,0),(2.22,23.3))
# tmp = []
# path = [(74, 249), (148.72475954683773, 222.08798967615007), (161.53742655005482, 212.70649368629367), (225.2035522402197, 215.12989811041012), (274.00182671944907, 216.11459038635817), (308.10788874980335, 232.652744718248), (340.49253198138854, 251.86115382465454), (347.3675535743691, 255.95188457796144), (371.7833759956958, 305.36150357314347), (399, 399)]
# for j in range(1,len(path)):
#     yes = raytrace(path[j-1],path[j])
#     tmp += yes
# print("path: ",tmp)

# a = raytrace((74, 249), (148, 230))

# a = raytrace((74, 249), (148, 222))
# print(a)



# a = raytrace((699, 799), (701, 828))


# print(a[1])
# print(a)

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
    # print(x)
    # return tuple((i,j) for i,j in zip(list(x),list(y)))

# a = bresenham2D(*(699, 799), *(701, 828))
# # a = bresenham2D(699, 799, 701, 828)
# a = list((i,j) for i,j in zip(list(a[0]),list(a[1])))
# print(a)

tmpl = []
path = [(74, 249), (137.45517420750866, 234.9947081924284), (162.24112900809516, 209.2176835114429), (182.33070421992286, 204.87996959391924), (241.2577540704845, 186.72544757802484), (302.7735696171119, 203.7939694364927), (358.3572176981154, 262.9233040414083), (399, 399)]
for j in range(1,len(path)):
    one = tuple(int(k) for k in path[j-1])
    two = tuple(int(k) for k in path[j])
    # print(*one,"ww",*two)
    # print(*path[j-1],"ww",*path[j])
    # tmp += bresenham2D(*path[j-1],*path[j])
    # print(one)
    # print(two)
    # print(bresenham2D(*one,*two))
    tmp = bresenham2D(*one,*two)
    tmpl += list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
    print(tmpl)
    # print(list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1]))))
    # tmp +=list((i,j) for i,j in zip(list(tmp[0]),list(tmp[1])))
    # tmp += bresenham2D(*one,*two)

# print(tmp)