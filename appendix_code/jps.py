def expand_horizontal_right(pos, blocked):
    """blocked contains edges and obstacles"""
    x,y = pos
    newx,newy = x,y
    forced_neighbors=[]
    while (newx+1,newy) not in blocked:
        # top is blocked
        if (newx, newy+1) in blocked:
            if (newx+1, newy+1) not in blocked:
                forced_neighbors.append((newx+1,newy+1))

        # bottom is blocked
        if (newx, newy-1) in blocked:
            if (newx+1, newy-1) not in blocked:
                forced_neighbors.append((newx+1,newy-1))
        newx+=1
def expand_horizontal_left(pos, blocked):
    """blocked contains edges and obstacles"""
    x,y = pos
    newx,newy = x,y
    forced_neighbors=[]
    while (newx-1,newy) not in blocked:
        # top is blocked
        if (newx, newy+1) in blocked:
            if (newx-1, newy+1) not in blocked:
                forced_neighbors.append((newx-1,newy+1))

        # bottom is blocked
        if (newx, newy-1) in blocked:
            if (newx-1, newy-1) not in blocked:
                forced_neighbors.append((newx-1,newy-1))
        newx-=1
def expand_vertical_up(pos, blocked):
    """blocked contains edges and obstacles"""
    x,y = pos
    newx,newy = x,y
    forced_neighbors=[]
    while (newx,newy+1) not in blocked:
        # right is blocked
        if (newx+1, newy) in blocked:
            if (newx+1, newy+1) not in blocked:
                forced_neighbors.append((newx+1,newy+1))

        # left is blocked
        if (newx-1, newy) in blocked:
            if (newx-1, newy+1) not in blocked:
                forced_neighbors.append((newx-1,newy+1))
        newy+=1
def expand_vertical_down(pos, blocked):
    """blocked contains edges and obstacles"""
    x,y = pos
    newx,newy = x,y
    forced_neighbors=[]
    while (newx,newy-1) not in blocked:
        # right is blocked
        if (newx+1, newy) in blocked:
            if (newx+1, newy-1) not in blocked:
                forced_neighbors.append((newx+1,newy-1))

        # left is blocked
        if (newx-1, newy) in blocked:
            if (newx-1, newy-1) not in blocked:
                forced_neighbors.append((newx-1,newy-1))
        newy-=1
def expand_diag_upright(pos,blocked):
    x,y = pos
    newx,newy = x,y
    forced_neighbors=[]
    while (newx+1,newy+1) not in blocked:
        if (newx-1,newy+1) in blocked:
            forced_neighbors.append((newx-1,newy+1))
        newx+=1
        newy+=1


def getSuccessors(pos,goal,blocked):
    x,y = pos


