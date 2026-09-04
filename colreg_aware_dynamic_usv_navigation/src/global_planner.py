import heapq
import math
import numpy as np


class GridMap:
    def __init__(self, width=120, height=80):
        self.width=width
        self.height=height
        self.occ=np.zeros((height,width),dtype=np.uint8)

    def add_rect(self,x0,y0,x1,y1):
        self.occ[y0:y1+1,x0:x1+1]=1

    def free(self,x,y):
        ix,iy=int(round(x)),int(round(y))
        return 0 <= ix < self.width and 0 <= iy < self.height and self.occ[iy,ix] == 0


def astar(grid,start,goal):
    start=tuple(map(int,start)); goal=tuple(map(int,goal))
    h=lambda n: math.hypot(goal[0]-n[0],goal[1]-n[1])
    pq=[(h(start),0.0,start)]
    g={start:0.0}; parent={}
    moves=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while pq:
        _,gc,cur=heapq.heappop(pq)
        if cur==goal:
            p=[cur]
            while p[-1] in parent: p.append(parent[p[-1]])
            return np.asarray(p[::-1],dtype=float)
        if gc > g[cur] + 1e-9:
            continue
        for dx,dy in moves:
            nb=(cur[0]+dx,cur[1]+dy)
            if not grid.free(*nb): continue
            ng=gc+math.hypot(dx,dy)
            if ng < g.get(nb,float("inf")):
                g[nb]=ng; parent[nb]=cur
                heapq.heappush(pq,(ng+h(nb),ng,nb))
    return None
