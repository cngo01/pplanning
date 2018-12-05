import math
import sys
import numpy as np

global_variables = {}
class Node:
    def __init__(self, x, y, nodeid):
        self.x = x
        self.y = y
        self.cost = math.inf
        self.id = nodeid
        self.visited = False
        self.parent = None

    def __str__(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.cost)

    def getx(self):
        return self.x

    def gety(self):
        return self.y

    def getID(self):
        return self.id

    def addPrev(self, xPrev, yPrev):
        self.xPrev = xPrev
        self.yPrev = yPrev

    def getPrev(self):
        return (self.xPrev, self.yPrev)

    def setCost(self, cost):
        self.cost = cost

    def getCost(self):
        return self.cost

    def markVistied(self):
        self.visited = True

    def checkVisited(self):
        return self.visited

    def addParentNode(self, parentNode):
        self.parent = parentNode

    def getParentNode(self):
        return self.parent
def calc_obstacle_cost(x,y, ob,robot_radius,Ps):
    #calc_obstacle_cost(neighborNode.getx(),neighborNode.gety(), ob,robot_radius,Ps)
    # calc obstacle cost inf: collistion, 0:free
    minr = float("inf")
    #print(x,y)
    for i in range(len(ob[:, 0])):
        ox = ob[i, 0]
        oy = ob[i, 1]
        dx = x - ox
        dy = y - oy
        
        r = math.sqrt(dx**2 + dy**2)
        if r == 0:
            
            return 10000
        elif r <= robot_radius:
            r_neg=Ps[i]
            #print(r_neg)
            return r_neg#float("Inf")  # collisiton
        
        if minr >= r:
            minr = r
   
    return 1.0 / minr  # OK

def AStar(nSx,nSy,nTx,nTy,to_goal_cost_gain,robot_radius):
    print('here')
    # CREATE RISK MAP
    #map = np.zeros((2,2))
    ob = np.matrix([[0, 2],
                [2.5, 8.0],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [7.0, 6.2],
                [12.0, 12.0],
                [12.0, 14.0],
                [13.0, 13.0]
                ])
    gmap = np.zeros([15,15])
    Ps=([90.17962699, 18.89381439, 20.28729583, 93.90378764, 63.28318137,
           85.05038426, 29.05149061, 38.55092059, 87.28532153, 19.05553487,
           20.01487473, 40.46982651, 59.25035024])
    
    for i in range(len(ob)):
        gmap[i,i]=Ps[i]
    print('check here')
    print(gmap)

    #map = np.array([[0.9, 0, 0],[0, 0.9, 0],[0, 0.3, 0.1]])
    #size_map_x,size_map_y = map.shape()
    cost_map = gmap*100
    print(gmap)
    xdim, ydim = np.shape(gmap)
    print('xdim:',xdim)
    print('ydim:',ydim)
    nTx = xdim-1 # 0-based index
    nTy = ydim-1
    startID = getID(nSx,nSy,ydim)
    startNode = Node(nSx,nSy,startID)
    startNode.setCost(0)
    # create a priority queue to queue Node
    #pQueue = PriorityQueue(maxsize=0)
    #pQueue.put((0,startNode)
    priority_dict = {}
    priority_dict[0] = [startNode]
    #print(startNode)
    #print(priority_dict)
    cur = sorted(priority_dict.items())[0]
    curcost, curnode = cur
    #print(curnode[0])
    #pQueue.put((1,(nSx,nSy)))
    #current = pQueue.get()[1]
    #x,y = current
    #print(current)
    #print(x)
    #print(y)

    ## CHECK INITIALIZE EVERY VERTEX TO HAVE A COST OF INFINITY
    #while (pQueue.empty() == False):
    #    print(pQueue.get()[1])
    # pQueue.get()  ONLY WORK IF PQUEUE IS NOT EMPTY OTHERWISE STUCK IN LOOP
    #print(pQueue.empty())
    global_variables['vistitedID_set'] = set() # consist of nodeID
    nodeID_pqueue_set = {}
    IDtoNode_dict = {}
    IDtoNode_dict[startID] = startNode

    #while (pQueue.empty() == False):
    #while bool(priority_dict):
    #while len(priority_dict) > 0:
    count = 0
    finalPath = []
    while bool(priority_dict) or count < 30:
        #print('count',count)
        count += 1
        #currentNode = pQueue.get()[1]
        currentNode_cost, nodelist = sorted(priority_dict.items())[0]
        currentNode = nodelist[0]
        currentNode.markVistied()
        #visted_set.add(currentNode.getID())
        current_x = currentNode.getx()
        current_y = currentNode.gety()
        #print(current_x, current_y)
        global_variables['vistitedID_set'].add(currentNode.getID())
        # DEQUEUE FROM DICT
        if len(nodelist) > 1:
            priority_dict[currentNode_cost] = nodelist[1:]
        else:
            del priority_dict[currentNode_cost]
        if (current_x, current_y) == (nTx, nTy):
            finalPath = reconstructPath(currentNode,nSx,nSy)
            break
        neighbors_list = getNeighbors(current_x,current_y,xdim,ydim,nTx,nTy,to_goal_cost_gain) # return (node, cost of motion)
        #print(neighbors_list)
        innercount = 0
        for ( nID, neighborNode, cost_motion) in neighbors_list:
            #print('innercount',innercount)
            innercount += 1
            if neighborNode.checkVisited() == False:
                obstacle_cost=calc_obstacle_cost(neighborNode.getx(),neighborNode.gety(), ob,robot_radius,Ps)
                new_cost = currentNode_cost + cost_motion + obstacle_cost#cost_map[neighborNode.getx()][neighborNode.gety()]
                #new_cost = currentNode_cost + cost_motion + cost_map[neighborNode.getx()][neighborNode.gety()]
                if neighborNode.getCost() > new_cost:
                    neighborNode.addPrev(current_x, current_y)
                    neighborNode.addParentNode(currentNode)
                    if neighborNode.getCost() != math.inf:
                        # neighborNode already in priority_dict
                        # remove that elem and add new one
                        priority_dict[neighborNode.getCost()].remove(neighborNode)
                    if new_cost not in priority_dict:
                        # this cost value is not in dict -> create new key:value
                        priority_dict[new_cost] = [neighborNode]
                    else:
                        # this cost value is in dict
                        priority_dict[new_cost].append(neighborNode)
                    neighborNode.setCost(new_cost)
    print('finalPath')
    print(finalPath)
    return finalPath
def calc_to_goal_cost(x,y,nTarget_x,nTarget_y, to_goal_cost_gain):
    # calc to goal cost. It is 2D norm.

    dy = nTarget_x - x
    dx = nTarget_y - y
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = to_goal_cost_gain * goal_dis

    return cost

def getNeighbors(cur_x ,cur_y, xsize, ysize,nTarget_x,nTarget_y,to_goal_cost_gain):
    neighbor_list = [(x,y) for x in range(cur_x-1,cur_x+2) for y in range(cur_y-1,cur_y+2) if (x,y) != (cur_x,cur_y) and 0<=x<xsize and 0<=y<ysize]
    #print(neighbor_list)
    final_list = []
    for (x,y) in neighbor_list:
        nodeID = getID(x,y,ysize)
        if nodeID in global_variables['vistitedID_set']:
            continue
        #print (x,y)
#        if x != cur_x and y != cur_y:
#            cost = 5
#        else:
#            cost = 40
        cost=calc_to_goal_cost(x,y,nTarget_x,nTarget_y,to_goal_cost_gain)
        #print(cost)
        # NEW RETURN: RETURN (NODE,NODEID,COST_MOTION)
        final_list.append(( nodeID, Node(x,y,nodeID) , cost ))
        #final_list.append((x,y,getID(x,y,ysize),cost))
    #print(final_list)
    return final_list


def reconstructPath(curNode,startX,startY):
    print('reconstruct')
    path = []
    path.append((curNode.getx(),curNode.gety()))
    counter = 1
    while ((curNode.getx(),curNode.gety()) != (startX,startY)):
        #print(counter)
        curNode = curNode.getParentNode()
        #print(curNode)
        path.append((curNode.getx(),curNode.gety()))
        counter += 1
    print('FINISHED')
    return path

def getID(x,y,ysize):
    return x*ysize + y

def main(to_goal_cost_gain,robot_radius):
#    if len(sys.argv) != 3:
#        raise Exception("usage: python pathv1.py n_grid_x n_grid_y")
#    gridsize_x = int(sys.argv[1])
#    gridsize_y = int(sys.argv[2])
    gridsize_x=15
    gridsize_y=15
    #print(gridsize_x)
    #print(type(gridsize_x))
    #print(gridsize_y)
    #print(type(gridsize_y))
    nStart_x = 0
    nStart_y = 0
    nTarget_x = gridsize_x-1
    nTarget_y = gridsize_y-1

    finalPath=AStar(nStart_x,nStart_y,nTarget_x,nTarget_y,to_goal_cost_gain,robot_radius)
    ## import drone_env.py to create gridworld

    ## TEST CREATENODE CLASS
    #x = Node(2,2)
    #print(x)
    #y = Node(3,3)
    #print(y)
    #x.addPrev(4,4)
    #test = x.getPrev()
    #test = Node.getPrev(x)
    #print(x.getPrev())
    #print(x.getPrev) doesn't work
    #Node.addPrev(x,1,1)
    #print(Node.getPrev(x)) work
    #Node.addCost(x,10)
    #print(x)
    #print(Node.getPrev(x))
    return finalPath


if __name__ == '__main__':
    finalPath=main(to_goal_cost_gain,robot_radius)