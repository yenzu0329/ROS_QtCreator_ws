import math
import copy
import sys
import rospy
from ros_gui.msg import Paths
from ros_gui.msg import PointXY
from xml.sax.xmlreader import InputSource
sys.path.append("/home/lab605/ros_qtcreator_ws/src/ros_gui/src")
from maze import maze

list_path=[]
min_path = 0
index = 0
pub = rospy.Publisher('/ros_gui/paths', Paths, queue_size=1)
# start point
list_start = (600, 880)
# end point
# list_end = (221, 288)

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    finded_list = []

    # Add the start node
    open_list.append(start_node)

    
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f and item.position not in finded_list:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        finded_list.append(current_node.position)
        #print(current_index, current_node.f, current_node.position)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[0]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            find = False
            for closed_child in closed_list:
                if child.position == closed_child.position:
                    find = True
                    break
            if find:    
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            find = False
            for open_node in open_list:
                if child.position == open_node.position and child.g > open_node.g:
                    find = True
                    break
            if find:
                continue
            
            # Add the child to the open list
            open_list.append(child)

def send_msg(path, cid):
    msg = Paths()
    if path == 0 and cid == 0:
        msg.point.append(PointXY())
        pub.publish(msg)
        rospy.loginfo(msg)
        return
    for p in path:
        point = PointXY()
        point.x = p[1]
        point.y = p[0]
        point.cab_id = cid
        msg.point.append(point)
    #rospy.loginfo(msg)
    pub.publish(msg)

def plan_path(datas):
#696X936(columnXrow)
    global list_start
    column=[]
    row=[]
    cab_id=[]
    sort_cab_id=[]
    all_path=[]
    all_copy=[]
    all_path_list=[]

    start_pos = datas.point.pop(0)
    list_start = (int(start_pos.y), int(start_pos.x))
    for point in datas.point:
        column.append(int(point.y))
        row.append(int(point.x))
        cab_id.append(point.cab_id)
        print(maze[int(point.y)][int(point.x)])
    q=len(column)
    print(column)
    print(row)
    print(cab_id)
    
    k=0
    n=len(column)-1
    print('start', 0, list_start)
    for k in range(n):
        list_path_len = []
        i=0
        while(i<n+1-k):                             
            start = list_start
            end = (column[i], row[i])
            path = astar(maze, start, end)
            list_path.append(path)
            arr_length = len(path)
            list_path_len.append(arr_length) 
            i+=1
        min_path = min(list_path_len)
        index = list_path_len.index(min_path)
        end = (column[index], row[index])
        list_start = (column[index], row[index])
        print('start', k+1, list_start)              
        all_copy.append(list_path[index])
        all_path = copy.deepcopy(all_copy)
        all_path_list.append(all_path[k])
        column.pop(index)
        row.pop(index)
        sort_cab_id.append(cab_id[index])
        cab_id.pop(index)
        list_path.clear()
        path.clear()

    start = list_start
    end = (column[0], row[0])
    sort_cab_id.append(cab_id[0])
    path = astar(maze, start, end)        
    all_path_list.extend([path])
    print('end_checkout counter',n+1,end)     
    #print('path',n+1,path)
    j=0
    i=0
    k=0
    a=0
    p=[]
    
    
    for a in range(q):
        p_length = len(all_path_list[a])
        p.append(p_length)
    x=0
    s0=1
    s1=1
    s2=1
    s3=1
    while x<q:   
        for j in range(p[x]-2):
            if all_path_list[x][i][0] == all_path_list[x][i+1][0] :
                if all_path_list[x][i+1][0] != all_path_list[x][i+2][0] :
                    j+=1
                    i+=1    
                else :
                    del all_path_list[x][i+1]
                    p[x]=p[x]-1
                    j+=1
            elif all_path_list[x][i][1] == all_path_list[x][i+1][1] :
                if all_path_list[x][i+1][1] != all_path_list[x][i+2][1] :
                    j+=1
                    i+=1   
                else :
                    del all_path_list[x][i+1]
                    j+=1
                    p[x]=p[x]-1
            elif (all_path_list[x][i][0] == all_path_list[x][i+1][0]+s0) & (all_path_list[x][i][1] == all_path_list[x][i+1][1]+s0) :
                if (all_path_list[x][i+1][0] == all_path_list[x][i+2][0]+1) & (all_path_list[x][i+1][1] == all_path_list[x][i+2][1]+1)  :
                    del all_path_list[x][i+1]
                    j+=1
                    p[x]=p[x]-1
                    s0+=1
                else:
                    j+=1
                    i+=1
                    s0=1
            elif (all_path_list[x][i][0] == all_path_list[x][i+1][0]-s1) & (all_path_list[x][i][1] == all_path_list[x][i+1][1]-s1) :
                if (all_path_list[x][i+1][0] == all_path_list[x][i+2][0]-1) & (all_path_list[x][i+1][1] == all_path_list[x][i+2][1]-1)  :
                    del all_path_list[x][i+1]
                    j+=1
                    p[x]=p[x]-1
                    s1+=1
                else:
                    j+=1
                    i+=1
                    s1=1
            elif (all_path_list[x][i][0] == all_path_list[x][i+1][0]-s2) & (all_path_list[x][i][1] == all_path_list[x][i+1][1]+s2) :
                if (all_path_list[x][i+1][0] == all_path_list[x][i+2][0]-1) & (all_path_list[x][i+1][1] == all_path_list[x][i+2][1]+1)  :
                    del all_path_list[x][i+1]
                    j+=1
                    p[x]=p[x]-1
                    s2+=1
                else:
                    j+=1
                    i+=1
                    s2=1        
            elif (all_path_list[x][i][0] == all_path_list[x][i+1][0]+s3) & (all_path_list[x][i][1] == all_path_list[x][i+1][1]-s3) :
                if (all_path_list[x][i+1][0] == all_path_list[x][i+2][0]+1) & (all_path_list[x][i+1][1] == all_path_list[x][i+2][1]-1)  :
                    del all_path_list[x][i+1]
                    j+=1
                    p[x]=p[x]-1
                    s3+=1
                else:
                    #print(s)
                    j+=1
                    i+=1
                    s3=1        
                
            else :
                i+=1
                j+=1
                s0=1
                s1=1
                s2=1
                s3=1
        x+=1
        j=0
        i=0
      
    #print(all_path_list)
    for i, path in enumerate(all_path_list):
        print(path, sort_cab_id[i])
        send_msg(path, sort_cab_id[i])
    send_msg(0, 0)

def setup_ros():
    rospy.init_node('astar')
    rospy.Subscriber('/ros_gui/dests', Paths, plan_path, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    setup_ros()
