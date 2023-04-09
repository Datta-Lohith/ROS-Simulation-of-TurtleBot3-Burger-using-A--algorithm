import numpy as np
import time
import cv2 as cv
import heapq
from tqdm import tqdm
import math

nodes={}                        # To store all nodes
start_node=None                 # To store the start node
end_node=None                   # To store the end node

animation=False                 # To display the animation of the path planning

RPM1=50                        # To store the RPM of the left wheel(default: 50)
RPM2=100                       # To store the RPM of the right wheel(default: 100)
r=7                            # Radius of the robot(Approx. 7cm)
clearance=1 + r                # To store the clearance of the robot(default: 8cm)
R=3.8                          # Wheel radius
L=11.7                         # Distance between wheels
invert_y=200                   # To invert the y axis

# Visualizing the path by writing video
frameSize = (600, 200)
fourcc = cv.VideoWriter_fourcc('m','p','4','v')
out  = cv.VideoWriter('output_part01.mp4', fourcc, 250, frameSize)

# Function to create the map and obstacle map
def create_map():
    global clearance
    # Main map for display
    map = np.full((200, 600,3),(51,51,44), dtype=np.uint8)

    # Obstacle map for path planning
    obs_map = np.full((200, 600),255, dtype=np.uint8)

    check_input=True
    while check_input:
        clearance=int(input("Please provide the clearance value(mm):"))
        if clearance<0:
            print("Please enter a positive value\n")
            continue
        clearance/=10
        clearance+=np.round(r)
        clearance=int(clearance)
        check_input=False
        
    print("\rTotal clearance value(cm):",clearance)
    
    # Draw rectangles with clearance
    rectangles=[((150-clearance, 0), (165+clearance, 125+clearance)),((250-clearance, 75-clearance), (265+clearance, 200+clearance)),
            ((0, 0), (600,clearance)),((0, 0), (clearance, 200)),((0, 200-clearance), (600, 200)),((600-clearance, 0), (600, 200))]
    for rectangle in rectangles:
        cv.rectangle(map, rectangle[0], rectangle[1],(222,228,203), -1)
        cv.rectangle(obs_map, rectangle[0], rectangle[1],(0,0,0), -1)

    # Draw circle with clearance
    cv.circle(map, (400, 90), 50+clearance,(222,228,203), -1)
    cv.circle(obs_map, (400, 90), 50+clearance,(0,0,0), -1)

    # Draw rectangles
    cv.rectangle(map, (150, 0), (165, 125), (136,131,14), -1)
    cv.rectangle(map, (250, 75), (265, 200),(136,131,14), -1)

    # Draw circle
    cv.circle(map, (400, 90), 50,(136,131,14), -1)
    
    # cv.imshow('Map',map)
    # cv.waitKey(0)   
    return map,obs_map

# Function to insert a node into the nodes dictionary
def insert_node(cost=None,node=None,parent=None,curve=None,action=None):    
    if len(nodes)==0:
        # Inserting obstacle nodes into the nodes dictionary
        for i in range(600):
            for j in range(200):
                if obs_map[j][i]!=255:
                    nodes.update({(i,j,0):[None,float('inf'),None,None]})
    else:
        nodes.update({node:[parent,cost,curve,action]})
        
# Action functions to move the mobile robot in 8 directions
def Actions(node,RPM1,RPM2):
  
    def action(left,right):
        curve=[]
        t=0
        dt=0.1
        x,y,th=node
        theta=th*np.pi/180
        cost=0
        
        while t<1:
            t+=dt
            xi,yi=int(x),int(y)
            dx=0.5*R*(left+right)*np.cos(theta)*dt
            dy=0.5*R*(left+right)*np.sin(theta)*dt
            x+=dx
            y+=dy
            dth=(R/L)*(right-left)*dt
            theta+=dth
            cost+=np.sqrt(dx**2+dy**2)
            curve.append(((xi,invert_y-yi,dth),(int(x),invert_y-int(y),dth)))
        x,y,th=int(x),int(y),int(theta*180/np.pi)
        cost=int(cost)
        if check_if_duplicate((x,y,th%360)):
            return None,None,None,None
        else:
            return (x,y,th%360),cost,curve,(left,right)
    
    def check_if_duplicate(node_n):
        x,y,th=node_n
        threshold=max(RPM1,RPM2)
        for i in range(-threshold,threshold+1):
            for j in range(-threshold,threshold+1):  
                for k in range(0,13):              
                    if (x+i,y+j,th+k*30) in nodes.keys():
                        return True
        return False
    
    return [action(0,RPM1),action(RPM1,0),action(RPM1,RPM1),action(0,RPM2),
            action(RPM2,0),action(RPM2,RPM2),action(RPM1,RPM2),action(RPM2,RPM1)]

# Function to calculate the total cost of a node f(n)=h(n)+g(n): h(n)=euclidean heuristic from node to end node, g(n)=cost of node   
def total_cost(node):
    i,j,_=node
    return int(math.sqrt((i-end_node[0])**2+(j-end_node[1])**2) +nodes[node][1])

# Function to check if the node is the end node
def check_if_end(node):
    if math.sqrt((node[0] - end_node[0])**2 + (node[1] - end_node[1])**2) <= 5:
        return True
    else:
        return False

# Function to check if the curve is valid
def check_curve(curve):
    for i in range(len(curve)):
        if obs_map[curve[i][0][1]][curve[i][0][0]]==0 or obs_map[curve[i][1][1]][curve[i][1][0]]==0:
            return False
    return True
     
# Tree function to generate the heap tree graph of the nodes using A* Algorithm
def tree():
    global nodes
    open_list=[]
    closed_list=set()                  
    tot_cost=total_cost(start_node)    
    heapq.heappush(open_list,(tot_cost,start_node))
    return_node=None
    # For animation of progress bar
    print('\nSearching for path:')
    for i in tqdm(range(100)):
        while open_list:
            _,current_node=heapq.heappop(open_list)
            current_c2c=nodes[current_node][1]      
            closed_list.add(current_node)                   

            if current_node!=start_node:
                curve=nodes[current_node][2]
                for i in range(len(curve)):
                    cv.line(map,curve[i][0][:2],curve[i][1][:2],(0,0,0),1)
                cv.line(map,curve[-1][1][:2],(current_node[0],invert_y-current_node[1]),(0,0,0),1)
                # cv.arrowedLine(map,(nodes[current_node][0][0],invert_y-nodes[current_node][0][1]),(current_node[0],invert_y-current_node[1]),(0,0,0),1)
                cv.waitKey(1)
                out.write(map)
            
            if animation:
                cv.imshow('Map',map)
                cv.waitKey(1)

            if check_if_end(current_node):
                open_list=None
                return_node=current_node
                break              
            
            for action in Actions(current_node,RPM1,RPM2):
                new_node,cost,curve,act=action
                if new_node is not None and new_node not in closed_list and (new_node[0] in range(0,600)) and (invert_y-new_node[1] in range(0,200)) and obs_map[invert_y-new_node[1]][new_node[0]]==255  and check_curve(curve):
                    new_cost=current_c2c+cost
                    # Checking if node is already explored and if the new cost is less than the previous cost
                    if new_node not in nodes.keys():
                        insert_node(new_cost,new_node,current_node,curve,act)
                        new_total_cost=total_cost(new_node)
                        heapq.heappush(open_list,(new_total_cost,new_node))
                    else:
                        for i in range(len(open_list)):
                            if open_list[i][1]==new_node and open_list[i][0]>new_cost:
                                insert_node(new_cost,new_node,current_node,curve,act)
                                new_total_cost=total_cost(new_node)
                                open_list[i]=(new_total_cost,new_node)
        time.sleep(0.01)
        i+=1
    return return_node
  
# Returns the parent node for a given node    
def get_parent(node):
    return nodes[node][0]

# Returns a path from the end_node to the start_node
def generate_path():
    # Searching starts here
    path=[tree()]
    if path[0] is not None:
        total_cost=nodes[path[0]][1]
        parent=get_parent(path[0])
        while parent is not None:
            path.append(parent)
            parent = get_parent(parent)
        path.reverse()
        print("Path found")
        return path,total_cost
    else:
        print("\nError: Path not found\nTry changing the orientation of the nodes")
        exit()

# Saving the map
def save_map(map):
    print("\nSaving the map:")
    for i,j in zip(range(len(path)),tqdm(range(len(path)-1))):
        curve=nodes[path[i+1]][2]
        for k in range(len(curve)):
            cv.line(map,curve[k][0][:2],curve[k][1][:2],(0,0,255),1)
        # cv.arrowedLine(map,(path[i][0],invert_y-path[i][1]),(path[i+1][0],invert_y-path[i+1][1]),(0,0,255),1)
        if animation:
            cv.imshow('Map',map)
            cv.waitKey(1)
        out.write(map)
    for i in range(500):
        out.write(map)
    print("Map saved as output.mp4\n")
    cv.waitKey(500)
    cv.destroyAllWindows()
    out.release()

# Getting user inputs
def get_inputs():
    global start_node,end_node,RPM1,RPM2,animation

    # Getting the RPMs of the wheels
    rpms=input("Enter the RPM of the wheels(Left:RPM1 Right:RPM2)(eg: 10 15): ")
    rpms=rpms.split()
    RPM1,RPM2=[int(i) for i in rpms]
    
    # Getting the user's choice to visualize the path
    animate=input("Do you want to visualize the path? (y/n): ")
    animation=(lambda x: True if x=='y'or x=='Y' else False)(animate)
    
    # Getting the start node
    check_input=True
    while check_input:
        s_node=input(f"\nNote:'({(-50+clearance+1)/100,(-invert_y+clearance+1)/100}) is the starting point at top left due to clearance on the walls'\nEnter the start node in the format 0 1 30 for (0,1,30): ")
        s_node=s_node.split()
        if len(s_node)==3:
            x,y,th=[(float(i)) for i in s_node]
            x,y=int(x*100),int(y*100)
            x+=50
            y=100-y
            if x>=600 or y>=200 or x<0 or y<0:
                print("Please enter valid coordinates.")
            elif obs_map[y][x]!=255:
                print("Please enter a valid start node(Node in obstacle place).")
            else:
                check_input=False
                start_node=(x,invert_y-y,th%360)
                insert_node(0,start_node,None,None)
                cv.circle(map,(x,y),2,(0,255,0),-1)
        else:
            print("Please enter valid coordinates.")
            
    # Getting the end node
    check_input=True
    while check_input:
        f_node=input(f"\nNote:'({(549-clearance)/100,(99-clearance)/100}) is the ending point at top right corner due to clearance on the walls'\nEnter the end node in the format 0 1 for (0,1) : ")
        f_node=f_node.split()
        if len(f_node)==2:
            x,y=[int(float(i)*100) for i in f_node]
            x+=50
            y=100-y
            if x>=600 or y>=200 or x<0 or y<0 or (x,y)==start_node[:2]:
                print("Please enter valid coordinates.")
            elif obs_map[y][x]!=255:
                print("Please enter a valid end node(Node in obstacle place).")
            else:
                check_input=False
                end_node=(x,invert_y-y)
                cv.circle(map,(x,y),2,(0,0,255),-1)
        else:
            print("Please enter valid coordinates.")

# Main function
if __name__ == "__main__":
    start_time = time.time()
    
    # Getting maps
    map,obs_map=create_map()
    
    # Inserting obstacle nodes into the nodes dictionary
    insert_node()

    # Getting user inputs
    get_inputs()
    
    # Generating the path and the total cost
    path,total_cost=generate_path()   
    
    # Saving the animation of the path generation
    save_map(map)
        
    end_time = time.time()
    
    gazebo_path=[((i[0]-50)/100,(100-i[1])/100,i[2]) for i in path]
    
    print(f"Time taken to execute the code is: {(end_time-start_time)/60} minutes.")
    print("\nTotal cost of the path is: ",total_cost)
    print("\nPath is: ",gazebo_path)
    
