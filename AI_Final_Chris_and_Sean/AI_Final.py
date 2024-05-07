########################################################
#### Pathfinding using A* in a hospital.
#### Chris Nolan, Sean Rock 
#### CSC 362 Artificial Intelligence
#### May 7
#### AI Final Project - Robot nurse agent finds its way through matrix representation of a hospital floor 
####                    to make deliveries to user designated target destination. It returns all locations
####                    that could not be reached.
#### AI, Spring 2024
########################################################
import tkinter as tk
import re
from queue import PriorityQueue

######################################################

#### A cell stores f(), g() and h() values
#### A cell is either open or part of a wall
######################################################

class Cell:
    #### Initially, arre maze cells have g() = inf and h() = 0
    def __init__(self, x, y, is_wall=False):
        self.x = x
        self.y = y
        self.reached =False
        self.is_wall = is_wall
        self.g = float("inf")
        self.h = 0
        self.f = float("inf")
        self.parent = None

    #### Compare two cells based on their evaluation functions
    def __lt__(self, other):
        return self.f < other.f


######################################################
# A maze is a grid of size rows X cols
######################################################
class MazeGame:
    def __init__(self, root, maze, startingPos, destList):
        self.root = root
        # set the hostpital matrix representation
        self.maze = maze
        self.rows = len(maze)
        self.cols = len(maze[0])
        # set agent position to the starting position
        self.agent_pos = startingPos  
        # set initial destination
        self.goal_pos = destList[0]
        # initialize cells
        self.cells = [[Cell(x, y, maze[x][y] == -1 or maze[x][y] == 13) for y in range(self.cols)] for x in range(self.rows)]
        
        #### Start state's initial values for f(n) = g(n) + h(n) 
        self.cells[self.agent_pos[0]][self.agent_pos[1]].g = 0
        self.cells[self.agent_pos[0]][self.agent_pos[1]].h = self.heuristic(self.agent_pos)
        self.cells[self.agent_pos[0]][self.agent_pos[1]].f = self.heuristic(self.agent_pos)

        #### The maze cell size in pixels
        self.cell_size = 20
        self.canvas = tk.Canvas(root, width=self.cols * self.cell_size, height=self.rows * self.cell_size, bg='white')
        self.canvas.pack()

        self.draw_maze()
        self.canvas.create_rectangle(startingPos[1] * self.cell_size, startingPos[0] * self.cell_size, (startingPos[1] + 1) * self.cell_size, (startingPos[0] + 1) * self.cell_size, fill="DarkOrchid4")
        self.color = 0
        self.success = False
        self.current_pass = True
        self.fails = []
        self.l1 = []    # list to hold every step for each location traversal
        self.l2 = []    # list to hold and then reverse the current path list and then append to l1
        self.end_node =[]
        #### Display the optimum path in the maze
        self.find_path()
        for n in range(len(destList)):
            for x in range(len(self.maze)):
                for y in range(len(self.maze[0])):
                    self.cells[x][y].g = float("inf")
                    self.cells[x][y].h = 0
                    self.cells[x][y].f = float("inf")
            if self.current_pass:
                self.agent_pos = self.goal_pos 
            else:
                self.current_pass= True
            self.goal_pos = destList[n]
            #### Start state's initial values for f(n) = g(n) + h(n) 
            self.cells[self.agent_pos[0]][self.agent_pos[1]].g = 0
            self.cells[self.agent_pos[0]][self.agent_pos[1]].h = self.heuristic(self.agent_pos)
            self.cells[self.agent_pos[0]][self.agent_pos[1]].f = self.heuristic(self.agent_pos)
            self.find_path()
        
        if self.success and not len(self.l1)==0:
            self.do_one_frame( 0)
            print("Success")
            for x in self.fails:
                print("Failed to reach:")
                print(x)
        else:
            print("Failure: Unable to reach the following locations")
            for x in self.fails:
                print(x)

    ############################################################
    #### Sets the color of the tile based off the hospital ward
    ############################################################
    def draw_maze(self):
        for x in range(self.rows):
            for y in range(self.cols):
                if self.maze[x][y] == 1:
                    color = 'blue'
                elif self.maze[x][y] == 0:
                    color = 'white'
                elif self.maze[x][y] == 13:
                    color = 'black'
                elif self.maze[x][y] == -1:
                    color = 'gray'
                elif self.maze[x][y] == 2:
                    color = 'red4'
                elif self.maze[x][y] == 3:
                    color = 'yellow'
                elif self.maze[x][y] == 4:
                    color = 'dark slate gray'
                elif self.maze[x][y] == 5:
                    color = 'teal'
                elif self.maze[x][y] == 6:
                    color = 'green'
                elif self.maze[x][y] == 7:
                    color = 'purple'
                elif self.maze[x][y] == 8:
                    color = 'orange'
                elif self.maze[x][y] == 9:
                    color = 'tomato2'
                elif self.maze[x][y] == 10:
                    color = 'sienna3'
                elif self.maze[x][y] == 11:
                    color = 'OliveDrab3'
                elif self.maze[x][y] == 12:
                    color = 'aquamarine'
                self.canvas.create_rectangle(y * self.cell_size, x * self.cell_size, (y + 1) * self.cell_size, (x + 1) * self.cell_size, fill=color)

    ############################################################
    #### Manhattan distance
    ############################################################
    def heuristic(self, pos):
        return (abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1]))

    ############################################################
    #### A* Algorithm
    ############################################################
    def find_path(self):
        open_set = PriorityQueue()
        
        #### Add the start state to the queue
        open_set.put((0, self.agent_pos))
        fail = True
        #### Continue exploring until the queue is exhausted
        while not open_set.empty():
            current_cost, current_pos = open_set.get()
            current_cell = self.cells[current_pos[0]][current_pos[1]]

            #### Stop if goal is reached
            if current_pos == self.goal_pos:
                print(self.goal_pos)
                fail = False        # note that this end_point could  be reached
                self.success = True # note that at least one point was reached by the robot
                self.reconstruct_path()
                break

            
            #### Agent goes E, W, N, and S, whenever possible
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                new_pos = (current_pos[0] + dx, current_pos[1] + dy)

                if 0 <= new_pos[0] < self.rows and 0 <= new_pos[1] < self.cols and not self.cells[new_pos[0]][new_pos[1]].is_wall:
                
                    #### The cost of moving to a new position is 1 unit
                    new_g = current_cell.g + 1
                    
                    
                    if new_g < self.cells[new_pos[0]][new_pos[1]].g:
                        ### Update the path cost g()
                        self.cells[new_pos[0]][new_pos[1]].g = new_g
                        
                        ### Update the heurstic h()
                        self.cells[new_pos[0]][new_pos[1]].h = self.heuristic(new_pos)
                        
                        ### Update the evaluation function for the cell n: f(n) = g(n) + h(n)
                        self.cells[new_pos[0]][new_pos[1]].f = new_g+self.cells[new_pos[0]][new_pos[1]].h
                        self.cells[new_pos[0]][new_pos[1]].parent = current_cell
                        
                        #### Add the new cell to the priority queue
                        open_set.put((self.cells[new_pos[0]][new_pos[1]].f, new_pos))
        if fail:
            self.fails.append(self.goal_pos)
            self.current_pass = False
                        
                        

    ############################################################
    #### This is for the GUI part. No need to modify this unless
    #### screen changes are needed.
    ############################################################
    def reconstruct_path(self):
        # save target destinations
        self.end_node.append(self.goal_pos)
        # set the current cell
        current_cell = self.cells[self.goal_pos[0]][self.goal_pos[1]]
        x, y = current_cell.x, current_cell.y
        # draw the target destination
        self.canvas.create_rectangle(y * self.cell_size, x * self.cell_size, (y + 1) * self.cell_size, (x + 1) * self.cell_size, fill='green2')
        # move to next cell in path
        current_cell = current_cell.parent
        # loop through each tile in the path and add each cell to l2
        while current_cell.parent:
            x, y = current_cell.x, current_cell.y
            current_cell2 = current_cell.parent
            current_cell.parent = None
            current_cell = current_cell2
            self.l2.append((x, y))

        # reverse l2 after adding each tile in the path so that it starts with the starting position
        self.l2.reverse()

        # apppend each tile in the current path to the total path list l1
        for x in self.l2:
            self.l1.append(x)

        #clear the current path list l2
        self.l2.clear()

    # recursive function that draws the path. It cycles through 3 colors (gold, pink, blue)
    def do_one_frame(self, index):
        x, y = self.l1[index][0], self.l1[index][1]
        boo = False
        # draws nothing if it its an end node. Also switch colors as an end node means a new path
        for b in self.end_node:
            if b[0] == x and b[1]==y:
                boo = True
        if boo:
            if self.color == 2:
                self.color= 0
            else:
                self.color= self.color+1
        # draw gold path if current color index is 0
        elif self.color == 0:
            self.canvas.create_rectangle(y * self.cell_size, x * self.cell_size, (y + 1) * self.cell_size, (x + 1) * self.cell_size, fill='gold3')
        # draw blue path if current color index is 0  
        elif self.color == 1:
            self.canvas.create_rectangle(y * self.cell_size, x * self.cell_size, (y + 1) * self.cell_size, (x + 1) * self.cell_size, fill='midnight blue')
        # draw pink path if current color index is 0
        else:
            self.canvas.create_rectangle(y * self.cell_size, x * self.cell_size, (y + 1) * self.cell_size, (x + 1) * self.cell_size, fill='orchid1')
        # if tiles remain move on to the next tile after a short pause
        if (index < len(self.l1)-1):
            self.canvas.after(100, self.do_one_frame, index+1)





                  

############################################################
#### Hospital matrix representation
#### -1 - Out of Hospital
#### 0  - Hallway
#### 1  - Maternity Ward
#### 2  - General Ward
#### 3  - Emergency
#### 4  - Admissions
#### 5  - Isolation Ward
#### 6  - Oncology
#### 7  - Burn Ward
#### 8  - ICU
#### 9  - Surgical Ward
#### 10 - Hematology 
#### 11 - Pediatric Ward
#### 12 - Medical Ward
#### 13 - Wall
############################################################
maze = [
    [-1,-1,-1,13,13,13,13,13,13,13,13,13,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1,13, 1, 1, 1, 1,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1,13, 1, 1, 1, 1,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1,13, 1, 1, 1, 1,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13,13,13, 1,13, 1,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,13, 2, 2,13, 2, 2,13, 2,13, 2,13, 2, 2, 2, 2,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,13, 2, 2,13, 2, 2,13, 2,13, 2,13, 2, 2, 2, 2,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,13, 1, 1, 1, 1, 1, 1, 1, 1,13, 2,13, 2, 2,13, 2, 2,13, 2,13, 2,13, 2, 2, 2, 2,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [13,13,13,13,13, 1, 1,13,13, 2, 2,13,13, 2,13, 2,13,13, 2,13,13, 2,13, 2,13,13, 2, 2,13,13,13,13,13,13,13,13,13,13,13,13],
    [13, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0,13, 3, 3, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0,13,13,13, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0,13, 5,13, 2,13, 2,13, 2, 2, 2, 2, 2,13,13, 2,13, 2, 2,13, 2, 2, 2,13,13, 0, 0,13, 3, 3, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0,13,13,13, 2,13,13,13, 2, 2, 2, 2, 2, 2,13, 2,13, 2, 2,13, 2, 2, 2,13,13, 0, 0,13,13,13, 3, 3,13, 4,13,13],
    [13, 0, 0, 0, 0,13, 5,13, 2,13, 2,13, 2, 2, 2, 2, 2,13,13, 2,13,13,13,13, 2, 2,13, 5, 5, 0, 0,13, 3, 3, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,13,13,13, 0, 0,13, 3, 3, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3,13, 0, 0, 3, 3, 3, 3, 3,13, 4, 4,13],
    [13, 0, 0, 0, 0,13,13, 6,13,13,13, 2,13, 2,13, 2,13, 7,13, 2,13, 2,13, 2, 2,13, 3, 3,13, 0, 0,13,13,13,13,13,13,13,13,13],
    [13, 0, 0, 0, 0,13, 6, 6, 6, 6,13, 2,13, 2,13, 2,13, 7,13, 2,13, 2,13, 2, 2,13,13,13, 5, 0, 0,13, 8,13, 4, 4, 4, 4, 4, 4],
    [13, 0, 0, 0, 0,13, 6, 6, 6, 6,13,13,13,13,13,13,13, 7,13,13,13,13,13, 2, 2, 3, 3,13,13, 0, 0, 8, 8, 8,13,13,13,13,13,13],
    [13, 0, 0, 0, 0,13, 6, 6, 6, 6,13, 7, 7, 7, 7, 7, 7, 7, 7,13, 2, 2, 2, 2, 2,13,13, 6, 6, 0, 0,13,13, 8, 8, 8, 8, 8, 8,13],
    [13, 0, 0, 0, 0,13,13,13,13, 6,13, 7,13,13,13,13,13, 7,13,13,13,13,13, 0, 0,13, 5,13,13, 0, 0,13, 8, 8, 8,13,13, 8, 8,13],
    [13,13,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8,13,13,13],
    [-1,-1,13, 0, 0,13,13,13,13, 6,13,13, 4,13, 4,13,13,13,10,13,13,13,13, 0, 0,13,13, 9,13, 0, 0,13,13,13,13,13,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13, 6, 6,13, 6, 6,13, 4,13, 4, 4,13,10,10,10,10,10,13, 0, 0,13, 9, 9,13, 6, 6,13, 6, 6,13, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13, 6, 6,13, 6, 6,13, 4,13, 4, 4,13,13,10,10,13,10,13, 0, 0,13, 9, 9,13, 6, 6,13, 6, 6,13, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13, 6, 6, 6, 6, 6, 6,13,11,13,13,10,10,10,10,13,13,13, 0, 0,13, 9, 9,13, 6, 6,13, 6,13,13, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0, 5,13,13,13, 6, 6, 6,13,11,11,13,10,10,10,13,11,11,11, 0, 0,13, 9, 9,13, 6, 6, 6, 6, 6, 6, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13, 6, 6, 6, 6,13,13,11,11,11,11,13,13,13,13,11,11,13, 0, 0,13, 9, 9,13, 6, 6, 6, 6, 6, 6, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0, 6, 6, 6, 6, 6,13,11,11,11,11,11,11,11,11,11,11,11,13, 0, 0,13, 9, 9,13, 6, 6, 6, 6, 6, 6, 6,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13,13,13,13,13,13,13,11,13,13,13,13,13,11,13,13,11,13, 0, 0,13, 9, 9, 9,13,13,13,13,13,13,13,13,-1,-1,-1],
    [-1,-1,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,13,-1,-1,-1],
    [-1,-1,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13,13,11,13,11,13,11,13,13,11,13,11,13,11,13,13,11,13,13,11,13,13, 9,13,12,13,13,12,13, 9,13,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13,11,11,11,11,13,11,11,13,11,13,11,13,11,13,11,11,13,11,11,13, 9, 9,13,12,12,13,12,13, 9, 9,13,-1,-1,-1],
    [-1,-1,13, 0, 0,13,13,11,13,13,13,11,11,13,11,13,11,11,11,13,11,11,13,13,11,13, 9, 9,13,12,13,13,13,13, 9, 9,13,-1,-1,-1],
    [-1,-1,13, 0, 0, 5,13,11,11,11,13,11,11,13,11,13,11,13,11,13,11,11,13,11,11,13, 9, 9,13,12,12,12,12,13, 9, 9,13,-1,-1,-1],
    [-1,-1,13, 5,13, 5,13,11,11,11,13,11,11,13,11,13,11,13,11,13,11,11,13,11,11,13, 9, 9,13,12,12,12,12,13, 9, 9,13,-1,-1,-1],
    [-1,-1,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,-1,-1,-1],


    

]



############################################################
#### The mainloop activates the GUI.
############################################################

#Store the priority of each ward for lookup
priority5 = [8,6,3,7]
priority4 = [9,1]
priority3 = [10,11]
priority2 = [12,2]
priority1 = [4,5]

#List to store the locations with the same priority
priority5Dest = []
priority4Dest = []
priority3Dest = []
priority2Dest = []
priority1Dest = []

#Store the validation state and the locations of goals
badInput = True
destList = []

#Custom sort to group rooms of same priority within the same ward
def groupWards(dest):
    return maze[dest[0]][dest[1]]

#Sort the destination list by Priority then group by ward
def findPriority(destList):

    finalList = []

    #Group by Priority
    for dest in destList:
        if maze[dest[0]][dest[1]] in priority5:
            priority5Dest.append(dest)
        elif maze[dest[0]][dest[1]] in priority4:
            priority4Dest.append(dest)
        elif maze[dest[0]][dest[1]] in priority3:
            priority3Dest.append(dest)
        elif maze[dest[0]][dest[1]] in priority2:
            priority2Dest.append(dest)
        elif maze[dest[0]][dest[1]] in priority1:
            priority1Dest.append(dest)
        else:
            priority1Dest.append(dest)

    #Group by ward
    priority5Dest.sort(key = groupWards)
    priority4Dest.sort(key = groupWards)
    priority3Dest.sort(key = groupWards)
    priority2Dest.sort(key = groupWards)
    priority1Dest.sort(key = groupWards)

    #Combine the lists into one
    for dest in priority5Dest:
        finalList.append(dest)
    
    for dest in priority4Dest:
        finalList.append(dest)

    for dest in priority3Dest:
        finalList.append(dest)
    
    for dest in priority2Dest:
        finalList.append(dest)
    
    for dest in priority1Dest:
        finalList.append(dest)

    return finalList

    
#Control
#(1,6),(20,37),(1,10),(36,7),(10,10),(11,6)

#(31,20),(11,6),(10,10),(36,7),(1,10),(20,37)

#one out of hospital node
# (15,6),(26,5),(36,27),(17,21),(12,34),(36,30),(12,37)
while badInput:

    #Reset loop variables and ask for user input
    badInput = False
    destList.clear()
    str = input("Enter the destinations for the nurse with the first entry being the starting possition Ex. (1,1),(2,2),(3,3): ")
    str = str.replace(" ","")

    #Regex check for decimals or negative numbers
    pattern = r'(?:[-.][0-9]+)+'
    match = re.search(pattern, str)
    if match:
        print(str, "<- Negative or decimal input found, please enter positive numbers only", match.group())
        badInput = True
        continue
    else:
        badInput = False

    #Regex check for letters
    pattern = r'[a-zA-Z]'
    match = re.search(pattern, str)
    if match:
        print(str, "<- Letter found, please enter numbers only", match.group())
        badInput = True
        continue
    else:
        badInput = False

    #Extract the cords from the input
    pattern = r'\(([^\)]+)\)'
    substrings = re.findall(pattern, str)

    #Check to see there is at least one start and one goal
    if len(substrings) < 2:
        print('Please enter at least one starting location and one destination')
        badInput = True
        continue
    else:
        badInput = False

    #Split the cords into an x and y to be stored in the list
    for string in substrings:
        splitString = string.split(',')
        destList.append((int(splitString[0]), int(splitString[1])))

    #Check for an array out of bounds as well as a goal in a wall or outside the hospital
    for dest in destList:
        if(dest[0] >= 40 or dest[1] >= 38):
            print('Please enter a location in the maze')
            badInput = True
            break
        elif(maze[dest[0]][dest[1]] == 13):
            print('Please enter a location that is not a wall')
            badInput = True
            break
        elif(maze[dest[0]][dest[1]] == -1):
            print('Please enter a location inside the hospital')
            badInput = True
            break

    if(badInput):
        continue

    #Print out the list inputed
    print('Before sort:', destList)
    startingPos = destList[0]
    destList.pop(0)

    #Sort our list
    destList = findPriority(destList)

    #Show sorted list
    print('Starting Pos:', startingPos)
    print('After sort:',destList)
else:

    #Start the GUI
    root = tk.Tk()
    root.title("Hospital AI Maze")
    game = MazeGame(root, maze,startingPos, destList)
    root.mainloop()