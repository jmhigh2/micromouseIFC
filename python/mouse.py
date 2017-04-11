import json, codecs #for simulated mazes
import numpy as np
from maze import Maze

class Mouse:

    '''Future Additions to Class:
    1. Add 'visited' attribute to each node in array, to visualize where the mouse has been.
    2. Add 'cost' attribute to each node, to take into account the level of difficulty in traversing that cell.
    '''

    #directions of travel in the matrix
    NORTH = (-1, 0) #row up
    SOUTH = (1, 0) #row down
    WEST = (0, -1) #row left
    EAST = (0, 1) #row right

    def __init__(self, start_wall, **kwargs): #expecting a start wall (for start position), a maze object (to store the memorized maze)
            #cont'd - and a maze file in kwargs, that stores the maze to be analyzed
        if ('maze_file' in kwargs): #the maze needed to be solved
            maze_file = codecs.open(kwargs['maze_file'], 'r', encoding='utf-8').read()
            maze_data = json.loads(maze_file)
            horiz_walls = np.array(maze_data['horiz_walls'])
            vert_walls = np.array(maze_data['vert_walls'])
            n = maze_data['n']

            self.horiz_walls_abs = horiz_walls #loading the walls to be analyzed
            self.vert_walls_abs = vert_walls

        self.maze = Maze(n, start="left") #create maze object for mouse to save
        self.visited = np.zeros(n, dtype=bool)

        for a in range(1, n):
            self.visited = np.vstack([self.visited, np.zeros(n, dtype=bool)])

        if self.maze.start == (0, 0): #top of maze looking down
            self.direction = self.SOUTH

        else:
            self.direction = self.NORTH

        #attributes needed for later
        self.cur_pos = self.maze.start #initialize starting position
        self.optimal_path = [] #to optimal path yet, because the target is unknown

    def search(self):

        while not self.cur_pos in self.maze.target: #check to see if at target

            self.visited[self.cur_pos[0], self.cur_pos[1]] = True
            self.read_walls() #read the walls
            on_path = self.get_next_move() #move in desired direction
            if not on_path: #if node value don't reflect the current optimal path, update

                self.maze.floodfill(self.cur_pos)
                self.get_next_move()

            next_row = self.cur_pos[0] + self.direction[0]
            next_col = self.cur_pos[1] + self.direction[1] #move
            self.cur_pos = (next_row, next_col)

        self.maze.final = self.cur_pos #found the entrance to the centerblocks, save it in maze
        self.maze.floodfill(self.cur_pos) #update array with current walls
        self.calc_optimal()
        '''
        Arrived at Target. Time to go back. Reverse floodfill the values and try to get back to the starting square as fast as possible.

        '''
    def goback(self): #need to be at different square

        self.maze.floodfill(self.cur_pos, reverse=True) #set the maze start position to be the target, reverse floodfill back to target
        while self.cur_pos != self.maze.start:

            self.visited[self.cur_pos[0], self.cur_pos[1]] = True
            self.read_walls()
            on_path = self.get_next_move() #returns true if can go to lower distance square
            if not on_path: #if all squares are higher/blocked, update the nodes

                self.maze.floodfill(self.cur_pos, reverse=True)
                self.get_next_move()

            next_row = self.cur_pos[0] + self.direction[0]
            next_col = self.cur_pos[1] + self.direction[1] #move
            self.cur_pos = (next_row, next_col)

        '''
        Arrived back to start. Should be able to find the optimal path from known walls. Refloodfill the array.

        '''
        self.maze.floodfill(self.cur_pos)
        self.calc_optimal()


    def get_next_move(self): #decide which way to go

        '''Assumptions:
        1. Assumes each node is currently up to date.

        Current Algorithm:
        1. Get Surrounding Values that aren't blocked, and store them into a list.
        2. Specific Indexes in List represent a direction: 0 - North, 1 - South, 2 - West, 3 - East
        3. Get the minimum value of the list.
        4. Get the indexe(s) of the minimum values
        5. If minimum surrounding value is higher/equal than current distance value, return False so maze can update
        6. If minimum surrounding is lower, point mouse in that direction and return True

        '''

        row_coord = self.cur_pos[0]
        col_coord = self.cur_pos[1]
        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n
        nodes = self.maze.nodes
        #get surrounding squares that aren't blocked
        #set direction to the lowest one in list

        next_values = [127, 127, 127, 127]

        if (row_coord > 0) and (horiz_walls[row_coord - 1, col_coord] != 1): #check if wall above or edge

            up_square = nodes[row_coord - 1, col_coord]
            next_values[0] = up_square

        if (row_coord < (n - 1)) and (horiz_walls[row_coord, col_coord] != 1): #check if wall below or at edge

            down_square = nodes[row_coord + 1, col_coord]
            next_values[1] = down_square

        if (col_coord > 0) and (vert_walls[row_coord, col_coord - 1] != 1): #check if wall to the left or edge

            left_square = nodes[row_coord, col_coord - 1]
            next_values[2] = left_square

        if (col_coord < (n - 1)) and (vert_walls[row_coord, col_coord] != 1): #check if wall to the right or edge edge

            right_square = nodes[row_coord, col_coord + 1]
            next_values[3] = right_square

        #go to lower number
        low_square = min(next_values)
        if low_square >= nodes[row_coord, col_coord]:
            return False #there is no lower square, need to update values

        index = [x for x, y in enumerate(next_values) if y == low_square] #gets index(s)

        if len(index) > 1: #if multiple choices, favor the straight line

            if self.direction == self.NORTH and 0 in index:
                return True
            elif self.direction == self.SOUTH and 1 in index:
                return True
            elif self.direction == self.WEST and 2 in index:
                return True
            elif self.direction == self.EAST and 3 in index:
                return True

            else: #choose one at random
                del index[0]

        if index[0] == 0:
            self.direction = self.NORTH
        if index[0] == 1:
            self.direction = self.SOUTH
        if index[0] == 2:
            self.direction = self.WEST
        if index[0] == 3:
            self.direction = self.EAST

        return True #successfull direction picked

    def read_walls(self):

        direction = self.direction #current properties
        row = self.cur_pos[0]
        column = self.cur_pos[1]
        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n

        horiz_abs = self.horiz_walls_abs #the walls that need to be explored
        vert_abs = self.vert_walls_abs

        #FRONT IR
        #the three if/else blocks of code simulate reading a wall in each sensor
        if direction == self.NORTH and row > 0:
            horiz_walls[row - 1, column] = horiz_abs[row - 1, column]
        if direction == self.SOUTH and row < (n - 1):
            horiz_walls[row, column] = horiz_abs[row, column]
        if direction == self.WEST and column > 0:
            vert_walls[row - 1, column] = vert_abs[row - 1, column]
        if direction == self.EAST and column < (n - 1):
            vert_walls[row, column] = vert_abs[row, column]

            #LEFT IR
        if direction == self.NORTH and column > 0:
            vert_walls[row, column - 1] = vert_abs[row, column - 1]
        if direction == self.SOUTH and column < (n - 1):
            vert_walls[row, column] = vert_abs[row, column]
        if direction == self.WEST and row < (n - 1):
            horiz_walls[row, column] = horiz_abs[row, column]
        if direction == self.EAST and column < (n - 1):
            horiz_walls[row - 1, column] = horiz_abs[row - 1, column]

            #RIGHT IR
        if direction == self.NORTH and column < (n - 1):
            vert_walls[row, column] = vert_abs[row, column]
        if direction == self.SOUTH and column > 0:
            vert_walls[row, column - 1] = vert_abs[row, column - 1]
        if direction == self.WEST and row > 0 :
            horiz_walls[row - 1, column] = horiz_abs[row - 1, column]
        if direction == self.EAST and row < (n - 1):
            horiz_walls[row, column] = horiz_abs[row, column]

        self.maze.horiz_walls = horiz_walls #saves found walls into memory
        self.maze.vert_walls = vert_walls

    def calc_optimal(self): #this function assumes that all the (relevant) walls are found
        #this function calculates the optimal path from the values

        """Algorithm Steps:

        Assumes all walls have been found.
        Initialize list of empty coordinates.

        1. Get distance value of start position
        2. Make a list of 4 high values. Each index represents next move.
        3. Gets all adjacent distance values. Any walls should reflect in distance values.
        4. Finds the index of any square that is (current - 1).
        5. If multiple, then favor the one that doesn't switch direction.
        6. Calculate next move, and append new position to coordinate list.
        """

        #create local variables to calculate path
        n = self.maze.n
        position = self.maze.start
        horiz_walls = self.maze.horiz_walls
        vert_walls = self.maze.vert_walls
        optimal_path = [] #list of coordinates to follow, starting with the start and ending with the final
        if position == (0, 0):
            cur_direc = self.SOUTH #set starting direction to south (at top of maze)
        else:
            cur_direc = self.NORTH #set starting direction to north (at bottom of maze)

        target = self.maze.final
        nodes = self.maze.nodes

        while (position != target): #checks to see to see if at target

            optimal_path.append(position) #adds the start/new position coord to the optimal path list

            row_coord = position[0]
            col_coord = position[1]
            cur_val = nodes[row_coord, col_coord] #get current distance value of square

            next_values = [-1, -1, -1, -1] #

            if (row_coord > 0) and (horiz_walls[row_coord - 1, col_coord] != 1): #check if wall above or edge

                up_square = nodes[row_coord - 1, col_coord]
                next_values[0] = up_square

            if (row_coord < (n - 1)) and (horiz_walls[row_coord, col_coord] != 1): #check if wall below or at edge

                down_square = nodes[row_coord + 1, col_coord]
                next_values[1] = down_square

            if (col_coord > 0) and (vert_walls[row_coord, col_coord - 1] != 1): #check if wall to the left or edge

                left_square = nodes[row_coord, col_coord - 1]
                next_values[2] = left_square

            if (col_coord < (n - 1)) and (vert_walls[row_coord, col_coord] != 1): #check if wall to the right or edge edge

                right_square = nodes[row_coord, col_coord + 1]
                next_values[3] = right_square


            index = [x for x, y in enumerate(next_values) if y == (cur_val - 1)] #gets index(s)

            if len(index) > 1: #if multiple choices, favor the straight line

                if cur_direc == self.NORTH and 0 in index:
                    pass
                elif cur_direc == self.SOUTH and 1 in index:
                    pass
                elif cur_direc == self.WEST and 2 in index:
                    pass
                elif cur_direc == self.EAST and 3 in index:
                    pass

                else: #choose #whatever one. Come up with algorithm in future
                    del index[0]

            if index[0] == 0:
                cur_direc = self.NORTH
            if index[0] == 1:
                cur_direc = self.SOUTH
            if index[0] == 2:
                cur_direc = self.WEST
            if index[0] == 3:
                cur_direc = self.EAST

            #move to next square
            new_row = row_coord + cur_direc[0]
            new_col = col_coord + cur_direc[1]
            position = (new_row, new_col)

        optimal_path.append(target) #arrived at target. Append target to list to finalize
        self.optimal_path = optimal_path #save it as class variable

        return True
