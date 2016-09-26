import json, codecs #for simulated mazes
import numpy as np
from maze import Maze
import time #for debugging
from random import random

class Mouse:

    #directions of travel in the matrix, a.k.a constants
    NORTH = (-1, 0)
    SOUTH = (1, 0)
    WEST = (0, -1)
    EAST = (0, 1)

    def __init__(self, start_wall, maze, **kwargs): #expecting a start wall (for start position), a maze object (to store the memorized maze)
            #cont'd - and a maze file in kwargs, that stores the maze to be analyzed
        if ('maze_file' in kwargs): #the maze needed to be solved
            maze_file = codecs.open(kwargs['maze_file'], 'r', encoding='utf-8').read()
            maze_data = json.loads(maze_file)
            horiz_walls = np.array(maze_data['horiz_walls'])
            vert_walls = np.array(maze_data['vert_walls'])

            self.horiz_walls_sol = horiz_walls #loading the walls to be analyzed
            self.vert_walls_sol = vert_walls

        self.maze = maze #expecting a maze object with walls and distances (the maze that the mouse remembers)

        if start_wall == "L":
            self.start_pos = (self.maze.n-1, 0) # bottom of maze looking up
            self.direction = self.NORTH

        else: #top of maze looking down
            self.start_pos = (0, 0)
            self.direction = self.SOUTH

        self.cur_pos = self.start_pos #initialize starting position
        self.optimal_path = [] #to optimal path yet, because the target is unknown

    def search(self):

        while not self.cur_pos in self.maze.target: #check to see if at target

            self.read_walls() #read the walls and floodfill the distance values

            print "Before: {}, Direction: {}".format(self.cur_pos, self.direction)
            print self.maze.nodes
            self.advance() #move in desired direction
            print "After: {}, Direction: {}".format(self.cur_pos, self.direction)
            print self.maze.nodes

        #print "Done Solving"
        #print "Calculating Optimal"
        self.maze.final = self.cur_pos #found the entrance to the centerblocks
        #self.calc_optimal() #calculate optimal path. Will do this while going to back to start

        #at target do something


    def advance(self):

        row_coord = self.cur_pos[0]
        col_coord = self.cur_pos[1]
        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n
        nodes = self.maze.nodes
        cur_direc = self.direction
        #get surrounding squares that aren't blocked
        #set direction to the lowest one in list

        next_values = [100, 100, 100, 100]

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
        index = [x for x, y in enumerate(next_values) if y == low_square] #gets index

        if len(index) > 1: #if multiple choices, favor the straight line

            if cur_direc == self.NORTH and 0 in index:
                pass
            elif cur_direc == self.NORTH and 1 in index:
                pass
            elif cur_direc == self.NORTH and 2 in index:
                pass
            elif cur_direc == self.NORTH and 3 in index:
                pass

            else: #choose one at random
                del index[0]
                if index[0] == 0:
                    cur_direc = self.NORTH
                if index[0] == 1:
                    cur_direc = self.SOUTH
                if index[0] == 2:
                    cur_direc = self.WEST
                if index[0] == 3:
                    cur_direc = self.EAST
        else:
            if index[0] == 0:
                cur_direc = self.NORTH
            if index[0] == 1:
                cur_direc = self.SOUTH
            if index[0] == 2:
                cur_direc = self.WEST
            if index[0] == 3:
                cur_direc = self.EAST

        next_row = row_coord + cur_direc[0]
        next_col = col_coord + cur_direc[1] #move

        self.direction = cur_direc
        self.cur_pos = (next_row, next_col)

    def read_walls(self):

        direction = self.direction #current properties
        row = self.cur_pos[0]
        column = self.cur_pos[1]

        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n

        horiz_sol = self.horiz_walls_sol #the walls that need to be explored
        vert_sol = self.vert_walls_sol

        try:
        #FRONT IR
        #the three if/else blocks of code simulate reading a wall in each sensor
            if direction == self.NORTH and row > 0:
                horiz_walls[row - 1, column] = horiz_sol[row - 1, column]
            if direction == self.SOUTH and row < (n - 1):
                horiz_walls[row, column] = horiz_sol[row, column]
            if direction == self.WEST and column > 0:
                vert_walls[row - 1, column] = vert_sol[row - 1, column]
            if direction == self.EAST and column < (n - 1):
                vert_walls[row, column] = vert_sol[row, column]

            #LEFT IR
            if direction == self.NORTH and column > 0:
                vert_walls[row, column - 1] = vert_sol[row, column - 1]
            if direction == self.SOUTH and column < (n - 1):
                vert_walls[row, column] = vert_sol[row, column]
            if direction == self.WEST and row < (n - 1):
                horiz_walls[row, column] = horiz_sol[row, column]
            if direction == self.EAST and column < (n - 1):
                horiz_walls[row - 1, column] = horiz_sol[row - 1, column]

            #RIGHT IR
            if direction == self.NORTH and column < (n - 1):
                vert_walls[row, column] = vert_sol[row, column]
            if direction == self.SOUTH and column > 0:
                vert_walls[row, column - 1] = vert_sol[row, column - 1]
            if direction == self.WEST and row > 0 :
                horiz_walls[row - 1, column] = horiz_sol[row - 1, column]
            if direction == self.EAST and row < (n - 1):
                horiz_walls[row, column] = horiz_sol[row, column]

        except IndexError as e:
            print "Values at time of exception: "
            print "Position: {}".format(self.cur_pos)
            print "Direction {}".format(self.direction)
            print "Nodes: {}".format(self.maze.nodes)
            print self.maze.vert_walls
            print self.maze.horiz_walls
            raise e

        self.maze.horiz_walls = horiz_walls #saves found walls into memory
        self.maze.vert_walls = vert_walls
        position = self.cur_pos[:]
        self.maze.floodfill([position])
        #after reading walls recommend a movement

    def sensor_read(self): #use for the actual device

        n = self.maze.n #local variables are faster than class attributes
        row = position[0]
        column = position[1]
        horiz_walls = self.horiz_walls
        vert_walls = self.vert_walls

        if FRONT_IR < SENSOR_THRESH: #thanks python for no switch statements

            #STOP THE CAR
            if direction == self.NORTH and row > 0: #north
                horiz_walls[row - 1, column] = 1
            if direction == self.SOUTH and row < (n - 1):
                horiz_walls[row, column] = 1
            if direction == self.WEST and column > 0:
                vert_walls[row - 1, column] = 1
            if direction == self.EAST and  column < (n - 1):
                vert_walls[row, column] = 1

        if LEFT_IR < SENSOR_THRESH:
            if direction == self.NORTH and column > 0:
                vert_walls[row, column - 1] = 1
            if direction == self.SOUTH and column < (n - 1):
                vert_walls[row, column] = 1
            if direction == self.WEST and row > 0:
                horiz_walls[row, column]
            if direction == self.EAST and column < (n - 1):
                horiz_walls[row - 1, column]

        if RIGHT_IR < SENSOR_THRESH: #thanks python for no switch statements
            if direction == self.NORTH and column < (n - 1):
                vert_walls[row, column] = 1
            if direction == self.SOUTH and column > 0:
                vert_walls[row, column - 1] = 1
            if direction == self.WEST and row > 0 :
                horiz_walls[row - 1, column] = 1
            if direction == self.EAST and row < (n - 1):
                horiz_walls[row, column] = 1

        self.horiz_walls = horiz_walls
        self.vert_walls = vert_walls

    def calc_optimal(self): #this function assumes that all the (relevant) walls are found.

        n = self.maze.n
        position = self.start_pos
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

            next_values = [x*random() for x in range(1,5)] #sets the next values to random. Needed

            #find square that is one less than current
            if row_coord > 0: #check if wall above or edge

                up_square = nodes[row_coord - 1, col_coord]
                next_values[0] = up_square

            if row_coord < (n - 1): #check if wall below or at edge

                down_square = nodes[row_coord + 1, col_coord]
                next_values[1] = down_square

            if col_coord > 0: #check if wall to the left or edge

                left_square = nodes[row_coord, col_coord - 1]
                next_values[2] = left_square

            if col_coord < (n - 1): #check if wall to the right or edge edge

                right_square = nodes[row_coord, col_coord + 1]
                next_values[3] = right_square

            try:
            #go to lower number
                index = [x for x, y in enumerate(next_values) if y == (cur_val - 1)] #gets index

                if len(index) > 1: #if multiple choices, favor the straight line
                    #get current direction,
                    if cur_direc == self.NORTH and 0 in index:
                        pass
                    elif cur_direc == self.NORTH and 1 in index:
                        pass
                    elif cur_direc == self.NORTH and 2 in index:
                        pass
                    elif cur_direc == self.NORTH and 3 in index:
                        pass
                    else:
                        del index[0] #if you can't just go straight then delete whatever, come back to this
                        if index[0] == 0:
                            cur_direc = self.NORTH
                        if index[0] == 1:
                            cur_direc = self.SOUTH
                        if index[0] == 2:
                            cur_direc = self.WEST
                        if index[0] == 3:
                            cur_direc = self.EAST

                else: #if only one choice to go, the go to that direction
                    if index[0] == 0:
                        cur_direc = self.NORTH
                    if index[0] == 1:
                        cur_direc = self.SOUTH
                    if index[0] == 2:
                        cur_direc = self.WEST
                    if index[0] == 3:
                        cur_direc = self.EAST
            except IndexError as e:
                print index
                print next_values
                raise e

            #move to next square
            new_row = row_coord + cur_direc[0]
            new_col = col_coord + cur_direc[1]
            position = (new_row, new_col)

        optimal_path.append(target) #arrived at target. Append target to list to finalize
        self.optimal_path = optimal_path #save it as class variable
