import json, codecs #for simulated mazes
import numpy as np
from maze import Maze
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

        if self.maze.start == (0, 0): #top of maze looking down
            self.direction = self.SOUTH

        else:
            self.direction = self.NORTH

        self.cur_pos = self.maze.start #initialize starting position
        self.optimal_path = [] #to optimal path yet, because the target is unknown

    def search(self):

        print "Target: {}".format(self.maze.target)
        while not self.cur_pos in self.maze.target: #check to see if at target

            print "Current Position: {}".format(self.cur_pos)
            print "FloodFill Values:"
            print self.maze.nodes
            self.read_walls() #read the walls and floodfill the distance values
            on_path = self.get_next_move() #move in desired direction
            if not on_path:

                self.maze.floodfill(self.cur_pos)
                self.get_next_move()

            next_row = self.cur_pos[0] + self.direction[0]
            next_col = self.cur_pos[1] + self.direction[1] #move
            self.cur_pos = (next_row, next_col)

        print "Current Position: {}".format(self.cur_pos)
        print "Done Solving"
        print self.maze.horiz_walls
        print self.maze.vert_walls
        print self.maze.nodes
        self.maze.final = self.cur_pos #found the entrance to the centerblocks

        #self.calc_optimal() #calculate optimal path. Will do this while going to back to start

        #Current position should still be at target
        #try to take optimal path and read walls while going
        #while self.cur_pos!= self.start_pos:


    def get_next_move(self): #decide which way to go

        '''Assumptions:
        1. Assumes each node is currently up to date.


        '''

        '''Current Algorithm:
        1. Get Surrounding Values that aren't blocked, and store them into a list.
        2. Specific Indexes in List represent a direction: 0 - North, 1 - South, 2 - West, 3 - East
        3. Get the minimum value of the list.
        4. Get the indexe(s) of the minimum values

        '''

        row_coord = self.cur_pos[0]
        col_coord = self.cur_pos[1]
        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n
        nodes = self.maze.nodes
        #get surrounding squares that aren't blocked
        #set direction to the lowest one in list

        next_values = [63,63,63,63]

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
        if low_square > nodes[row_coord, col_coord]:
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

        horiz_sol = self.horiz_walls_sol #the walls that need to be explored
        vert_sol = self.vert_walls_sol

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

        self.maze.horiz_walls = horiz_walls #saves found walls into memory
        self.maze.vert_walls = vert_walls

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

    def calc_optimal(self): #this function assumes that all the (relevant) walls are found
        #this function calculates the optimal path from the values

        """Algorithm Steps:

        Assumes all walls have been found.
        Initialize list of empty coordinates.

        1. Get distance value of current position
        2. Make a list of 4 random values. Each index represents next move.
        3. Gets all adjacent distance values. Any walls should reflect in distance values.
        4. Finds the index of any square that is current - 1.
        5. If multiple, then favor the one that doesn't switch direction.
        6. Calculate next move, and append new position to coordinate list.
        """

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
