import json, codecs #for simulated mazes
import numpy as np
from maze import Maze

class Mouse:

    NORTH = (-1, 0)
    SOUTH = (1, 0)
    WEST = (0, -1)
    EAST = (0, 1)

    def __init__(self, start_wall, maze, **kwargs): #add in another maze object that has walls

        if ('maze_file' in kwargs): #the maze needed to be solved
            maze_file = codecs.open(kwargs['maze_file'], 'r', encoding='utf-8').read()
            maze_data = json.loads(maze_file)
            horiz_walls = np.array(maze_data['horiz_walls'])
            vert_walls = np.array(maze_data['vert_walls'])

            self.maze_solve = Maze(maze_data['n'])
            self.maze_solve.horiz_walls = horiz_walls
            self.maze_solve.vert_walls = vert_walls

        self.maze = maze #expecting a maze object with walls and distances (the maze that the mouse remembers)
        if start_wall == "L":
            self.start_pos = (self.maze.n-1, 0) # bottom of maze looking up
            self.direction = self.NORTH

        else: #top of maze looking down
            self.start_pos = (0, 0)
            self.direction = self.SOUTH

        self.cur_pos = self.start_pos #initialize starting position
        #self.optimal_path = #count from start_position to target. set start to beginning square node and end to zero. optimal path is the path with the least squares

    def search(self):

        while not any([target == self.cur_pos for target in self.maze.target]): #check to see if at target

            self.read_walls()
            self.advance()
            print self.cur_pos
            #self.calc_optimal()
            #if not move to lowest one, if there's a tie pick a random one. Create a backtrack stack as well
            #add current square to queue and floodfill

        self.maze.final = self.cur_pos #found the entrance to the centerblocks
        self.calc_optimal()
        #at target do something
        print "Done Solving"

        pass

    def advance(self): #need to create stack that saves the last squares if you reach a dead end

        cur_pos = self.cur_pos
        row_coord = cur_pos[0]
        col_coord = cur_pos[1]
        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n
        nodes = self.maze.nodes
        #get surrounding squares that aren't blocked
        #set direction to the lowest one in list

        next_values = [255, 255, 255, 255]

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
            del index[0]

        if index[0] == 0:
            cur_direc = self.NORTH
        if index[0] == 1:
            cur_direc = self.SOUTH
        if index[0] == 2:
            cur_direc = self.WEST
        if index[0] == 3:
            cur_direc = self.EAST

        next_row = cur_pos[0] + cur_direc[0]
        next_col = cur_pos[1] + cur_direc[1] #move

        self.direction = cur_direc
        self.cur_pos = (next_row, next_col)

    def read_walls(self):

        #direction values
        #NORTH: (-1, 0)
        #SOUTH: (1, 0)
        #WEST: (0, -1)
        #EAST: (0, 1)
        direction = self.direction #current properties
        row = self.cur_pos[0]
        column = self.cur_pos[1]

        horiz_walls = self.maze.horiz_walls #the walls that the mouse knows
        vert_walls = self.maze.vert_walls
        n = self.maze.n

        horiz_sol = self.maze_solve.horiz_walls #the walls that need to be explored
        vert_sol = self.maze_solve.vert_walls

        #FRONT IR
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
        if direction == self.WEST and row > 0:
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

        self.maze.floodfill([self.cur_pos])
        #after reading walls recommend a movement

    def sensor_read(self): #use for the actual device
        #Assume there are no walls
        #Use 1 for wall
        #use 0 for free
        n = self.maze.n
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

    def calc_optimal(self):

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

        while (position != target):
            print "Postion: {}".format(position)
            cur_val = nodes[position[0], position[1]] #get current distance value of square
            row_coord = position[0]
            col_coord = position[1]
            optimal_path.append(position)

            next_values = [0, 0, 0, 0]

            #find square that is one less than current
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
            index = [x for x, y in enumerate(next_values) if y == (cur_val - 1)] #gets index

            if len(index) > 1: #if multiple choices, favor the straight line
                del index[0]

            if index[0] == 0:
                cur_direc = self.NORTH
            if index[0] == 1:
                cur_direc = self.SOUTH
            if index[0] == 2:
                cur_direc = self.WEST
            if index[0] == 3:
                cur_direc = self.EAST

            #biases for straight paths if there's a tie

            #move to next square
            new_row = row_coord + cur_direc[0]
            new_col = col_coord + cur_direc[1]
            position = (new_row, new_col)

        optimal_path.append(target)
        print optimal_path
