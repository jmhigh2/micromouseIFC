import json, codecs #for simulated mazes
import numpy as np

class Mouse:

    def __init__(self, start_wall, maze, **kwargs): #add in another maze object that has walls

        if ('maze_file' in kwargs): #the maze needed to be solved
            try:
                maze_file = codecs.open(kwargs['maze_file'], 'r', encoding='utf-8').read()
                maze_data = json.loads(maze_file)
                horiz_walls = np.array(maze_data['horiz_walls'])
                vert_walls = np.array(maze_data['vert_walls'])
                
            except:
                pass

        self.maze = maze #expecting a maze object with walls and distances (the maze that the mouse remembers)
        if start_wall == "L":
            self.start_pos = (self.maze.n-1, 0) # bottom of maze looking up
            self.direction = "N"

        else: #top of maze looking down
            self.start_pos = (0, 0)
            self.direction = "S"

        self.cur_pos = self.start_pos #initialize starting position
        #self.optimal_path = #count from start_position to target

    def search(self):

        while not any([target == self.cur_pos for target in self.maze.target]): #check to see if at target

            self.read_walls()
            #if not move to lowest one, if there's a tie pick a random one. Create a backtrack stack as well
            #add current square to queue and floodfill

        #at target do something

        pass

    def advance(self):

        #get surrounding squares that aren't blocked
        #set direction to the lowest one in list
        #move forward

        pass

    def read_walls(self):


        direction = self.direction
        horiz_walls = self.horiz_walls
        vert_walls = self.vert_walls
        row = self.cur_pos[0]
        column = self.cur_pos[1] #read values from front left and right. Will need to write a state machine again....

        if direction == 'N':

            pass
            #read from file, or from json
            #above horizontal
            #to the left and right vertical

        if direction == 'S':
            pass
            #below horizontal
            #to the left and right vertical

        if direction == 'W':
            pass
            #above and below horizontal
            #left vertical

        if direction == 'E':
            pass
            #above and below horizontal
            #right vertical

        self.maze.horiz_walls = horiz_walls
        self.maze.vert_walls = vert_walls

        self.maze.floodfill(self.cur_pos)
        #after reading walls recommend a movement



    def sensor_read(self): #use for the actual device
        #Use 0 for unknown
        #Use 1 for wall
        #use 2 for free
        n = self.maze.n
        row = position[0]
        column = position[1]
        horiz_walls = self.horiz_walls
        vert_walls = self.vert_walls

        if FRONT_IR < SENSOR_THRESH: #thanks python for no switch statements

            #STOP THE CAR
            if direction == 'N' and row > 0: #horiz_walls
                horiz_walls[row - 1, column] = 1
            if direction == 'S' and row < (n - 1):
                horiz_walls[row, column] = 1
            if direction == 'W' and column > 0:
                vert_walls[row - 1, column] = 1
            if direction == 'E' and  column < (n - 1):
                vert_walls[row, column] = 1

        if LEFT_IR < SENSOR_THRESH:
            if direction == 'N' and column > 0:
                vert_walls[row, column - 1] = 1
            if direction == 'S' and column < (n - 1):
                vert_walls[row, column] = 1
            if direction == 'W' and row > 0:
                horiz_walls[row, column]
            if direction == 'E' and column < (n - 1):
                horiz_walls[row - 1, column]

        if RIGHT_IR < SENSOR_THRESH: #thanks python for no switch statements
            if direction == 'N' and column < (n - 1):
                vert_walls[row, column] = 1
            if direction == 'S' and column > 0:
                vert_walls[row, column - 1] = 1
            if direction == 'W' and row > 0 :
                horiz_walls[row - 1, column] = 1
            if direction == 'E' and row < (n - 1):
                horiz_walls[row, column] = 1

        self.horiz_walls = horiz_walls
        self.vert_walls = vert_walls
