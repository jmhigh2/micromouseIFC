import numpy as np
import time

class Maze:

    def __init__(self, n, target="default"): #default is four center squares for even, single center for odd

        DTYPE = np.int
        horiz_walls = np.zeros(n, dtype=DTYPE)
        vert_walls = np.zeros(n-1, dtype=DTYPE)
        nodes = np.ones(n, dtype=DTYPE)

        for a in range(1, n):
            nodes = np.vstack([nodes, np.ones(n, dtype=DTYPE)]) #create nxn matrix that represents the distance values
        for a in range(1, n):
            vert_walls = np.vstack([vert_walls, np.zeros(n-1, dtype=DTYPE)]) #create matrix that represents the vertical walls
        for a in range(1, n-1):
            horiz_walls = np.vstack([horiz_walls, np.zeros(n, dtype=DTYPE)]) #create matrix that repressent the horizontal walls

        #set center blocks to 0 (targets)
        if target == "default":
            if (n % 2 == 0): #even number
                nodes[int(n/2), int(n/2)] = 0
                nodes[int(n/2), int(n/2) - 1] = 0
                nodes[int(n/2) - 1, int(n/2)] = 0
                nodes[int(n/2) - 1, int(n/2) - 1] = 0

            else: #odd number of squares. set center block to target
                nodes[int(n/2), int(n/2)] = 0

        else:  #expecting a tuple, if target isn't default i.e: (3,2)
            tarx = target[0]
            tary = target[1]
            nodes[tarx, tary] = 0

        nodes = nodes*n*n #initialize distance to really high values for initial fill

        x_target, y_target = np.where(nodes == 0) #target was set earlier

        #Maze Properties
        self.target = [(x,y) for x, y in zip(x_target, y_target)]
        self.final = () #there is only one entrance to the target, so this is that one square
        self.n = n #number of squares
        self.nodes = nodes #nodes in maze
        self.vert_walls = vert_walls #walls
        self.horiz_walls = horiz_walls

        initial = self.target[:] #floodfill will use up list, so need to copy it to new one
        self.initialize(initial) #initialize values

    def initialize(self, queue): #modified algorthim to get the proper starting values

        n = self.n
        nodes = self.nodes

        while queue:

            coord = queue.pop()
            vals = [] # [up, down, left, right]
            row_coord = coord[0]
            col_coord = coord[1] #current distance of
            up = False
            down = False
            left = False
            right = False

            if row_coord > 0: #check if at top
                up_square = nodes[row_coord - 1, col_coord]
                vals.append(up_square)
                up = True

            if row_coord < (n - 1): #check if bottom
                down_square = nodes[row_coord + 1, col_coord]
                vals.append(down_square)
                down = True

            if col_coord > 0: #check if boundary to the left

                left_square = nodes[row_coord, col_coord - 1]
                vals.append(left_square)
                left = True

            if col_coord < (n - 1): #check if boundary to the right

                right_square = nodes[row_coord, col_coord + 1]
                vals.append(right_square)
                right = True

            if nodes[row_coord, col_coord] > 0:
                nodes[row_coord, col_coord] = min(vals) + 1
            val = nodes[row_coord, col_coord]

            if up and (up_square > 0) and (up_square > val):
                queue.append((row_coord - 1, col_coord))

            if down and (down_square > 0) and (down_square > val):
                queue.append((row_coord + 1, col_coord))

            if right and  (right_square > 0) and (right_square > val):
                queue.append((row_coord, col_coord + 1))

            if left and (left_square > 0) and (left_square > val):
                queue.append((row_coord, col_coord - 1))

        self.nodes = nodes

    def floodfill(self, queue):

        n = self.n
        nodes = self.nodes
        horiz_walls = self.horiz_walls
        vert_walls = self.vert_walls

        while queue:

            coord = queue.pop()
            vals = [] # [up, down, left, right]
            row_coord = coord[0]
            col_coord = coord[1] #current distance of
            val = nodes[row_coord, col_coord]

            if (row_coord > 0) and (horiz_walls[row_coord - 1, col_coord] != 1): #check if wall above or edge

                up_square = nodes[row_coord - 1, col_coord]
                vals.append(up_square)
                if (up_square > 0) and (up_square > val):
                    queue.append((row_coord - 1, col_coord))

            if (row_coord < (n - 1)) and (horiz_walls[row_coord, col_coord] != 1): #check if wall below or at edge

                down_square = nodes[row_coord + 1, col_coord]
                vals.append(down_square)
                if (down_square > 0) and (down_square > val):
                    queue.append((row_coord + 1, col_coord))

            if (col_coord > 0) and (vert_walls[row_coord, col_coord - 1] != 1): #check if wall to the left or edge

                left_square = nodes[row_coord, col_coord - 1]
                vals.append(left_square)
                if (left_square > 0) and (left_square > val):
                    queue.append((row_coord, col_coord - 1))


            if (col_coord < (n - 1)) and (vert_walls[row_coord, col_coord] != 1): #check if wall to the right or edge edge

                right_square = nodes[row_coord, col_coord + 1]
                vals.append(right_square)
                if (right_square > 0) and (right_square > val):
                    queue.append((row_coord, col_coord + 1))


            if nodes[row_coord, col_coord] > 0:
                nodes[row_coord, col_coord] = min(vals) + 1

        self.nodes = nodes #if done, save all updated values in class attributes
