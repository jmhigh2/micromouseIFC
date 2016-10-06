import numpy as np

class Maze:

    def __init__(self, n, target="default", start="default"): #default is four center squares for even, single center for odd

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

        nodes = np.multiply(nodes, 127) #initialize all values to be highvalues

        if target == "default": #default target is center block for odds, center four for even
            if (n % 2 == 0): #even number
                self.target = [(n/2, n/2), (n/2 - 1, n/2 -1), (n/2 - 1, n/2), (n/2, n/2 - 1)]

            else: #odd number of squares. set center block to target

                self.target = [(int(n/2), int(n/2))]

        else:  #expecting a tuple, if target isn't default i.e: (3,2)
            self.target = [(target[0], target[1])]

        if start == "default":
            self.start = (0, 0)

        else:
            self.start = (n-1, 0)

        #Maze Properties
        self.final = self.target[0] #there is only one entrance to the target, so this is that one square
        self.n = n #number of squares
        self.nodes = nodes #nodes in maze
        self.vert_walls = vert_walls #walls
        self.horiz_walls = horiz_walls

        #get maze ready to be explored
        self.floodfill(self.target[0])


    def floodfill(self, position, reverse=False):

        coordinates = []
        self.nodes.fill(127) #fill all values with really high number

        if not reverse: #going from start to target, set target(s) to zero
            targets = self.target[:]
            for target in targets:
                self.nodes[target[0], target[1]] = 0

        else: #going back to start, set start position to zero
            self.nodes[self.start[0], self.start[1]] = 0

        pathdist = 1 #set initial path value to be 1

        while True:
            for row_num in range(0, self.n): #loop over rows
                for col_num in range(0, self.n): #loop over columns
                    if self.nodes[row_num, col_num] != 127: #if cell has been updated, skip over it
                        continue

                    if self.get_lowest_square((row_num, col_num)) != 127: #if any adjacent square has already been reached, that means this square has just been reached
                        coordinates.append((row_num, col_num)) #cell needs to be updated, add to update queue

            for coordinate in coordinates:
                 self.nodes[coordinate[0], coordinate[1]] = pathdist #update all values that are next to a cell that's been reached.

            coordinates = [] #resets the update quere for next distance

            #check if any destination squares have been reached. If reached, break the loop and exit
            if not reverse:
                if (self.nodes[self.start[0], self.start[1]] != 127) and (self.nodes[position[0], position[1]] != 127):
                    break
            else:
                if (self.nodes[self.final[0], self.final[1]] != 127) and (self.nodes[position[0], position[1]] != 127):
                    break

            pathdist += 1 #increase the distance away from initial value
            #no return value needed, all class attributes should be updated

    def get_lowest_square(self, position): #expecting a (row, column) tuple

        n= self.n
        nodes = self.nodes
        horiz_walls = self.horiz_walls
        vert_walls = self.vert_walls
        row_coord = position[0]
        col_coord = position[1]

        vals = []
        if (row_coord > 0) and (horiz_walls[row_coord - 1, col_coord] != 1): #check if wall above or edge

            up_square = nodes[row_coord - 1, col_coord] #value of up square
            vals.append(up_square) #add to list

        if (row_coord < (n - 1)) and (horiz_walls[row_coord, col_coord] != 1): #check if wall below or at edge

            down_square = nodes[row_coord + 1, col_coord] #value of down square
            vals.append(down_square)

        if (col_coord > 0) and (vert_walls[row_coord, col_coord - 1] != 1): #check if wall to the left or edge

            left_square = nodes[row_coord, col_coord - 1] #value of left square
            vals.append(left_square)

        if (col_coord < (n - 1)) and (vert_walls[row_coord, col_coord] != 1): #check if wall to the right or edge edge

            right_square = nodes[row_coord, col_coord + 1] #value of right square
            vals.append(right_square)

        return min(vals) #return the minimum value around the given square
