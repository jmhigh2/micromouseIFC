from mouse import Mouse
import matplotlib.pyplot as plt
import numpy as np


def graph(mouse):
    path = mouse.optimal_path
    x_coord = []
    y_coord = []
    for coord in path:
        x_coord.append(coord[0]+.5)
        y_coord.append(coord[1]+.5)

    nodes = mouse.maze.nodes #turn these into coordinates
    horiz_walls = mouse.maze.horiz_walls # will be drawing from  (x, y) to (x+1, y)
    vert_walls = mouse.maze.vert_walls #will be drawing from (x, y) to (x, Y=1)

    rotate=False

    if rotate:
        nodes = np.rot90(nodes,3)
        horiz_walls = np.rot90(horiz_walls, 3)
        vert_walls = np.rot90(vert_walls, 3)

    plt.axis([0, 16, 0, 16])

    plt.vlines(16, 0, 16)
    plt.hlines(16, 0, 16)

    plt.plot(x_coord, y_coord)

    for x in range(0, 16):
        for y in range(0, 16):
            plt.text(x+.25, y+.30, str(nodes[x][y]))

#horizontal walls will be drawn vertical
    for x in range(0, 15):
        for y in range(0, 16):
            if horiz_walls[x][y] == 1:
            #draw ()
                plt.vlines(x+1, y, y+1)

    for x in range(0, 16): #vertical walls will be drawn
        for y in range(0, 15):
            if vert_walls[x][y] == 1:
                #draw line
                plt.hlines(y+1, x, x+1)

    plt.xticks(np.arange(0, 17, 1))
    plt.yticks(np.arange(0, 17, 1))
    plt.show()


mouse = Mouse("L", maze_file="python/maze.json") #the maze file to be read
#print "Initialized Values"
#print mouse.maze.nodes
#print mouse.maze.horiz_walls
#print mouse.maze.vert_walls

mouse.search()
mouse.search()
mouse.search()
#mouse.search()
graph(mouse)
