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

    nodes = mouse.maze.nodes
    horiz_walls = mouse.maze.horiz_walls
    vert_walls = mouse.maze.vert_walls

    abs_vertwalls = mouse.vert_walls_abs
    abs_horizwalls = mouse.horiz_walls_abs

    rotate=False

    #if rotate: #Eventuallly rotate values
    #    nodes = np.rot90(nodes,3)
    #    horiz_walls = np.rot90(horiz_walls, 3)
    #    vert_walls = np.rot90(vert_walls, 3)
    fig = plt.figure()
    fig.add_subplot(121)
    plot1 = plt.gca()
    plot1.set_aspect('equal', adjustable='box')
    plot1.axes.get_xaxis().set_visible(False)
    plot1.axes.get_yaxis().set_visible(False)
    plt.axis([0, 16, 0, 16])

    plt.vlines(16, 0, 16) #borders of maze
    plt.hlines(16, 0, 16)

    plt.plot(x_coord, y_coord) #plot optimal path
    plt.title("Memorized Maze")

    for x in range(0, 16): #plot floodfill numbers
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

    #subplot of the absolute mazeB
    fig.add_subplot(122)
    plot2 = plt.gca()
    plot2.set_aspect('equal', adjustable='box')
    plot2.axes.get_xaxis().set_visible(False)
    plot2.axes.get_yaxis().set_visible(False)
    plt.axis([0, 16, 0, 16])

    plt.vlines(16, 0, 16) #borders of maze
    plt.hlines(16, 0, 16)
    plt.plot(4, 4)
    plt.title("Absolute Maze")

    plt.plot(x_coord, y_coord) #plot optimal path
            #horizontal walls will be drawn vertical
    for x in range(0, 15):
        for y in range(0, 16):
            if abs_horizwalls[x][y] == 1:
            #draw ()
                plt.vlines(x+1, y, y+1)

    for x in range(0, 16): #vertical walls will be drawn
        for y in range(0, 15):
            if abs_vertwalls[x][y] == 1:
                #draw line
                plt.hlines(y+1, x, x+1)

    mng = plt.get_current_fig_manager()
    #mng.full_screen_toggle()
    plt.show()

#def animate(i): #need to pass interval


if __name__ == '__main__':

    mouse = Mouse("L", maze_file="maze.json") #the maze file to be read
    graph(mouse)

    mouse.search()
    graph(mouse)

    mouse.search()
    graph(mouse)

    mouse.search()
    graph(mouse)
