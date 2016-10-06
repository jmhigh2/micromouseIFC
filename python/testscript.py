from mouse import Mouse
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.font_manager import FontProperties


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
    position = mouse.cur_pos

    abs_vertwalls = mouse.vert_walls_abs
    abs_horizwalls = mouse.horiz_walls_abs

    #rotate=False

    #if rotate: #Eventuallly rotate values
    #    nodes = np.rot90(nodes,3)
    #    horiz_walls = np.rot90(horiz_walls, 3)
    #    vert_walls = np.rot90(vert_walls, 3)
    fig = plt.figure()
    fig.add_subplot(121)
    plot1 = plt.gca()
    plot1.set_aspect('equal', adjustable='box')
    #plot1.axes.get_xaxis().set_visible(False)
    plot1.axes.get_yaxis().set_visible(False)
    plt.axis([0, 16, 0, 16])

    plt.vlines(16, 0, 16) #borders of maze
    plt.hlines(16, 0, 16)

    plot1.scatter(position[0]+.5, position[1]+.5, s=100, color='g')

    #a = patches.Rectangle(color='k')
    #b = patches.Rectangle(color='m')
    #c = patches.Rectangle(color='b')

    #plot1.legend([a, b, c] ,['Floodfill Values', 'Horizontal Walls', 'Vertical Walls' ], handler_map={a: Handler(), b: ()Handler, c: Handler()}
    #bbox_to_anchor=(-0.025,1), fontsize=10)
    plt.title("Memorized Maze/FloodFill Values")
    plot1.set_xlabel('This is the maze that the mouse has memorized. \n It always goes to the lower number square adjacent to current position. \n If it cant move to a lower square, then it knows the values are outdated, and floodfills with the new walls.')

    font = FontProperties()
    font.set_weight('bold')

    plt.plot(x_coord, y_coord, linewidth=2.0, color='g') #plot optimal path
    for x in range(0, 16): #plot floodfill numbers
        for y in range(0, 16):
            plt.text(x+.25, y+.30, str(nodes[x][y]), fontproperties=font)
            if mouse.visited[x][y]:
                plot1.add_patch(patches.Rectangle((x,y),1, 1, color='y'))

            #horizontal walls will be drawn vertical
    for x in range(0, 15):
        for y in range(0, 16):
            plt.text(x+.85, y+.35, str(horiz_walls[x][y]), color='m', fontsize=7)
            if horiz_walls[x][y] == 1:
            #draw ()
                plt.vlines(x+1, y, y+1)

    for x in range(0, 16): #vertical walls will be drawn
        for y in range(0, 15):
            plt.text(x+.35, y+.75, str(vert_walls[x][y]), color='b', fontsize=7)
            if vert_walls[x][y] == 1:
                #draw line
                plt.hlines(y+1, x, x+1)

    #subplot of the absolute maze
    fig.add_subplot(122)
    plot2 = plt.gca()
    plot2.set_aspect('equal', adjustable='box')
    plot2.scatter(position[0]+.5, position[1]+.5, s=100, color='g') #position
    #plot2.axes.get_xaxis().set_visible(False)
    plot2.axes.get_yaxis().set_visible(False)
    plt.axis([0, 16, 0, 16])

    plt.vlines(16, 0, 16) #borders of maze
    plt.hlines(16, 0, 16)
    plt.title("Absolute Maze/Vertical/Horizontal Wall Arrays")
    #plot2.set_xlabel('This is the maze that the mouse searches one square at a time. Optimal Path is shown with colored line.')

    plt.plot(x_coord, y_coord, linewidth=2.0, color='g') #plot optimal path
            #horizontal walls will be drawn vertical
    for x in range(0, 15):
        for y in range(0, 16):
            plt.text(x+.75, y+.35, str(abs_horizwalls[x][y]), color='m')
            if abs_horizwalls[x][y] == 1:
            #draw ()
                plt.vlines(x+1, y, y+1)


    for x in range(0, 16): #vertical walls will be drawn
        for y in range(0, 15):
            plt.text(x+.35, y+.75, str(abs_vertwalls[x][y]), color='b')
            if abs_vertwalls[x][y] == 1:
                #draw line
                plt.hlines(y+1, x, x+1)


    #mng = plt.get_current_fig_manager()
    #mng.full_screen_toggle()
    plt.show()

#def animate(i): #need to pass interval

if __name__ == '__main__':

    mouse = Mouse("L", maze_file="maze.json") #the maze file to be read
    for a in range(1,3):
        graph(mouse) #initialized values
        mouse.search() #first search, find target
        graph(mouse)
        mouse.goback() #from target, go back to start
