import json, codecs
import numpy as np
from maze import Maze

new_maze = Maze(16)
#store files into dictionary
maze_dict = {'n': new_maze.n, 'horiz_walls': new_maze.horiz_walls.tolist(), 'vert_walls': new_maze.vert_walls.tolist()}
json.dump(maze_dict, codecs.open("maze.json", 'w', encoding='utf-8'), separators=(',', ':'), sort_keys=True, indent=4)

print "Done Storing Files"

maze_file = codecs.open("maze.json", 'r', encoding='utf-8').read()
maze_data = json.loads(maze_file)

horiz_walls = np.array(maze_data['horiz_walls'])
vert_walls = np.array(maze_data['vert_walls'])

print horiz_walls
print vert_walls
print "Number of Squares {}".format(maze_data['n'])
