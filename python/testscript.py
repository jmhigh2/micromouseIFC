from maze import Maze
from mouse import Mouse
import json, codecs
import time

maze = Maze(16)
print maze.nodes
print maze.vert_walls
print maze.horiz_walls

#mouse = Mouse("L", maze, maze_file="python/maze.json")
#mouse.search()
#print mouse.optimal_path
#print mouse.maze.nodes
