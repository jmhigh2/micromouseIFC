from maze import Maze
from mouse import Mouse
import json, codecs
import time

maze = Maze(4, target=(1,2))
mouse = Mouse("L", maze, maze_file="python/maze.json")
mouse.search()
print mouse.optimal_path
print mouse.maze.nodes
