from maze import Maze
from mouse import Mouse
import json, codecs

maze = Maze(8)
mouse = Mouse("L", maze, maze_file="maze.json")

print mouse.maze.final
