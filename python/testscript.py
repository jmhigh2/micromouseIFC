from maze import Maze
from mouse import Mouse

maze = Maze(16, start="left")
maze.floodfill((7, 7))

mouse = Mouse("L", maze, maze_file="maze.json")
mouse.search()
print mouse.path
