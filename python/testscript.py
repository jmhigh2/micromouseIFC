from maze import Maze
from mouse import Mouse

maze = Maze(4, target = (1, 2), start="left")
maze.floodfill()

mouse = Mouse("L", maze, maze_file="maze.json")
mouse.search()
#print mouse.maze.horiz_walls
#print mouse.maze.vert_walls
#print mouse.optimal_path
#print mouse.maze.nodes
