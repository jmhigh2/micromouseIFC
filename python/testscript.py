from mouse import Mouse

mouse = Mouse("L", maze_file="python/maze.json") #the maze file to be read
print "Initialized Values"
print mouse.maze.nodes
print mouse.maze.horiz_walls
print mouse.maze.vert_walls

mouse.search()

#Results
print "Results"
print mouse.maze.nodes
print mouse.maze.horiz_walls
print mouse.maze.vert_wallsf
print "Optimal Path of Length {}".format(len(mouse.optimal_path))
print mouse.optimal_path
