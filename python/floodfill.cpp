currentPathDistance = 1; // This is how far the 'water' has flowed
while(1){
for(x = 1; x <= 5; x++) { //Creating a loop which scans the whole maze
for(y = 1; y <= 5; y++) {

if(cell[x][y] != -1) //If the cell has already been reached, then continue to the next cell
continue;

if(highestNeighbourCell(x,y) != -1) //If there is a neighbouring cell which has been
cell[x][y] = currentPathDistance; //reached, then you have reached the current cell
//so give it a value
}
}
if(cell[dist_x][dist_y] != -1) //If the destination cell has a value after a sweep, the algorithm ends
break;

currentPathDistance++; //Increment the distance because we are scanning again.
}
//The highestNeighbouringCell(x,y) function returns the value of the highest, accessible (ie there
//are no separating walls) neighbouring cell
