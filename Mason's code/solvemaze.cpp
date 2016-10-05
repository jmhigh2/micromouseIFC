//Mason Carnino Maze Conversion
//Copyright (c) Mason A. Carnino 10/1/16
//


#include <iostream>
#include <fstream>
#include "location.h"

using namespace std;

void intmaze(struct cell maze[16][16]) {
	int current = 0;
	int n = 0;
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			maze[i][j].currentpathdistance = current;
			maze[i][j].col = j;
			maze[i][j].row = i;
			maze[i][j].N_wall = 0;
			maze[i][j].E_wall = 0;
			maze[i][j].S_wall = 0;
			maze[i][j].W_wall = 0;
			current++;
			cout << maze[i][j].currentpathdistance;
		}
		current = 1 + n;
		n++;
		cout << endl;
	}
}

int main()
{
	struct cell maze[16][16];
	intmaze(maze);
	cout << "\n\n" << maze[7][7].currentpathdistance;
	cout << endl << endl;
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			cout << maze[i][j].N_wall;
		}
		cout << endl;
	}
}