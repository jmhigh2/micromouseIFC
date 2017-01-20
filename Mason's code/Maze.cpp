e//Mason Carnino Maze Conversion
//Copyright (c) Mason A. Carnino 10/1/16
//


#include <iostream>
#include <fstream>
#include "location.h"

using namespace std;

bool neighbors(struct cell maze[16][16], int x, int y) {
	if (x == 0) {
		if (y == 0) {
			if ((maze[x + 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1))
				return true;
		}
		else if (y == 15) {
			if ((maze[x + 1][y].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1))
				return true;
		}
		else {
			if ((maze[x + 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1))
				return true;
		}
	}
	else if (x == 15) {
		if (y == 0) {
			if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1))
				return true;
		}
		else if (y == 15) {
			if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1))
				return true;
		}
		else {
			if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1))
				return true;
		}
	}
	else if (y == 0 && (x != 0 || x != 15)) {
		if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1) || (maze[x + 1][y].currentpathdistance != -1))
			return true;
	}
	else if (y == 15 && (x != 0 || x != 15)) {
		if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1) || (maze[x + 1][y].currentpathdistance != -1))
			return true;
	}
	else if (x > 0 && x < 15 && y > 0 && y < 15) {
		if ((maze[x - 1][y].currentpathdistance != -1) || (maze[x][y + 1].currentpathdistance != -1) || (maze[x + 1][y].currentpathdistance != -1) || (maze[x][y - 1].currentpathdistance != -1))
			return true;
	}
	else
		return 0;
}

void flood(struct cell maze[16][16]) {
	for (int current = 1; current < 15; current++) {
		for (int i = 0; i <= 15; i++) {
			for (int j = 0; j <= 15; j++) {
				if (neighbors(maze, i, j) == true && maze[i][j].currentpathdistance != 0 && maze[i][j].previous == false) {
						maze[i][j].currentpathdistance = current;
						maze[i][j].previous = true;
					}

				}
			}

	}
}

void intmaze(struct cell maze[16][16]) {
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			maze[i][j].currentpathdistance = -1;
			maze[i][j].previous = false;
			maze[i][j].N_wall = 0;
			maze[i][j].E_wall = 0;
			maze[i][j].S_wall = 0;
			maze[i][j].W_wall = 0;
		}
		cout << endl;
	}
	maze[7][7].currentpathdistance = 0;
	maze[7][8].currentpathdistance = 0;
	maze[8][7].currentpathdistance = 0;
	maze[8][8].currentpathdistance = 0;
}


int main()
{
	struct cell maze[16][16];
	intmaze(maze);
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			cout << maze[i][j].currentpathdistance << "  ";
		}
		cout << endl;
	}
	cout << "\n\n";
	flood(maze);
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			cout << maze[i][j].currentpathdistance << "  ";
		}
		cout << endl;
	}
}
