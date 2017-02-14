//Mason Carnino Maze Conversion
//Copyright (c) Mason A. Carnino 10/1/16



#include <iostream>
#include <fstream>
#include "location.h"
#include <array>
#include <stdlib.h>
#include <time.h> 
#include <chrono>

using namespace std;


bool neighbors(struct cell maze[16][16], int x, int y) {
	if (x == 0) {
		if (y == 0) {
			if ((maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false))
				return true;
		}
		else if (y == 15) {
			if ((maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false))
				return true;
		}
		else {
			if ((maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false))
				return true;
		}
	}
	else if (x == 15) {
		if (y == 0) {
			if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false))
				return true;
		}
		else if (y == 15) {
			if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false))
				return true;
		}
		else {
			if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false))
				return true;
		}
	}
	else if (y == 0 && (x != 0 || x != 15)) {
		if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false) || (maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false))
			return true;
	}
	else if (y == 15 && (x != 0 || x != 15)) {
		if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false) || (maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false))
			return true;
	}
	else if (x > 0 && x < 15 && y > 0 && y < 15) {
		if ((maze[x - 1][y].currentpathdistance != -1 && maze[x][y].N_wall == false) || (maze[x][y + 1].currentpathdistance != -1 && maze[x][y].E_wall == false) || (maze[x + 1][y].currentpathdistance != -1 && maze[x][y].S_wall == false) || (maze[x][y - 1].currentpathdistance != -1 && maze[x][y].W_wall == false))
			return true;
	}
	else
		return 0;
}

void flood(struct cell maze[16][16]) {
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			maze[i][j].currentpathdistance = -1;
			}
		}
	maze[7][7].currentpathdistance = 0;
	maze[7][8].currentpathdistance = 0;
	maze[8][7].currentpathdistance = 0;
	maze[8][8].currentpathdistance = 0;
	for (int current = 1; current < 120; current++) {
		for (int i = 0; i <= 15; i++) {
			for (int j = 0; j <= 15; j++) {
				if (neighbors(maze, i, j) == true && maze[i][j].currentpathdistance != 0 && maze[i][j].previous == false) {
					maze[i][j].previous = true;
					maze[i][j].update = true;
				}

			}
		}
		for (int i = 0; i <= 15; i++) {
			for (int j = 0; j <= 15; j++) {
				if (maze[i][j].update == true) {
					maze[i][j].currentpathdistance = current;
					maze[i][j].update = false;
				}
			}
		}
	}
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			maze[i][j].previous = false;
		}
	}
}

void updwalls(struct mouse direction, struct cell maze[16][16], struct cell exmaze[16][16]) {
	if (direction.navi == 1) {
		maze[direction.row][direction.col].E_wall = exmaze[direction.row][direction.col].E_wall;
		maze[direction.row][direction.col].N_wall = exmaze[direction.row][direction.col].N_wall;
		maze[direction.row][direction.col].W_wall = exmaze[direction.row][direction.col].W_wall;
	}
	else if (direction.navi == 2) {
		maze[direction.row][direction.col].E_wall = exmaze[direction.row][direction.col].E_wall;
		maze[direction.row][direction.col].N_wall = exmaze[direction.row][direction.col].N_wall;
		maze[direction.row][direction.col].S_wall = exmaze[direction.row][direction.col].S_wall;
	}
	else if (direction.navi == 3) {
		maze[direction.row][direction.col].E_wall = exmaze[direction.row][direction.col].E_wall;
		maze[direction.row][direction.col].S_wall = exmaze[direction.row][direction.col].S_wall;
		maze[direction.row][direction.col].W_wall = exmaze[direction.row][direction.col].W_wall;
	}
	else if (direction.navi == 4) {
		maze[direction.row][direction.col].N_wall = exmaze[direction.row][direction.col].N_wall;
		maze[direction.row][direction.col].S_wall = exmaze[direction.row][direction.col].S_wall;
		maze[direction.row][direction.col].W_wall = exmaze[direction.row][direction.col].W_wall;
	}
}

void nextpath(struct mouse direction, struct cell maze[16][16], struct cell exmaze[16][16]) {
	direction.navi = 2;
	srand(time(NULL));
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	bool run = true;
	while (run == true) {
		updwalls(direction, maze, exmaze);

		int random = rand() % 4 + 1;
		if (direction.navi == 2) {
			if (random == 1 && (maze[direction.row][direction.col].E_wall == false) && (direction.col + 1 <= 15))
				direction.col += 1;
			else if (random == 2 && (maze[direction.row][direction.col].S_wall == false) && (direction.row + 1 <= 15)) {
				direction.navi = 3;
				direction.row += 1;
			}
			else if (random == 3 && (maze[direction.row][direction.col].W_wall == false) && (direction.col - 1 >= 0)) {
				direction.navi = 4;
				direction.col -= 1;
			}
			else if (random == 4 && (maze[direction.row][direction.col].N_wall == false) && (direction.row - 1 >= 0)) {
				direction.navi = 3;
				direction.row -= 1;
			}
			else
				continue;
		}

		else if (direction.navi == 3) {
			if (random == 1 && (maze[direction.row][direction.col].S_wall == false) && (direction.row + 1 <= 15))
				direction.row += 1;
			else if (random == 2 && (maze[direction.row][direction.col].W_wall == false) && (direction.col - 1 >= 0)) {
				direction.navi = 4;
				direction.col -= 1;
			}
			else if (random == 3 && (maze[direction.row][direction.col].N_wall == false) && (direction.row + 1 >= 0)) {
				direction.navi = 1;
				direction.row -= 1;
			}
			else if (random == 4 && (maze[direction.row][direction.col].E_wall == false) && (direction.col + 1 >= 0)) {
				direction.navi = 2;
				direction.col += 1;
			}
			else
				continue;
		}
		else if (direction.navi == 4) {
			if (random == 1 && (maze[direction.row][direction.col].W_wall == false) && (direction.col - 1 <= 15))
				direction.col -= 1;
			else if (random == 2 && (maze[direction.row][direction.col].S_wall == false) && (direction.row + 1 >= 0)) {
				direction.navi = 3;
				direction.row += 1;
			}
			else if (random == 3 && (maze[direction.row][direction.col].N_wall == false) && (direction.row + 1 >= 0)) {
				direction.navi = 1;
				direction.row -= 1;
			}
			else if (random == 4 && (maze[direction.row][direction.col].E_wall == false) && (direction.col + 1 >= 0)) {
				direction.navi = 2;
				direction.col += 1;
			}
			else
				continue;
		}
		else if (direction.navi == 1) {
			if (random == 1 && (maze[direction.row][direction.col].N_wall == false) && (direction.row - 1 <= 15))
				direction.row -= 1;
			else if (random == 2 && (maze[direction.row][direction.col].W_wall == false) && (direction.col - 1 >= 0)) {
				direction.navi = 4;
				direction.col -= 1;
			}
			else if (random == 3 && (maze[direction.row][direction.col].S_wall == false) && (direction.row + 1 >= 0)) {
				direction.navi = 3;
				direction.row += 1;
			}
			else if (random == 4 && (maze[direction.row][direction.col].E_wall == false) && (direction.col + 1 >= 0)) {
				direction.navi = 2;
				direction.col += 1;
			}
			else
				continue;
		}
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		if (elapsed_seconds.count() >= 15)
			run = false;
	}
		
}

void intexmaze(struct cell exmaze[16][16]) {
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			exmaze[i][j].N_wall = 0;
			exmaze[i][j].E_wall = 0;
			exmaze[i][j].S_wall = 0;
			exmaze[i][j].W_wall = 0;
		}
		cout << endl;
	}
	for (int y = 0; y <= 15; y++) {
		exmaze[0][y].N_wall = 1;
		exmaze[15][y].S_wall = 1;
	}
	for (int x = 0; x <= 15; x++) {
		exmaze[x][0].W_wall = 1;
		exmaze[x][15].E_wall = 1;
	}
}

void intexmaze_hor(struct cell exmaze[16][16], int hwalls[15][16]) {
	for (int colh = 0; colh <= 15; colh++) {
		for (int rowh = 0; rowh <= 15; rowh++) {
			if (hwalls[rowh][colh] == 1) {
				exmaze[rowh][colh].S_wall = 1;
				exmaze[rowh + 1][colh].N_wall = 1;
			}
		}
	}
}

void intexmaze_vert(struct cell exmaze[16][16], int vwalls[16][15]) {
	for (int rowv = 0; rowv <= 15; rowv++) {
		for (int colv = 0; colv <= 15; colv++) {
			if (vwalls[rowv][colv] == 1) {
				exmaze[rowv][colv].E_wall = 1;
				exmaze[rowv][colv + 1].W_wall = 1;
			}
		}
	}
}



void horizontalwalls(struct cell exmaze[16][16]) {
	int hwalls[15][16] = {
		1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0,
		0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0,
		1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0,
		0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0,
		1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0,
		0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0,
		0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0,
		1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0,
		0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
	};
	intexmaze_hor(exmaze, hwalls);
}

void verticalwalls(struct cell exmaze[16][16]) {
	int vwalls[16][15] = {
		0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1,
		0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
		0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1,
		0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
		1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1,
		1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1,
		1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1,
		1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
		1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1,
		0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1,
		1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1,
		0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
	intexmaze_vert(exmaze, vwalls);
}



void intmaze(struct mouse direction, struct cell maze[16][16]) {
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			maze[i][j].currentpathdistance = -1;
			maze[i][j].previous = false;
			maze[i][j].update = false;
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
	for (int y = 0; y <= 15; y++) {
		maze[0][y].N_wall = 1;
		maze[15][y].S_wall = 1;
	}
	for (int x = 0; x <= 15; x++) {
		maze[x][0].W_wall = 1;
		maze[x][15].E_wall = 1;
	}
	direction.row = 0;
	direction.col = 0;
	direction.navi = 2;
}

int main()
{
	struct cell maze[16][16];
	struct cell exmaze[16][16];
	struct mouse direction;
	direction.col = 0;
	direction.row = 0;
	direction.navi = 2;
	intexmaze(exmaze);
	intmaze(direction, maze);
	cout << "\n\n";
	cout << endl;
	verticalwalls(exmaze);
	cout << endl;
	horizontalwalls(exmaze);
	flood(maze);
	
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			cout << maze[i][j].currentpathdistance << "  ";
		}
		cout << endl;
	}
	nextpath(direction, maze, exmaze);
	flood(maze);
	cout << "\n\n";
	for (int i = 0; i <= 15; i++) {
		for (int j = 0; j <= 15; j++) {
			cout << maze[i][j].currentpathdistance << "  ";
		}
		cout << endl;
	}
}
