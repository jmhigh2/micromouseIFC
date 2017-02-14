#pragma once


struct cell
{
	int currentpathdistance;
	bool previous;
	bool update;
	bool N_wall;
	bool E_wall;
	bool S_wall;
	bool W_wall;
};

struct mouse
{
	int row;
	int col;
	int navi;
};