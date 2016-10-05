#pragma once


struct cell
{
	int currentpathdistance;
	int row;
	int col;
	bool N_wall;
	bool E_wall;
	bool S_wall;
	bool W_wall;
};