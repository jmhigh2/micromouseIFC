////Copyright(c) Mason A. Carnino
////
//
//#include <iostream>
//#include "location.h"
//#include <array>
//
//using namespace std;
//
//void intmaze(struct cell exmaze[16][16]) {
//	for (int i = 0; i <= 15; i++) {
//		for (int j = 0; j <= 15; j++) {
//			exmaze[i][j].N_wall = 0;
//			exmaze[i][j].E_wall = 0;
//			exmaze[i][j].S_wall = 0;
//			exmaze[i][j].W_wall = 0;
//		}
//		cout << endl;
//	}
//}
//
//void intexmaze(struct cell exmaze[16][16]) {
//	for (int i = 0; i <= 15; i++) {
//		int j = 0;
//		exmaze[i][j].W_wall = 1;
//		j = 15;
//		exmaze[i][j].E_wall = 1;
//	}
//	for (int j = 0; j <= 15; j++) {
//		int i = 0;
//		exmaze[i][j].N_wall = 1;
//		i = 15;
//		exmaze[i][j].S_wall = 1;
//	}
//}
//
//void innerwalls(struct cell exmaze[16][16]) {
//	int col0[] = { 0, 2, 4, 13};
//	int n = (sizeof(col0) / sizeof(*col0));
//	for (int x = 0; x <= n - 1; x++)
//		exmaze[col0[x]][0].S_wall = 1;
//}
//
//int main()
//{
//	struct cell exmaze[16][16];
//	intmaze(exmaze);
//	intexmaze(exmaze);
//	innerwalls(exmaze);
//	for (int i = 0; i <= 15; i++) {
//		for (int j = 0; j <= 15; j++) {
//			cout << exmaze[i][j].S_wall;
//		}
//		cout << endl;
//	}
//}