/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "config.c"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart1;

//User functions and defines

/*CHODE PROPERTIEs */
#if MOUSE_REV == 69
//straightaway speed
#define FWD_L 180 //110
#define FWD_R 178 //105
//RIGHT turn speed
#define RIGHT_L FWD_L
#define RIGHT_R 0
//LEFT turn speed
#define LEFT_L 0
#define LEFT_R FWD_R
//how far to turn for dead ends
#define PIVOT_ENC 485
//dead end stop condidtion
#define STOP_CONDITION 3620
//read walls
#define LEFT_THRESHOLD 900
#define RIGHT_THRESHOLD 900
#define FRONT_THRESHOLD 500
//transitoin encoders counters
#define FWD_TRANS 410
#define T_OFF 20 //negative for left farther

//wall transition buffers
#define IR_BUFFER 200
#define IR_DIFF 150
#define R_IR_CHANGE 1200 //400
#define L_IR_CHANGE 1300

//SPEED RUN BASE PWM SPEED
#define LEFT_BASE_SPEED 450
#define RIGHT_BASE_SPEED 450 //set right to be higher than left
//correction
#define RIGHT_CORRECTION 1700 //increase for closer to wall. decrease for farther from wall
#define LEFT_CORRECTION 1700 //increase for closer to wall
#define WALL_OFFSET 0 //LEFT - RIGHT

#elif MOUSE_REV == 1 //the mouse with no fucked up emitters
//straightaway speed
#define FWD_L 105
#define FWD_R 150
//RIGHT turn speed
#define RIGHT_L 120
#define RIGHT_R 0
//LEFT turn speed
#define LEFT_L 0
#define LEFT_R 135
//how far to turn for dead ends
#define PIVOT_ENC 445
//dead end stop condidtion
#define STOP_CONDITION 3750
//read walls
#define LEFT_THRESHOLD 500
#define RIGHT_THRESHOLD 500
#define FRONT_THRESHOLD 300

//transitoin encoders counters
#define FT_LENC1 350
#define FT_RENC1 350
#define FT_LENC2 570
#define FT_RENC2 570
#define T_OFF 0

//correction
#define RIGHT_CORRECTION 1350 //increase for closer to wall. decrease for farther from wall
#define LEFT_CORRECTION 1350 //increase for closer to wall
#define WALL_OFFSET -50 //negative for closer to right wall, positive for closer to left wall

//wall transition buffers
#define IR_BUFFER 1000
#define IR_DIFF 150
#define R_IR_CHANGE 400 //400
#define L_IR_CHANGE 450

#define LEFT_BASE_SPEED 300
#define RIGHT_BASE_SPEED 325 //set right to be higher than left
#endif


//STUFF THATS SAME FOR BOTH
//how many times to read wall
#define WHEEL_RADIUS 14 //millimeters
#define ECR_RES 4.09 // 4.09 counts per mm
#define WALL_SAMPLES 10
//read walls FWD
#define F_ENC1 500
//execute next move FWD
#define F_ENC2 696
//pivot right turn search
#define RT_LENC_1 482
//straight part of right turn search
#define RT_ENC_2 420  //405
//pivot left turn search
#define LT_RENC_1 470
//straight part of left turn search
#define LT_ENC_2 420
//read walls dead end
#define DEAD_ENC1 140
//execute next move dead end
#define DEAD_ENC2 425 //460
//left speed run
#define LEFT_L_SR1 100
#define LEFT_R_SR1 250
#define LEFT_L_SR2 120
#define LEFT_R_SR2 300
//left speed run distance
#define LEFT_LENC_SR_A 160
#define LEFT_RENC_SR_A 360
#define LEFT_LENC_SR_B 220
#define LEFT_RENC_SR_B 516

//right speed run
#define RIGHT_L_SR1 220 //changing radius
#define RIGHT_R_SR1 100
#define RIGHT_L_SR2 270 //constant radius
#define RIGHT_R_SR2 120
//right speed run distance
#define RIGHT_LENC_SR_A 360 //changing radius
#define RIGHT_RENC_SR_A 120
#define RIGHT_LENC_SR_B 516 //constant radius
#define RIGHT_RENC_SR_B 220

//maze shit
#define START_DIR NORTH

#define X_MAZE_SIZE 5
#define Y_MAZE_SIZE 5
#define X_FINAL 2
#define Y_FINAL 1

#if START_DIR == NORTH
#define X_START 0 //
#define Y_START Y_MAZE_SIZE - 1 //bottom left hand corner

#elif STAR_DIR == SOUTH
#define X_START 0 //
#define Y_START 0 //or [Y_MAZE_SIZE-1]

#endif


//FLASH STUFF
#define ADDR_FLASH_SECTOR_5  ((uint32_t)0x08040000) //use for walls
#define ADDR_FLASH_SECTOR_6  ((uint32_t)0x08080000) //use for settings
#define WALLS_ADDR  ADDR_FLASH_SECTOR_5
#define WALLS_ADDR_END ADDR_FLASH_SECTOR_5 - 1
#define SETTINGS_ADDR ADDR_FLASH_SECTOR_6
#define SETTINGS_ADDR_END ADDR_FLASH_SECTOR_6 - 1
static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;

__IO uint32_t data8_buffer = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClock_Config_old(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//Stop Searching
void Stop(void);

//Motor Control
void Set_Left(int speed, int direction);  //set left motor
void Set_Right(int speed, int direction); //set right motor
int Motor_Correction(int ir_disable); //motor correction
void Speed_Profiler(int *speeds, int num_squares);
void Speed_Test();

void Send_Buffers();
void Clear_Buffers();
void Reset_Flags();
void Reset_Maze();
void Fill_Center();
void Detect_Transition();

void Test_Clock();
void Measure_Speed();

//Sensors and Feedback
void Start_IR(void); //start DMA conversion of ADC values
void Stop_IR(void); //stop DMA conversion. used in interrupt routine
void Get_IR(int front_save, int side_save, int front_disable, int side_disable); //function to get all the IR values
void Reset_Counters(void);
void Set_Buzzer(int freq, int enable);

//Correction
void Dead_End_Correct(void); //algorithm to correct position in a dead end
int Emergency_Stop();

//In progress functions
void Readline(void);
void Read_Gyro(void);

//Debug Functions
void Transmit(char message[]); //send data
void Send_Debug(void); //send debug stuff...counters + ir data
void Save_State(void); //save state into debug buffer for analysis
void Send_State(void); //send debug

void Program_Walls_Flash();
void Read_Walls_Flash();

void Program_Settings();
void Read_Settings();
//Maze Functions
void Floodfill(int reverse, int path_search, int full);
int Get_Lowest_Square(int x, int y);

void Read_Walls();
void Update_Position();

void Update_Position_New();
int Get_Next_Move();
int Get_Next_Move_New();
int Get_Next_Dumb();
void Print_Maze();
void Get_Coordinate();

void Search();

void Calc_Optimal();
void Fast_Straights();
void Speed_Run();

void Turn_On_Lights();
void Turn_Off_Lights();

void Side_Calibrate(); //for sending side walls
void Front_Calibrate(); //for sensing front walls
void Deadend_Calibrate(); //for moving to the end of the dead end
void Correction_Calibrate();
void Adjuster();
void micros(uint32_t microseconds); //microsecond delay

void Forward_Search();
void Left_Search();
void Right_Search();
void Dead_End();

void Update_Sensors(int state);
void Search_Correction();
void Mark_Center();

void Forward_Search_New();
void Right_Search_New();
void Left_Search_New();
void Dead_End_New();
void Switch_Direction();

void Search_New();

void Forward_Spd(int num, char n_state, int add_distance);
void Left_Spd(int num, char n_state);
void Right_Spd(int num, char n_state);

int HAL_state = 0; //debug state
char tx_buffer[200]; //UART buffers
char rx_buffer[200];

uint32_t l_count = 0; //encoder counts
uint32_t r_count = 0;
uint32_t prev_l_count;
uint32_t prev_r_count;
uint32_t temp_l = 0;
uint32_t temp_r = 0;
int button_state = 0;

uint32_t time_count;
uint32_t prev_time_count;

uint32_t l_count_corr = 0;
uint32_t r_count_corr = 0;

uint32_t lenc_diff_corr = 0;
uint32_t renc_diff_corr = 0;

uint32_t l_dist;
uint32_t r_dist;

uint32_t lenc_diff = 0; //l_count - prev_l_count
uint32_t renc_diff = 0;

int m_correction = 0; //correction variable for motor speed

/* NOTE ON MAZE: X = COLUMN NUMBER, Y = ROW NUMBER */
uint32_t maze[X_MAZE_SIZE][Y_MAZE_SIZE]; //floodfill values from start to finish
//uint32_t search_maze[X_MAZE_SIZE][Y_MAZE_SIZE]; //floodfill values to go to specific coordinates
static uint8_t horiz_walls[X_MAZE_SIZE][Y_MAZE_SIZE - 1]; //horizontal walls have one less row
static uint8_t vert_walls[X_MAZE_SIZE - 1][Y_MAZE_SIZE]; //vertical walls have one less column
uint32_t visited_squares[X_MAZE_SIZE][Y_MAZE_SIZE];
uint32_t optimal_x[300];
uint32_t optimal_y[300];
uint32_t optimal_count = 0;
uint32_t search_x; //coordinate to be found
uint32_t search_y; //coordinate to be found
uint32_t coordinate_count = 0;

uint32_t final_x = 0;
uint32_t final_y = 0;

uint32_t x_coord = X_START; //maze position in maze
uint32_t y_coord = Y_START;
uint32_t prevx = 0; //previous maze position
uint32_t prevy = 0;
int cur_dir = START_DIR; //Start direction of maze. Can be SOUTH or NORTH
char optimal_path[100]; //regular path
char fast_path[100]; //new path with straightaways

//Mouse movement
int cur_move = FWD;
int next_move = FWD;

//on - off
int dif_l = 0;
int dif_r = 0;
int dif_rf = 0;
int dif_lf = 0;

int gyro_reading;

//buffers for reading derivative
static int lf_buffer[IR_BUFFER];
static int rf_buffer[IR_BUFFER];
static int l_buffer[IR_BUFFER];
static int r_buffer[IR_BUFFER];

int offsets[3] = {LEFT_CORRECTION, RIGHT_CORRECTION, WALL_OFFSET};

//values sampled multiple times
int front_l = 0;
int front_r = 0;
int lf_side = 0;
int rf_side = 0;

//interrupt variables
volatile int adc_conv = FALSE;
volatile unsigned int l = 0;
volatile unsigned int r = 0;
volatile unsigned int rf = 0;
volatile unsigned int lf = 0;

//variables to save decision stuff in
static unsigned int l_debug[DBG_BUFFER]; //static variables are set to zero
static unsigned int r_debug[DBG_BUFFER];
static unsigned int rf_debug[DBG_BUFFER];
static unsigned int lf_debug[DBG_BUFFER];
static unsigned int cur_debug[DBG_BUFFER];
static unsigned int turn_debug[DBG_BUFFER];
static unsigned int l_count_debug[DBG_BUFFER];
static unsigned int r_count_debug[DBG_BUFFER];
static unsigned int prev_l_debug[DBG_BUFFER];
static unsigned int prev_r_debug[DBG_BUFFER];
int dbg_count = 0;

static int buff_count = 0; //Get_IR

//turn flags. These will be set when mouse is done pivoting and should be going straight. Can correct during this time
int r_turnflag = FALSE; //end right pivot
int l_turnflag = FALSE;  //end left pivot
int dead_flag = ARRIVE; //different stages of dead end
int fwd_flag = FALSE; //read walls on forward
int l_transition_flag = FALSE; //transition on left detected
int r_transition_flag = FALSE; //transition on right detected
int transition_flag = FALSE; //right or left transition detected
int done_flag = FALSE; //done searching flag, reset on search beginning
int send_debug = FALSE; //send debug flag
int reverse_flag = FALSE; //end goal of search phase. TRUE for returning to start
int search_flag = FALSE; //try to find all the squares on the current optimal path
int start_flag = TRUE; //used for speed run
int speedrunturn = FALSE;
int disable_reset = FALSE;

//system flags. stop_flag stops everything. Press button to enable everything. Debug flag send debug data while sitting
int stop_flag = TRUE; //system stop flag.
int debug_flag = FALSE;
int buzzer_flag = FALSE;

int dem1 = 0;
int dem2 = 0;
int dem3 = 0;

//SPI Buffer
uint8_t aTxBuffer[] = {0xe3, 0x00, 0x00, 0x00};
uint8_t aRxBuffer[4]; //buffer to read gyro data

enum {ADC_VAL_BUFFER_LENGTH = 32}; //DMA Buffer size
uint32_t ADC_valbuffer[ADC_VAL_BUFFER_LENGTH];

int main(void)
{
   //STARTUP

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
#if MOUSE_REV == 1
  MX_SPI1_Init();
#endif
  MX_TIM1_Init(); //encoder left
  MX_TIM2_Init(); //pwm shit
  MX_TIM5_Init(); //timer
  //MX_TIM3_Init();
  MX_TIM4_Init(); //encoder right
  //MX_TIM9_Init();
  MX_USART1_UART_Init();

  //buzzer
  //HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  //start PWM for motors
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  Set_Left(0, FORWARD); //set mouse to sit
  Set_Right(0, FORWARD);
  //END STARTUP
  //HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);
  //HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
  //HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, ON);
  //HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);

  //Read_Settings();

  //MAIN INFINITE PROGRAM LOOP aka ready loop
  while (1)
  {

	  if (stop_flag == FALSE || send_debug == TRUE || dem1 == TRUE || dem2 == TRUE || dem3 == TRUE) { //press top button to activate IR sensors
		Update_Sensors(BARE);
	    if (send_debug == TRUE) {
	  	  Send_Debug();
	    }
	  //else if (debug_flag == TRUE && dif_l > 1500) {
	  	  //Correction_Calibrate();
	  //}
	  }

	  if (dem1 == TRUE && dif_l > 2000) { //test going straight without correction
		  Turn_On_Lights();
		  HAL_Delay(1000);
		  Turn_Off_Lights();
		  Set_Left(FWD_L, FORWARD);
		  Set_Right(FWD_R, FORWARD);
		  HAL_Delay(4000);
		  Set_Right(0, 0);
		  Set_Left(0, 0);
		  dem1 = FALSE;
	  }

	  if (dem2 == TRUE && dif_l > 2000) { //test correction
		  Turn_On_Lights();
		  HAL_Delay(1000);
		  Turn_Off_Lights();
		  Set_Left(FWD_L, FORWARD);
		  Set_Right(FWD_R, FORWARD);
		  while(1) {
			  Update_Sensors(BARE);
			  Search_Correction();
		  }
		  dem2 = FALSE;

	 }

	  if (dem3 == TRUE && dif_l > 2000) { //debug options
		  Turn_On_Lights();
		  HAL_Delay(1000);
		  Turn_Off_Lights();
		  //Speed_Run("blbs");
		  Measure_Speed();
		  dem3 = FALSE;
	  }

	  if (stop_flag == FALSE && dif_r > 2000) { //read walls from memory

		  Read_Walls_Flash();
		  Print_Maze();
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ON);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  disable_reset = TRUE;

	  }

	  if (stop_flag == FALSE && dif_l > 2000) { //start searching (place finger in front)
		  if (disable_reset == FALSE) {
			  Reset_Maze();
		  }
		  Calc_Optimal();
		  Get_Coordinate();

		  while (coordinate_count > 0) { //if all walls found go directly to speed run
			  reverse_flag = FALSE;
			  stop_flag = FALSE;
			  Search(); //reached the end
			  if (done_flag == TRUE) {
				  disable_reset = TRUE;
				  Fill_Center();
	 			  Program_Walls_Flash();
#if DEBUG == TRUE
			 	  Print_Maze();
#endif
			  }
			  reverse_flag = TRUE;
			  stop_flag = FALSE;
			  Search(); //go back
#if DEBUG == TRUE
			  Print_Maze();
#endif
			  Program_Walls_Flash();
			  horiz_walls[0][Y_MAZE_SIZE - 2] = FALSE; //ignore the first wall
			  Calc_Optimal();
			  Get_Coordinate();
		  }

		  Turn_On_Lights(); //turn on lights to show that its ready to speed run

		  stop_flag = TRUE;
		  do {
		  HAL_Delay(100);
		  } while(stop_flag == TRUE); //wait for button press

		  Calc_Optimal();
		  Fast_Straights();

		  Speed_Run(fast_path);

	  } //if front wall > 1500
  HAL_Delay(500); //ONLY CHECK FOR FINGER every half second. If you check to quickly it'll never start
  } //ready loop
} //main function

void Adjuster() {

int left = 0;
int right = 0;
int l_dic = FORWARD;
int r_dic = FORWARD;

left = 2500 - dif_l;
if (left < 0) {

	l_dic = BACKWARD;
	left = abs(left);
}

right = 2500 - dif_r;
if (right < 0) {

	r_dic = BACKWARD;
	right = abs(right);
}

Set_Left(left, l_dic);
Set_Right(right, r_dic);

}

void Correction_Calibrate() {

#define VAL 20

Read_Settings();
Reset_Counters();

int val;

//modify offsets


HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);

stop_flag = TRUE;
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
sprintf(tx_buffer, "LEFT CORRECTION VALUE: %d \r\n", offsets[0] + l_count/VAL - r_count/VAL);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[0] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[0] = val;
}

HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);

stop_flag = TRUE;
Reset_Counters();
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
sprintf(tx_buffer, "RIGHT CORRECTION VALUE: %d \r\n", offsets[1] + l_count/VAL - r_count/VAL);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[1] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[1] = val;
}

HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ON);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);

Reset_Counters();
stop_flag = TRUE;
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
sprintf(tx_buffer, "WALL OFFSET VALUE: %d \r\n", offsets[2] + l_count/VAL - r_count/VAL);
Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[2] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[2] = val;
}
Program_Settings();

/*
HAL_GPIO_WritePin(GPIOD, LED5_Pin, OFF); //Turn on LEDs to indicate it is searching
HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED1_Pin, ON);

stop_flag = TRUE;
do {
	Get_IR(FALSE, FALSE, FALSE, FALSE);
} while (stop_flag == TRUE);


HAL_GPIO_WritePin(GPIOD, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED1_Pin, OFF);

HAL_Delay(1000); //delay before start to get finger out of the way

for (int i = 0; i < 2; i++) {

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);
	Reset_Counters();

	do {
	Update_Sensors(BARE);
	Search_Correction();
	} while (lenc_diff < F_ENC2*4 && renc_diff < F_ENC2*4);

	Dead_End_Correct();
}
Stop();
*/


}

void Deadend_Calibrate() {

	Clear_Buffers();
	buff_count = 0;
	int l_test = 0;
	int r_test = 0;

	for (int i = 0; i < IR_BUFFER; i++) {
		Get_IR(TRUE, FALSE, FALSE, TRUE);
	}

	for (int i = 0; i < IR_BUFFER; i++) {

	l_test = l_test + l_buffer[i];
	r_test = r_test + r_buffer[i];

	}

	l_test = l_test/IR_BUFFER;
	r_test = r_test/IR_BUFFER;

	sprintf(tx_buffer, "DeadEnd: \r\nLEFT: %d    RIGHT: %d      OFFSET(LEFT - RIGHT): %d \r\n", l_test, r_test, l_test - r_test);
	Transmit(tx_buffer);
}

void Search_New() {

	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

	HAL_Delay(1000); //delay before start to get finger out of the way

	if (reverse_flag == FALSE) {
		Reset_Maze(); //reset walls
		Read_Walls();
	}

	Clear_Buffers();
	Reset_Counters();

	Reset_Flags(); //reset flags
	Floodfill(reverse_flag, search_flag, FALSE);

	Update_Position_New(); //move to next square (first move is always forward)

	Set_Left(FWD_L, FORWARD); //start going straight. get moving
	Set_Right(FWD_R, FORWARD);

	while(1) {

		switch(cur_move) { //perform whatever move is queued
		case FWD:
			Forward_Search_New();
			break;

		case RIGHT: //break up turn into turn and accelerate
			Right_Search_New();
			break;

		case LEFT:
			Left_Search_New();
			break;

		case DEAD:
			Dead_End_New();
			break;
		} //walls are read during the state machine

		prev_l_count = l_count; //get current counter values
		prev_r_count = r_count;

		if (maze[x_coord][y_coord] == 0) { //reached target square
			if (search_flag == FALSE) { //reached center
				sprintf(tx_buffer, "|||FINAL||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
				Transmit(tx_buffer);
				Dead_End_Correct();
				Switch_Direction();
				break; //break out of while loop
			}

			else { //looking along for the optimal path still
				Calc_Optimal(); //recalculate optimal path
				Get_Coordinate(); //get any coordinates that haven't been seen yet
				if (coordinate_count == 0) { //if none, go back to start
					search_flag = FALSE;
				}
				Floodfill(reverse_flag, search_flag, FALSE); // go back to center
				next_move = Get_Next_Move(); //queue up next move
				Update_Position_New(); //update the position so walls can be read
				cur_move = next_move;
			}
		}
		else {
			next_move = Get_Next_Move(); //queue up next move
			Update_Position_New(); //update the position so walls can be read
			cur_move = next_move;
		}
	}

}

void Search() {

	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

	HAL_Delay(1000); //delay before start to get finger out of the way

	if (disable_reset == FALSE) {
		Reset_Maze(); //reset walls
	}

	Read_Walls();

	Clear_Buffers();
	Reset_Counters();
	if (reverse_flag == TRUE) {
		Mark_Center();
	}

	Reset_Flags(); //reset flags
	Floodfill(reverse_flag, search_flag, FALSE);

	Update_Position(); //move to next square

	Set_Left(FWD_L, FORWARD); //start going straight. get moving
	Set_Right(FWD_R, FORWARD);

    while(1) { //searching loop //while(maze[x_coord][y_coord] != 0)

    	if (cur_move == FWD && transition_flag == FALSE) {
    		Update_Sensors(FWD_SEARCH); //detect transitions
    	}
    	else {
    		Update_Sensors(TURN_SEARCH); //dont detect transitions. Just get side sensors
    	}

    	if  (cur_move == FWD || r_turnflag > 0 || l_turnflag > 0 || dead_flag > 0) {
    		Search_Correction(); //motor correction
    	}

    	if (Emergency_Stop() || stop_flag == TRUE) {
    		Stop();
    		break;
    	}

     	//wall to no wall transition, or no wall to wall transition
    	Detect_Transition();

    	switch (cur_move) { //main case statement. While moving, check distance traveled. If 1 unit has been covered, execute next move
    	//will eventually combine with above statement
    	case FWD:
    		Forward_Search();
    		break;

    	case RIGHT: //break up turn into turn and accelerate
    		Right_Search();
    		break;

    	case LEFT:
    		Left_Search();
    		break;

    	case DEAD:
    		Dead_End();
    		break;
    	} //switch
    } //searching loop
}

void Forward_Search_New() {
	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);
	transition_flag = FALSE;

	do {

		Update_Sensors(FWD_SEARCH);
		if (transition_flag == FALSE && (r_transition_flag == TRUE || l_transition_flag == TRUE)) { //detect transition and start counting from there

			prev_l_count = l_count;
			prev_r_count = r_count;
			transition_flag = TRUE;
			lenc_diff = 0;
			renc_diff = 0;

			if (l_transition_flag == TRUE) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);
			}
			else if (r_transition_flag == TRUE) {
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
			}
		}

		Search_Correction();

	} while ((transition_flag == FALSE && lenc_diff < F_ENC1 && renc_diff < F_ENC1) || (transition_flag == TRUE && lenc_diff < FWD_TRANS/2 && renc_diff < FWD_TRANS/2));

	Read_Walls();
	Save_State();

	do {
		Update_Sensors(BARE);
		Search_Correction();

	} while ((transition_flag == FALSE && lenc_diff < F_ENC2 && renc_diff < F_ENC2) || (transition_flag == TRUE && lenc_diff < FWD_TRANS && renc_diff < FWD_TRANS));

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);


}

void Forward_Search() {

	if (transition_flag == TRUE) { //if theres a transition sense use this position instead

		if (fwd_flag == FALSE && (lenc_diff_corr >= l_dist*3/4 || renc_diff_corr >= r_dist*3/4)) { //330

			fwd_flag = TRUE;

			if (done_flag == TRUE) {
				next_move = DEAD;
			}

			else {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
			}

			Save_State();
		}

		else if (fwd_flag == TRUE && (lenc_diff_corr >= l_dist || renc_diff_corr >= r_dist)) { //620

			prev_l_count = l_count; // Save encoder values
			prev_r_count = r_count;
			fwd_flag = FALSE;
			transition_flag = FALSE;
			Clear_Buffers();
			Turn_Off_Lights();


			switch (next_move) { //check if motor speeds have to change with next move

			case RIGHT:
				Set_Left(RIGHT_L, FORWARD); //need to make right pivot
				Set_Right(RIGHT_R, FORWARD);
				break;

			case LEFT:
				Set_Left(LEFT_L, FORWARD); //need to make left pivot
				Set_Right(LEFT_R, FORWARD);
				break;
			}
			cur_move = next_move; //execute next move
		}
	}

	else { //if theres no transition sensed, then just use absolute position

	if (fwd_flag == FALSE && (lenc_diff >= F_ENC1 || renc_diff >= F_ENC1))
	{
		fwd_flag = TRUE;

		if (done_flag == TRUE) {
		next_move = DEAD;
		}

		else {
		Read_Walls();
		next_move = Get_Next_Move();
		Update_Position();
		}

		Save_State();
	}

	else if (fwd_flag == TRUE && (lenc_diff >= F_ENC2 || renc_diff >= F_ENC2))
	{ //left and right wheel moving at same speed. If statement checks if distance has been covered

		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

		prev_l_count = l_count; //no change. keep going straight. Save encoder values
		prev_r_count = r_count;
		fwd_flag = FALSE;
		transition_flag = FALSE;
		Clear_Buffers();

		switch (next_move) { //check if motor speeds have to change with next move
		case FWD:
			break;

		case RIGHT:
			Set_Left(RIGHT_L, FORWARD); //need to make right pivot
			Set_Right(RIGHT_R, FORWARD);
			break;

		case LEFT:
		    Set_Left(LEFT_L, FORWARD); //need to make left pivot
		    Set_Right(LEFT_R, FORWARD);
		    break;

		case DEAD:
		    break;
		}
		cur_move = next_move; //execute next move
	}
	}
}

void Forward_Spd(int num, char n_state, int add_distance) {

int l_distance = 0;
int r_distance = 0;

if (add_distance == TRUE) { //if coming off of a right or left turn
	l_distance = F_ENC2*num + 415;
	r_distance = F_ENC2*num + 415;
}

else { //normal distance
	l_distance = F_ENC2*num;
	r_distance = F_ENC2*num;
}

int base_l = LEFT_BASE_SPEED;
int base_r = RIGHT_BASE_SPEED;

int speeds[4] = {0, 0, 0, 0}; //left/right base , left/right corrected
Reset_Counters();


speeds[0] = base_l + (num - 6)*30; //scale PWM, scale PWM
speeds[1] = base_r + (num - 6)*30;


Set_Left(speeds[0], FORWARD); //start slow
Set_Right(speeds[1], FORWARD);

do {
Update_Sensors(TIME);
Speed_Profiler(speeds, num);
Set_Left(speeds[2], FORWARD);
Set_Right(speeds[3], FORWARD);
} while (lenc_diff < l_distance && renc_diff < r_distance); // while ((start_flag == TRUE && lenc_diff < F_ENC2*num/10 - 170 && renc_diff < F_RENC2*num/10 - 170) || (start_flag == FALSE && lenc_diff < F_ENC2*num/10 && renc_diff < F_RENC2*num/10));


prev_l_count = l_count;
prev_r_count = r_count;
speedrunturn = FALSE;
}

void Left_Search_New() {
	Set_Left(LEFT_L, FORWARD);
	Set_Right(LEFT_R, FORWARD);

	do {
		r_count = __HAL_TIM_GET_COUNTER(&htim4);
		renc_diff = r_count - prev_r_count;
	} while (renc_diff < LT_RENC_1);

	prev_l_count = l_count;
	prev_r_count = r_count;

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < LT_ENC_2*2/3 && renc_diff < LT_ENC_2*2/3);

	Read_Walls();
	Save_State();

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < LT_ENC_2 && renc_diff < LT_ENC_2);

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD);
}

void Left_Search() {

	if (l_turnflag == ARRIVE && renc_diff >= LT_RENC_1) { //finished making turn. left and right wheel don't travel at same speeds

		Clear_Buffers();
		Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		Set_Right(FWD_R, FORWARD);


		l_turnflag = PEEK;
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;

		Save_State();
	}

	else if ((l_turnflag == PEEK) && (lenc_diff >= LT_ENC_2*3/4 || renc_diff >= LT_ENC_2*3/4)) {

		if (done_flag == TRUE) {
			//Read_Walls();
			next_move = DEAD;
		}

		else {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
		}
		l_turnflag = EXECUTE;
	}

	else if ((l_turnflag == EXECUTE) && (lenc_diff >= LT_ENC_2 || renc_diff >= LT_ENC_2)) { //made it to same point

		l_turnflag = ARRIVE;
		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		Save_State();
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;

		switch (next_move) { //need to change direction or nah

		case LEFT:
			Set_Left(LEFT_L, FORWARD); //need to make right turn again
			Set_Right(LEFT_R, FORWARD);
			break;

		case RIGHT:
			Set_Left(RIGHT_L, FORWARD); //need to make right turn again
			Set_Right(RIGHT_R, FORWARD);
			break;

		}
		cur_move = next_move;
	} //case LEFT
}

void Left_Spd(int num, char n_state) {

for (int i = 0; i < num; i++) {

Set_Left(LEFT_L, FORWARD); //accelerate
Set_Right(LEFT_R, FORWARD);

do {
Update_Sensors(69);
} while (renc_diff < LT_RENC_1);

prev_l_count = l_count;
prev_r_count = r_count;

Set_Left(FWD_L, FORWARD); //accelerate
Set_Right(FWD_R, FORWARD);

if ((n_state == 'b' || n_state == 'c' || n_state == 'd' || n_state == 'e') && (num < 2 || i > 0) ) {
	speedrunturn = TRUE;
	return;
}
//if next state is forward, go directly to FORWARD Speed

do {
Update_Sensors(TURN_SEARCH);
Search_Correction();
} while (lenc_diff < LT_ENC_2 && renc_diff < LT_ENC_2);

prev_l_count = l_count;
prev_r_count = r_count;

}

/*
do {
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
} while (lenc_diff < LEFT_LENC_SR_A*num && renc_diff < LEFT_RENC_SR_A*num);

Set_Left(LEFT_L_SR2, FORWARD); //accelerate
Set_Right(LEFT_R_SR2, FORWARD);

do {
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
} while (lenc_diff < LEFT_LENC_SR_B*num && renc_diff < LEFT_RENC_SR_B*num);

Set_Left(LEFT_L_SR1, FORWARD); //accelerate
Set_Right(LEFT_R_SR1, FORWARD);

do {
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
} while (lenc_diff < (LEFT_LENC_SR_A + LEFT_LENC_SR_B)*num && renc_diff < (LEFT_RENC_SR_A + LEFT_RENC_SR_B)*num);


prev_l_count = l_count;
prev_r_count = r_count;
*/


}

void Right_Search_New() {
	Set_Left(RIGHT_L, FORWARD);
	Set_Right(RIGHT_R, FORWARD);

	do {
		l_count = __HAL_TIM_GET_COUNTER(&htim1);
		lenc_diff = l_count - prev_l_count;
	} while (lenc_diff < RT_LENC_1);

	prev_l_count = l_count;
	prev_r_count = r_count;

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < RT_ENC_2*2/3 && renc_diff < RT_ENC_2*2/3);

	Read_Walls();

	Save_State();

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < RT_ENC_2 && renc_diff < RT_ENC_2);

	Set_Left(0, FORWARD); //brake
	Set_Right(0, FORWARD);

}



void Right_Search() {

	if (r_turnflag == ARRIVE && lenc_diff >= RT_LENC_1) { //finished making turn. left and right wheel don't travel at same speeds

		Clear_Buffers();

		Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		Set_Right(FWD_R, FORWARD);

		r_turnflag = PEEK;
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;

		Save_State();
	}

	else if ((r_turnflag == PEEK) && (lenc_diff >= RT_ENC_2*3/4 || renc_diff >= RT_ENC_2*3/4)) {
		if (done_flag == TRUE) {
			next_move = DEAD;
			//Read_Walls();
		}
		else {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
		}
		r_turnflag = EXECUTE;

	}

	else if ((r_turnflag == EXECUTE) && (lenc_diff >= RT_ENC_2 || renc_diff >= RT_ENC_2)) { //made it to same point. execute next direction

		r_turnflag = ARRIVE;
		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;

		switch (next_move) { //need to change direction or nah

		case RIGHT:
			Set_Left(RIGHT_L, FORWARD); //need to make right turn again
			Set_Right(RIGHT_R, FORWARD);
			break;

		case LEFT:
			Set_Left(LEFT_L, FORWARD); //need to make right turn again
			Set_Right(LEFT_R, FORWARD);
			break;

		}
		cur_move = next_move;
	} //case RIGHT
}

void Right_Spd(int num, char n_state) {

for (int i = 0; i < num; i++) {

	Set_Left(RIGHT_L, FORWARD); //accelerate
	Set_Right(RIGHT_R, FORWARD);


	do {
		Update_Sensors(69);
	} while (lenc_diff < RT_LENC_1);

	prev_l_count = l_count;
	prev_r_count = r_count;

	Set_Left(FWD_L, FORWARD); //accelerate
	Set_Right(FWD_R, FORWARD);

	if ((n_state == 'b' || n_state == 'c' || n_state == 'd' || n_state == 'e') && (num < 2 || i > 0) ) {
		speedrunturn = TRUE;
		return;
	}

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < RT_ENC_2 && renc_diff < RT_ENC_2);

	prev_l_count = l_count;
	prev_r_count = r_count;
	}
	 //case RIGHT

/*
do {
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
} while (lenc_diff < RIGHT_LENC_SR_B*num && renc_diff < RIGHT_RENC_SR_B*num);

Set_Left(RIGHT_L_SR1, FORWARD); //accelerate
Set_Right(RIGHT_R_SR1, FORWARD);

do {
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
} while (lenc_diff < (RIGHT_LENC_SR_A + RIGHT_LENC_SR_B)*num && renc_diff < (RIGHT_RENC_SR_A + RIGHT_RENC_SR_B)*num);
*/
}

void Turn_Profiler(int *speeds, int direction, int num_squares) {

	//inside wheel speed 1 (same as 3)
	//outside wheel speed 1 (same as 3)
	//inside wheel speed 2
	//outside wheel speed 2
	//outside wheel distance
	//inside wheel distance

	uint32_t left_speed = 180*1000000/700*(lenc_diff - temp_l)/(time_count - prev_time_count); // in mm/s
	uint32_t right_speed = 180*1000000/700*(renc_diff - temp_r)/(time_count - prev_time_count); // in mm/s

	/*if (inside_wheel distance < inside threshold) {
	 correction}
	 else if (inside_wheel distance < inside threshold 2) { speed correction
	 }
	 else { speed correction
	 }
	 */
}

void Speed_Profiler(int *speeds, int num_squares) {

	static int debug_count = 0;
	uint32_t top_speed = 1500/(16 - num_squares); //mm/s
	uint32_t end_speed = 330; //mm/s
	uint32_t left_speed = 0;
	uint32_t right_speed = 0;
	uint32_t distance_left;
	int left_acceleration = 0;
	int right_acceleration = 0;
	static uint32_t prev_left_speed = 0;
	static uint32_t prev_right_speed = 0;
	/*
	if (time_count - prev_time_count > 50000) { //sample every 50 ms

		if (prev_time_count > time_count) { //overflow
			int diff = prev_time_count - 0xffffffff;
			prev_time_count = (unsigned int) diff;
		}

		left_speed = 180*1000000/700*(l_count - temp_l)/(time_count - prev_time_count); // in mm/s
		right_speed = 180*1000000/700*(r_count - temp_r)/(time_count - prev_time_count); // in mm/s
		left_acceleration = 1000000*(left_speed - prev_left_speed)/(time_count - prev_time_count);
		right_acceleration = 1000000*(right_speed - prev_right_speed)/(time_count - prev_time_count);


		temp_l = l_count;
		temp_r = r_count;
		prev_time_count = time_count;
*/
		distance_left = (num_squares*F_ENC2 + speedrunturn*415) - (lenc_diff + renc_diff)/2;

		if (distance_left < (F_ENC2*num_squares + speedrunturn*415)/8) { //decelerate to end speed
			Get_IR(FALSE, FALSE, TRUE, FALSE);
			if (dif_lf > LEFT_CORRECTION + 500 || dif_rf > RIGHT_CORRECTION + 500) {
				m_correction = Motor_Correction(FALSE); //motor correction
			}
			else {
				m_correction = Motor_Correction(TRUE); //motor correction
				//m_correction = left_speed - right_speed
			}
			speeds[2] = FWD_L + m_correction;
			speeds[3] = FWD_R - m_correction;
			//brake until a certain speed reached
		}

		else { //accelerate

			Get_IR(FALSE, FALSE, TRUE, FALSE);
			if (dif_lf > 2500 || dif_rf > 2500) {
				m_correction = Motor_Correction(FALSE); //motor correction no infrared
			}
			else {
				m_correction = Motor_Correction(TRUE); //motor correction with infrared
			}

			speeds[2] = speeds[0] + m_correction; //return speeds
			speeds[3] = speeds[1] - m_correction;
		}
		prev_left_speed = left_speed;
		prev_right_speed = right_speed;


#if DEBUG == TRUE
	if (debug_count % 1000 == 0) {
		sprintf(tx_buffer, "Left Speed %u mm/s   Right Speed %u mm/s \r\n", left_speed, right_speed);
		Transmit(tx_buffer);
		sprintf(tx_buffer, "PREV TIME %d   NOW TIME: %d \r\n", prev_time_count, time_count);
		Transmit(tx_buffer);
		sprintf(tx_buffer, "TEMP L %d   TEMP R: %d \r\n", temp_l, temp_r);
		Transmit(tx_buffer);
		debug_count = 0;
	}
	debug_count++;
#endif
	//}
}

void Dead_End_New() {

	Dead_End_Correct(); //should be aligned in middle of square in the back

	Floodfill(reverse_flag, search_flag, FALSE); //floodfill maze
	Print_Maze();
	Reset_Counters();
	Clear_Buffers();

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD); //move forward to next square

	do {
		Update_Sensors(TURN_SEARCH);
		Search_Correction();
	} while (lenc_diff >= 650 || renc_diff >= 650);

}

void Dead_End() {

	if (dead_flag == ARRIVE) {

		if (done_flag == TRUE) {
			//Read_Walls();
		}

		Dead_End_Correct(); //should be aligned in middle of square

		Floodfill(reverse_flag, search_flag, FALSE);
		Reset_Counters();
		Clear_Buffers();
		if (done_flag == TRUE) {
			cur_move = PAUSE;
		}

		else {
		Set_Left(FWD_L, FORWARD);
		Set_Right(FWD_R, FORWARD); //move forward to next square
		dead_flag = PEEK;
		}
	}

	else if (dead_flag == PEEK && (lenc_diff >= DEAD_ENC1 || renc_diff >= DEAD_ENC1)) { //get next state

		dead_flag = EXECUTE;
		next_move = Get_Next_Move();
		Update_Position();

		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;
	}
	//after correction
	else if (dead_flag == EXECUTE && (lenc_diff > DEAD_ENC2 || renc_diff >= DEAD_ENC2)) {
		dead_flag = ARRIVE;
		prev_l_count = l_count;
		prev_r_count = r_count;

		switch (next_move) { //need to change direction or nah

		case LEFT:
			Set_Left(LEFT_L, FORWARD); //need to make right turn again
			Set_Right(LEFT_R, FORWARD);
			break;
		case RIGHT:
			Set_Left(RIGHT_L, FORWARD); //need to make right turn again
			Set_Right(RIGHT_R, FORWARD);
			break;
		}
		cur_move = next_move;
	}
}

//DONT USE YET
void Set_Buzzer(int freq, int enable) {

	if (enable == TRUE) {

	int duty = freq/2;

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 8;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = freq;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
	  Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	//pulse needs to be half of period (50% duty cycle)

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	  Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	}
}

int Emergency_Stop() {

/*if (dif_l > STOP_CONDITION || dif_r > STOP_CONDITION || cur_move == PAUSE) //Emergency STOP conditions
{
	  if (cur_move == FWD || cur_move == PAUSE) //|| l_turnflag == TRUE || r_turnflag == TRUE)
	  {
		  return TRUE;
	  }
}
*/
if (cur_move == PAUSE) {
	return TRUE;
}
return FALSE;
}

void Update_Sensors(int state) {

	switch(state) {
	case BARE: //disable nothing, save nothing, only correction
		Get_IR(FALSE, FALSE, FALSE, FALSE); //
	break;
	case TURN_SEARCH: //disable front, only detect sides
		Get_IR(FALSE, FALSE, TRUE, FALSE);
	break;
	case FWD_SEARCH: //forward searching, detect transitions
		Get_IR(FALSE, TRUE, TRUE, FALSE);
	break;
	case TIME:
		time_count = __HAL_TIM_GET_COUNTER(&htim5);
	break;
	}

	l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
	r_count = __HAL_TIM_GET_COUNTER(&htim4);
	lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
	renc_diff = r_count - prev_r_count;

}

void Search_Correction() {

	m_correction = Motor_Correction(FALSE);
	Set_Left(FWD_L + m_correction, FORWARD);
	Set_Right(FWD_R - m_correction, FORWARD);

}

void Detect_Transition() {

	if (transition_flag == TRUE) {
		lenc_diff_corr = l_count - l_count_corr; //update corrected encoder counts
		renc_diff_corr = r_count - r_count_corr;
	}

	//fwd_flag == FALSE &&
	else if (cur_move == FWD && (r_transition_flag == TRUE || l_transition_flag == TRUE)) { //detect transition and start counting from there
		l_count_corr = l_count; //save current count
		r_count_corr = r_count;
		transition_flag = TRUE; //enable transition flag
		lenc_diff_corr = 0; //reset corrected difference
		renc_diff_corr = 0;
		if (l_transition_flag == TRUE) { //left leds to represent left
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ON);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);

			l_dist = FWD_TRANS - T_OFF;
			r_dist = FWD_TRANS - T_OFF;
		}
		else if (r_transition_flag == TRUE) { //right leds to represent right
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, ON);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, ON);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ON);
			l_dist = FWD_TRANS;
			r_dist = FWD_TRANS;
		}
		r_transition_flag = FALSE; //reset transition flags after transition is detected
		l_transition_flag = FALSE;
		if (debug_flag == TRUE) { //for debug at competitions
			Stop(); //stop
			while(1); //infinite loop
		}
		//break;
	}

}

int Motor_Correction(int ir_disable) {

	//int D = 100; //parameters
	int P_norm = 70;
	int P_speed = 100;

	int errorP = 0;
	//int errorD = 0;
	//int oldErrorP = 0;

	int correction = 0;

	if (ir_disable == TRUE) {
		errorP = ((r_count - prev_r_count) - (l_count - prev_l_count))*10;
		return errorP/P_speed;
	}

	else {
		if (dif_lf > LEFT_THRESHOLD && dif_rf > RIGHT_THRESHOLD && r_transition_flag == FALSE && l_transition_flag == FALSE) { //both walls available
			errorP = (dif_lf - (dif_rf + offsets[2])); //dif_rf + 500 for primary
			//errorD = errorP - oldErrorP;
		}
		else if (dif_lf >= LEFT_THRESHOLD && dif_rf <= RIGHT_THRESHOLD) { //only left wall to correct. Optimal reading should be 1600
			errorP = (dif_lf - offsets[0]); //75 is correction factor. Left side needs more corrections for some reason
			//errorD = errorP - oldErrorP;
		}
		else if (dif_rf >= RIGHT_THRESHOLD && dif_lf <= LEFT_THRESHOLD ) {//only right wall to correct. Optimal reading should be 1500. (200 is offset)
			errorP = (offsets[1] - dif_rf); //100 is correction factor. Right side needs less correction
			//errorD = errorP - oldErrorP;
		}
		else if (dif_lf <= LEFT_THRESHOLD && dif_rf <= RIGHT_THRESHOLD) { //use encoders when there's no walls available
			errorP = ((r_count - prev_r_count) - (l_count - prev_l_count))*6;
		}	//when right side has moved more, add more to left side
	}
	//motor correction adds to left motor
	correction = errorP/P_norm; //+ errorD/D; //P and D are tuning parameters
	//oldErrorP = errorP;
	if (abs(correction) < FWD_R) {
	return correction;
	}
	else
	{
		return 0;
	}

}

#define EQUAL_VAL 300

void Dead_End_Correct(void) {

 //align while going into square
Get_IR(FALSE, FALSE, FALSE, FALSE);

if (dif_l > FRONT_THRESHOLD || dif_r > FRONT_THRESHOLD) { //if there's a front wall

	if (dif_lf >= LEFT_THRESHOLD && dif_rf >= RIGHT_THRESHOLD) { //both left and right wall
		do { //both left and right wall
			Get_IR(FALSE, FALSE, FALSE, FALSE);
			m_correction = (dif_lf - (dif_rf + offsets[2]))/30;
			Set_Left(FWD_L + m_correction, FORWARD);
			Set_Right(FWD_R - m_correction, FORWARD);
		} while (dif_r < STOP_CONDITION && dif_l < STOP_CONDITION);
	}

	else if (dif_lf > LEFT_THRESHOLD && dif_rf <= RIGHT_THRESHOLD) { //just left wall
		do { //both left and right wall
			Get_IR(FALSE, FALSE, FALSE, FALSE);
			m_correction = (dif_lf - offsets[0])/30;
			Set_Left(FWD_L + m_correction, FORWARD);
			Set_Right(FWD_R - m_correction, FORWARD);
		} while (dif_r < STOP_CONDITION && dif_l < STOP_CONDITION);
	}

	else if (dif_lf <= LEFT_THRESHOLD && dif_rf > RIGHT_THRESHOLD) { //just right wall
		do { //both left and right wall
			Get_IR(FALSE, FALSE, FALSE, FALSE);
			m_correction = (offsets[1] - dif_rf)/30;
			Set_Left(FWD_L + m_correction, FORWARD);
			Set_Right(FWD_R - m_correction, FORWARD);
		} while (dif_r < STOP_CONDITION && dif_l < STOP_CONDITION);
	}

	if (abs(dif_l - dif_r) > EQUAL_VAL) {
		if (dif_r > dif_l) {
			Set_Left(FWD_L, FORWARD); //100
			Set_Right(FWD_R, BACKWARD); //140
		}

		else { //Turn to the left if the left side is greater
			Set_Left(FWD_L, FORWARD);
			Set_Right(FWD_R, BACKWARD);
		}

		do { //should be perfectly facing the back wall
			Get_IR(FALSE, FALSE, FALSE, TRUE);
		} while (abs(dif_l  - dif_r ) > EQUAL_VAL);

		Set_Left(0, FORWARD);
		Set_Right(0, FORWARD);

		HAL_Delay(250);
	}
}

else if (dif_l < FRONT_THRESHOLD + 400 || dif_r < FRONT_THRESHOLD + 400) { //no front wall

	Reset_Counters();
	do {
		Update_Sensors(BARE);
		m_correction = (dif_lf - (dif_rf + WALL_OFFSET))/30;
		Set_Left(FWD_L + m_correction, FORWARD);
		Set_Right(FWD_R - m_correction, FORWARD);

	} while (l_count < 125 && r_count < 125);
/*
	Get_IR(FALSE, FALSE, FALSE, FALSE);

	if (dif_lf > LEFT_CORRECTION) { //turn right
		Set_Left(FWD_L, FORWARD);
		Set_Right(FWD_R, BACKWARD);

	}

	else if (dif_lf < LEFT_CORRECTION && dif_lf > LEFT_THRESHOLD) {
		Set_Left(FWD_L, BACKWARD);
		Set_Right(FWD_R, FORWARD);

	}

	else if (dif_rf > RIGHT_CORRECTION) {
		Set_Left(FWD_L, BACKWARD);
		Set_Right(FWD_R, FORWARD);

	}
	else if (dif_rf < RIGHT_CORRECTION && dif_rf > RIGHT_THRESHOLD) {
		Set_Left(FWD_L, FORWARD);
		Set_Right(FWD_R, BACKWARD);

	}

	while (abs(dif_rf - 1000) > 100 && abs(dif_lf - 1000) > 100) {
	Get_IR(FALSE, FALSE, FALSE, FALSE);
	}
	*/
}

Set_Left(0, FORWARD); //pause to settle weight
Set_Right(0, FORWARD);

HAL_Delay(200);

//Turn to the right if right side is greater
Get_IR(FALSE, FALSE, FALSE, TRUE);

Reset_Counters();

//rotate left do 180 degree turn
Set_Left(FWD_L, BACKWARD);
Set_Right(FWD_R, FORWARD);

int p_correction = 0;

do {
	Update_Sensors(BARE);
	r_count = __HAL_TIM_GET_COUNTER(&htim4);
	l_count = __HAL_TIM_GET_COUNTER(&htim1);
	p_correction = (r_count + (int) (l_count - 65536))/5;
	if (abs(p_correction) < 135) {
	Set_Left(100 + p_correction, BACKWARD);
	Set_Right(110 - p_correction, FORWARD);
	}
} while (r_count < PIVOT_ENC);// || abs(dif_lf - dif_rf) > 120);

//pause after rotating
Set_Left(0, FORWARD);
Set_Right(0, FORWARD);
Reset_Counters();

//go backward a tad
int temp  = 0;
if (done_flag == TRUE) {
	Set_Left(FWD_L, BACKWARD);
	Set_Right(FWD_R, BACKWARD);
	do {
		temp++;
		if (temp > 4000000) {
			temp = 0;
		}
		r_count = __HAL_TIM_GET_COUNTER(&htim4);
	} while ((r_count - 65506) > 0); //65536
}
Set_Left(0, FORWARD);
Set_Right(0, FORWARD);

#if MOUSE_REV == 1

Reset_Counters();
HAL_Delay(200);

Set_Left(FWD_L, BACKWARD);
Set_Right(FWD_R + 10, BACKWARD);

//go backward a tad
if (done_flag == TRUE) {
	do {
		r_count = __HAL_TIM_GET_COUNTER(&htim4);
	} while (abs(r_count - 65386) > 0); //65536
}

else {
do {
	r_count = __HAL_TIM_GET_COUNTER(&htim4);
} while (abs(r_count - 65476) > 0); //65536
}

Set_Left(0, FORWARD);
Set_Right(0, FORWARD);
#endif

HAL_Delay(100);

}

void Program_Walls_Flash() {

Transmit("Programming Walls in Flash.....\r\n");
HAL_FLASH_Unlock();

EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
EraseInitStruct.Sector        = FLASH_SECTOR_5;
EraseInitStruct.NbSectors     = 1;

if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
  {
    while (1)
    {
      Transmit("Programming Error! \r\n");
      HAL_Delay(100);
    }
  }

Address = WALLS_ADDR;

for (int i = 0; i < X_MAZE_SIZE; i++) {
	for (int j = 0; j < Y_MAZE_SIZE - 1; j++) {

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, horiz_walls[i][j]) == HAL_OK) {
			Address = Address + 4;
		}
		else {
			HAL_state = FUCKEDUP;
			Error_Handler();
		}
	}
}

for (int i = 0; i < X_MAZE_SIZE - 1; i++) {
	for (int j = 0; j < Y_MAZE_SIZE; j++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, vert_walls[i][j]) == HAL_OK) {
			Address = Address + 4;
		}
		else {
			HAL_state = FUCKEDUP;
			Error_Handler();
		}
	}
}

for (int i = 0; i < X_MAZE_SIZE; i++) {
	for (int j = 0; j < Y_MAZE_SIZE; j++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, visited_squares[i][j]) == HAL_OK) {
			Address = Address + 4;
		}
		else {
			HAL_state = FUCKEDUP;
			Error_Handler();
		}
	}
}

Transmit("Done! \r\n");
HAL_FLASH_Lock();
}

void Read_Walls_Flash() {


Transmit("Reading Walls from Flash...... \r\n");
Address = WALLS_ADDR;

for (int i = 0; i < X_MAZE_SIZE; i++) {
	for (int j = 0; j < Y_MAZE_SIZE - 1; j++) {
		horiz_walls[i][j] = *(__IO uint32_t *)Address;
		Address = Address + 4;
	}
}

for (int i = 0; i < X_MAZE_SIZE - 1; i++) {
	for (int j = 0; j < Y_MAZE_SIZE; j++) {
		vert_walls[i][j] = *(__IO uint32_t *)Address; //type conversion
		Address = Address + 4;
	}
}

for (int i = 0; i < X_MAZE_SIZE; i++) {
	for (int j = 0; j < Y_MAZE_SIZE; j++) {
		visited_squares[i][j] = *(__IO uint32_t *)Address; //type conversion
		Address = Address + 4;
	}
}

Transmit("Done! \r\n");
}

void Program_Settings() {

	Transmit("Programming Settings in Flash.....\r\n");
	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_6;
	EraseInitStruct.NbSectors     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
	    while (1)
	    {
	      Transmit("Programming Error! \r\n");
	      HAL_Delay(100);
	    }
	  }

	Address = SETTINGS_ADDR;

	for (int i = 0; i < 3; i++) {
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, offsets[i]) == HAL_OK) {
		Address = Address + 4;
	}
	else {
		HAL_state = FUCKEDUP;
		Error_Handler();
	}
	}

	Transmit("Done! \r\n");
	HAL_FLASH_Lock();

}

void Read_Settings() {

/*
sprintf(tx_buffer, "LEFT CORRECTION BEFORE: %d \r\n", offsets[0]);
Transmit(tx_buffer);
sprintf(tx_buffer, "RIGHT CORRECTION BEFORE: %d \r\n", offsets[1]);
Transmit(tx_buffer);
sprintf(tx_buffer, "WALL OFFSETS BEFORE: %d \r\n", offsets[2]);
Transmit(tx_buffer);
*/

Transmit("Reading Settings from Flash...... \r\n");
Address = SETTINGS_ADDR;

for (int i = 0; i < 3; i++) {
	offsets[i] = *(__IO int *)Address;
	Address = Address + 4;

}

if (abs(offsets[0]) > 2000 || abs(offsets[0]) < 100) { //resest

	offsets[0] = LEFT_CORRECTION;
	offsets[1] = RIGHT_CORRECTION;
	offsets[2] = WALL_OFFSET;
}

/*
sprintf(tx_buffer, "LEFT CORRECTION AFTER: %d \r\n", offsets[0]);
Transmit(tx_buffer);
sprintf(tx_buffer, "RIGHT CORRECTION AFTER: %d \r\n", offsets[1]);
Transmit(tx_buffer);
sprintf(tx_buffer, "WALL OFFSETS AFTER: %d \r\n", offsets[2]);
Transmit(tx_buffer);
*/
}

void Reset_Counters() {

__HAL_TIM_SET_COUNTER(&htim1, 0); //reset counters --left encoder
__HAL_TIM_SET_COUNTER(&htim4, 0); //right encdoer
__HAL_TIM_SET_COUNTER(&htim5, 0); //time base

temp_l = 0;
temp_r = 0;
prev_time_count = 0;

prev_l_count = 0;
prev_r_count = 0;
lenc_diff = 0;
renc_diff = 0;

l_count = 0;
r_count = 0;
l_count_corr = 0;
r_count_corr = 0;

lenc_diff_corr = 0;
renc_diff_corr = 0;

}

void Start_IR() {

adc_conv = FALSE;
if(HAL_ADC_Start_DMA(&hadc1, ADC_valbuffer, ADC_VAL_BUFFER_LENGTH) != HAL_OK)
  {
     Error_Handler();
  }

}

void Stop_IR() {

adc_conv = TRUE;
if(HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
  {
     Error_Handler();
  }

}

void Get_IR(int front_save, int side_save, int front_disable, int side_disable) { //front and side variables used to save values in a buffer

	//sensor readings
	int on_l = 0; //with emitter on
	int on_r = 0;
	int on_rf = 0;
	int on_lf = 0;

	int off_l = 0; //with emitter off. detect ambient light
	int off_r = 0;
	int off_rf = 0;
	int off_lf = 0;

	int comp_lf = 0;
	int comp_rf = 0;

	if (front_disable == FALSE) {
		//left sensor
		Start_IR();
		while (adc_conv == FALSE);
		off_l = l;
		HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, ON);
		Start_IR();
		while (adc_conv == FALSE);
		on_l = l;
		HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);
	}

	//right front
	if (side_disable == FALSE) {
		Start_IR();
		while (adc_conv == FALSE);
		off_rf = rf;
		HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, ON);
		Start_IR();
		while (adc_conv == FALSE);
		on_rf = rf;
		HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);
	}

	//left front
	if (side_disable == FALSE) {
		Start_IR();
		while (adc_conv == FALSE);
		off_lf = lf;
		HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, ON);
		Start_IR();
		while (adc_conv == FALSE);
		on_lf = lf;
		HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);
	}

	//right sensor
	if (front_disable == FALSE) {
		Start_IR();
		while (adc_conv == FALSE);
		off_r = r;
		HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, ON);
		Start_IR();
		while (adc_conv == FALSE);
		on_r = r;
		HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
	}

	dif_l = on_l - off_l; //val_array[0]
	dif_r = on_r - off_r; //val_array[1]
	dif_rf = on_rf - off_rf; //val_array[2]
	dif_lf = on_lf - off_lf; //val_array[3]

	//lf and rf transitions
	if (side_save == TRUE) {
		lf_buffer[buff_count] = dif_lf;
		rf_buffer[buff_count] = dif_rf;

		comp_lf = lf_buffer[((buff_count - IR_DIFF) + IR_BUFFER) % IR_BUFFER];
		comp_rf = rf_buffer[((buff_count - IR_DIFF) + IR_BUFFER) % IR_BUFFER];

		if (comp_lf > 10 && abs(dif_lf  - comp_lf) > L_IR_CHANGE) { //send flag for left transition detected
			l_transition_flag = TRUE;
		}
		//else {
		//	l_transition_flag = FALSE;
		//}
		if (comp_rf > 10 && abs(dif_rf - comp_rf) > R_IR_CHANGE) { //send flag for right transition detected
			r_transition_flag = TRUE;
		}
		//else {
		//	r_transition_flag = FALSE;
		//}

		buff_count = (buff_count+1) % IR_BUFFER;  //increase buffer count
	}

	/*if (front_save == TRUE) { //used only for calibration

		l_buffer[buff_count] = dif_l;
		r_buffer[buff_count] = dif_r;
		buff_count = (buff_count + 1) % IR_BUFFER;
	}
	*/

}

void Clear_Buffers() {
	for (int i = 0; i < IR_BUFFER; i++) {
		lf_buffer[i] = 0;
		rf_buffer[i] = 0;
		r_buffer[i] = 0;
		l_buffer[i] = 0;
	}
	buff_count = 0;
}

void Reset_Flags() {

	if (reverse_flag == FALSE) {
		r_turnflag = FALSE; //reset turn flags
		l_turnflag = FALSE;
		dead_flag = ARRIVE;
		fwd_flag = FALSE;
		l_transition_flag = FALSE;
		r_transition_flag = FALSE;
		transition_flag = FALSE;
		done_flag = FALSE;

		search_flag = FALSE;
		stop_flag = FALSE;

		cur_dir = START_DIR;
		cur_move = FWD; //reset to default direction
		next_move = FWD;
		x_coord = X_START;
		y_coord = Y_START;
		prevx = X_START;
		prevy = Y_START;
	}
	else
	{
		r_turnflag = FALSE; //reset turn flags
		l_turnflag = FALSE;
		dead_flag = ARRIVE;
		fwd_flag = FALSE;
		l_transition_flag = FALSE;
		r_transition_flag = FALSE;
		transition_flag = FALSE;
		stop_flag = FALSE;
		done_flag = FALSE;
		cur_move = FWD; //reset to default direction
		next_move = FWD;
	}
}

void Set_Left(int speed, int direction) {

	//when switching directions, PWM polarity switches
	if (direction == FORWARD) {
		speed = 665 - speed;
	}

	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	TIM_OC_InitTypeDef tim2config;

	tim2config.Pulse = speed;
	tim2config.OCMode = TIM_OCMODE_PWM1;
	tim2config.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim2config.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &tim2config, TIM_CHANNEL_1) != HAL_OK)
	  {
	     Error_Handler();
	  }

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, LDIC_Pin, direction);

}

//when switching directions, PWM polarity switches
void Set_Right(int speed, int direction) {

#if MOUSE_REV == 69
	if (direction == FORWARD) {
			speed = 665 - speed;
	}
#else
	if (direction == FORWARD) {
				speed = 665 - speed;
		}
#endif

	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	TIM_OC_InitTypeDef tim2config;
	tim2config.Pulse = speed;
	tim2config.OCMode = TIM_OCMODE_PWM1;
	tim2config.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim2config.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &tim2config, TIM_CHANNEL_3) != HAL_OK)
	{
	   Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

#if MOUSE_REV == 69
	HAL_GPIO_WritePin(GPIOA, RDIC_Pin, direction);
#else
	HAL_GPIO_WritePin(GPIOA, RDIC_Pin, !direction);
#endif

}

void Send_Debug(void) {

	sprintf(tx_buffer, "L Value: %d  LF Value: %d \r\nRF Value: %d R Value: %d \r\n--------------------- \r\n", dif_l, dif_lf, dif_rf, dif_r); //lf, rf, r);
	Transmit(tx_buffer); //transmitm the message above
	sprintf(tx_buffer, "Left Count Value: %d \r\nRight Count Value %d \r\n-----------------\r\n", l_count, r_count);
	Transmit(tx_buffer); //transmit the message above
	sprintf(tx_buffer, "Prev_L: %d \r\nPrev R %d \r\n-----------------\r\n", prev_l_count, prev_r_count);
	Transmit(tx_buffer); //transmit the message above
	sprintf(tx_buffer, "cur_move: %d \r\nnext_move: %d\r\n-----------------\r\n", cur_move, next_move);
	Transmit(tx_buffer);
	sprintf(tx_buffer, "Current Direction: %d \r\n--------------------- \r\n", cur_dir);
	Transmit(tx_buffer);
	sprintf(tx_buffer, "X COORD: %d    Y COORD: %d \r\n--------------------- \r\n", x_coord, y_coord);
	Transmit(tx_buffer);
	//sprintf(tx_buffer, "RIGHT Turn Flag: %d \r\nLEFT Turn Flag: %d \r\n-----------------------", r_turnflag, l_turnflag);
}

void Send_Buffers() {

sprintf(tx_buffer, "Buff Counter: %d \r\n", buff_count);
Transmit(tx_buffer);

Transmit("Left Values: \r\n");
for (int i = 0; i<IR_BUFFER; i++) {

	if (i == buff_count) {

		Transmit(" ||START|| ");
	}
	sprintf(tx_buffer, "%d ", lf_buffer[i]);
	Transmit(tx_buffer);
}

Transmit("\r\n");
Transmit("Right Values: \r\n");

for (int i = 0; i < IR_BUFFER; i++) {

	if (i == buff_count) {

			Transmit(" ||START|| ");
		}
	sprintf(tx_buffer, "%d ", rf_buffer[i]);
	Transmit(tx_buffer);

}

Transmit("\r\n");
}

//takes char array
void Transmit(char message[]) {

	int len;
	len=strlen(message);
	HAL_UART_Transmit(&huart1, message, len, 1000);
}

/*
void Readline(void) {

	int len = strlen(rx_buffer);
	HAL_UART_Receive(&huart1, rx_buffer, len, 5000);
	Transmit("HAHA");
}
*/

void Stop(void) {

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD); //STOP
	Reset_Counters();
	debug_flag = FALSE;

	sprintf(tx_buffer, "Stopping...... Stop Flag:  %d\r\n", stop_flag);
	Transmit(tx_buffer);
	//Send_State();
	//Send_Debug();
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ON);
	//Print_Maze();
	stop_flag = TRUE;
}

void Send_State(void) {

	int i = 0;
	//for(int i = 0; i < DBG_BUFFER; i++)
    while (l_debug[i] != 0 && i < dbg_count)
	{
	sprintf(tx_buffer, "Decision %d: \r\nL: %d   \r\nR: %d   \r\nRF: %d    \r\nLF: %d   \r\nCur Direc: %d \r\nNext Direction: %d  \r\nL Count: %d   R Count: %d \r\nPrev L: %d    Prev R: %d  \r\n---------------------\r\n", (i+ 1), l_debug[i], r_debug[i], rf_debug[i], lf_debug[i], cur_debug[i], turn_debug[i], l_count_debug[i], r_count_debug[i], prev_l_debug[i], prev_r_debug[i]);
	Transmit(tx_buffer);
	i++;
	}
	dbg_count = 0;
}

void Save_State(void) {

	l_debug[dbg_count] = front_l;
	r_debug[dbg_count] = front_r;
	rf_debug[dbg_count] = rf_side;
	lf_debug[dbg_count] = lf_side;
	cur_debug[dbg_count] = cur_move;
	turn_debug[dbg_count] = next_move;
	l_count_debug[dbg_count] = l_count;
	r_count_debug[dbg_count] = r_count;
	prev_l_debug[dbg_count] = prev_l_count;
	prev_r_debug[dbg_count] = prev_r_count;

	dbg_count = (dbg_count+1) % DBG_BUFFER;
}

//updates the buffer. Read from global buffer
void Read_Gyro(void) {

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_RESET); //assert CS pin
		if(HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 4) != HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_SET);
		}
	gyro_reading = *aRxBuffer;
	sprintf(tx_buffer, "Gyro Reading: %X \r\n", gyro_reading);
	Transmit(tx_buffer);
}

void Print_Maze() {

sprintf(tx_buffer, "Floodfill Values: \r\n");
Transmit(tx_buffer);
for (int i = 0; i < Y_MAZE_SIZE; i++) { //floodfill values
	for (int j = 0; j < X_MAZE_SIZE; j++) {  //go row by row, that's why y is looped first
		sprintf(tx_buffer, "%d ", maze[j][i]);
		Transmit(tx_buffer);
	}
	sprintf(tx_buffer, "\r\n \r\n");
	Transmit(tx_buffer);
}

sprintf(tx_buffer, "Horizontal Walls: \r\n");
Transmit(tx_buffer);
for (int i = 0; i < Y_MAZE_SIZE - 1; i++) { //horizontal walls
	for (int j = 0; j < X_MAZE_SIZE; j++) {
		sprintf(tx_buffer, "%d ", horiz_walls[j][i]);
		Transmit(tx_buffer);
	}
	sprintf(tx_buffer, "\r\n \r\n");
	Transmit(tx_buffer);
}

sprintf(tx_buffer, "Vertical Walls: \r\n");
Transmit(tx_buffer);
for (int i = 0; i < Y_MAZE_SIZE; i++) { //vertical walls
	for (int j = 0; j < X_MAZE_SIZE - 1; j++) {
		sprintf(tx_buffer, "%d ", vert_walls[j][i]);
		Transmit(tx_buffer);
	}
	sprintf(tx_buffer, "\r\n \r\n");
	Transmit(tx_buffer);
}
/*
Calc_Optimal(); //generate optimal path
Transmit("Optimal Path: \r\n");
*/

return;
}

int Get_Lowest_Square(int x, int y) { //gets lowest square

int values[5] = {127, 127, 127, 127, 127}; //first value is default 127. Then it goes: up, down, left, right
int min = 0; //make sure there is default value

if (y > 0 && horiz_walls[x][y-1] != TRUE) { //if row is greater than zero
	values[1] = maze[x][y - 1]; //up square
}

if (y < Y_MAZE_SIZE - 1 && horiz_walls[x][y] != TRUE) {
	values[2] = maze[x][y + 1]; //down square
}

if (x > 0 && vert_walls[x - 1][y] != TRUE) {
	values[3] = maze[x - 1][y]; //left square
}

if (x < X_MAZE_SIZE - 1 && vert_walls[x][y] != TRUE) {
	values[4] = maze[x + 1][y]; //right square
}

for (int i = 1; i < 5; i++) { //get the index of the lowest square. Should be 127 by default
if(values[i] < values[min]) {
	min = i;
}
}

return values[min];
}

void Floodfill(int reverse, int path_search, int full) { //reverse chooses to set the start or the end as the target. path_search sets the next coordinate on the list as the target


	int x_buffer[100]; //buffer to store coordinates that need to be updated
	int y_buffer[100];

	int temp_x = 0;
	int temp_y = 0;

	int buffer_counter = 0; //counter to loop through x_buffer and y_buffer
	int pathdist = 1; //floodfill values

	for (int i = 0; i < X_MAZE_SIZE; i++) { //initialize all values to 127
		for (int j = 0; j < Y_MAZE_SIZE; j++) {
			maze[i][j] = 127;
		}
	}

	if (path_search == TRUE) {
		maze[search_x][search_y] = 0;
	}

	else {
		if (reverse == TRUE) {
			maze[X_START][Y_START] = 0;
		}

		else
		{
			maze[X_FINAL][Y_FINAL] = 0;
#if X_MAZE_SIZE == 16
			maze[X_FINAL+1][Y_FINAL] = 0;
			maze[X_FINAL][Y_FINAL+1] = 0;
			maze[X_FINAL+1][Y_FINAL+1] = 0;
#endif
		}
	}

	while(1) {

		buffer_counter = 0;

		for (int i = 0; i < X_MAZE_SIZE; i++) { //loop through all values to see if square has been reached
			for (int j = 0; j < Y_MAZE_SIZE; j++) {
				if (maze[i][j] != 127) {
					continue; //if cell has already been updated skip it
				}
				if (Get_Lowest_Square(i, j) != 127) { //if reached, add to buffer
					x_buffer[buffer_counter] = i; //x_ coordinate buffer
					y_buffer[buffer_counter] = j; //y coordinate buffer
					buffer_counter++;
				}
			}
		}

		for (int i = 0; i < buffer_counter; i++) { //write all buffer coordinates with the path distance
			temp_x = x_buffer[i];
			temp_y = y_buffer[i];
			maze[temp_x][temp_y] = pathdist;
		}

		if (full == FALSE) {
			if (maze[x_coord][y_coord] != 127) { //if the algorithm has reached the desired position
				break;
			}
		}
		else {

			if (maze[X_START][Y_START] != 127) {
				break;
			}
		}


		pathdist++;

		if (pathdist > 200) { //prevent infinite loops
			stop_flag = TRUE;
#if DEBUG == TRUE
			sprintf(tx_buffer, "Floodfill Error pathdist: %d \r\n", pathdist);
			Print_Maze();
			Transmit(tx_buffer);
#endif
			break;
		}

	}
//end algorithm while
}

void Reset_Maze() { ///resets the entire maze to blank, and floodfills the value

	for (int i = 0; i < X_MAZE_SIZE; i++) { //horizontal walls
		for (int j = 0; j < Y_MAZE_SIZE - 1; j++) {
			horiz_walls[i][j] = 0;
		}
	}

	for (int i = 0; i < X_MAZE_SIZE - 1; i++) { //vertical walls
		for (int j = 0; j < Y_MAZE_SIZE; j++) {
			vert_walls[i][j] = 0;
		}
	}

	for (int i = 0; i < X_MAZE_SIZE; i++) { //set all squares to be unvisited
		for (int j = 0; j < Y_MAZE_SIZE; j++) {
			visited_squares[i][j] = FALSE;
		}
	}
}

void Read_Walls() {

#define DIFFERENTIAL 1000

if (visited_squares[x_coord][y_coord] == TRUE) {
	return;
}

front_l = 0;
front_r = 0;
lf_side = 0;
rf_side = 0;

for (int i = 0; i < WALL_SAMPLES; i++) {
	Get_IR(FALSE, FALSE, FALSE, FALSE);
	front_l = front_l + dif_l;
	front_r = front_r + dif_r;
	lf_side = lf_side + dif_lf;
	rf_side = rf_side + dif_rf;
}

front_l = front_l/WALL_SAMPLES;
front_r = front_r/WALL_SAMPLES;
lf_side = lf_side/WALL_SAMPLES;
rf_side = rf_side/WALL_SAMPLES;

switch(cur_dir) {

case NORTH: //facing up
	if (y_coord > 0 && (front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && abs(front_l - front_r) < DIFFERENTIAL) { //front sensor
		horiz_walls[x_coord][y_coord - 1] = 1; //up wall
	}
	if (x_coord < X_MAZE_SIZE - 1 && rf_side >= RIGHT_THRESHOLD) { //right sensor
		vert_walls[x_coord][y_coord] = 1; //right wall
	}
	if (x_coord > 0 && lf_side >= LEFT_THRESHOLD) {  //left sensor
		vert_walls[x_coord - 1][y_coord] = 1; //left wall
	}
	break;

case SOUTH: //facing down
	if (y_coord < Y_MAZE_SIZE - 1 && (front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && abs(front_l - front_r) < DIFFERENTIAL) {  //front sensor
		horiz_walls[x_coord][y_coord] = 1; //down_wall
		}
	if (x_coord > 0 && rf_side >= RIGHT_THRESHOLD) {  //right sensor
		vert_walls[x_coord - 1][y_coord] = 1; //left wall
	}
	if (x_coord < X_MAZE_SIZE - 1 && lf_side >= LEFT_THRESHOLD) { //left sensor
		vert_walls[x_coord][y_coord] = 1; //right wall
	}
	break;

case WEST: //facing left
	if (x_coord > 0 && (front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && abs(front_l - front_r) < DIFFERENTIAL) { //front sensor
		vert_walls[x_coord - 1][y_coord] = 1; //left wall
	}
	if (y_coord > 0 && rf_side >= RIGHT_THRESHOLD) {  //right sensor
		horiz_walls[x_coord][y_coord - 1] = 1; //up wall
	}
	if (y_coord < Y_MAZE_SIZE - 1 && lf_side >= LEFT_THRESHOLD) {//left sensor
		horiz_walls[x_coord][y_coord] = 1; //down wall
	}
	break;

case EAST: //facing right
	if (x_coord < X_MAZE_SIZE - 1 && (front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && abs(front_l - front_r) < DIFFERENTIAL) { //front sensor
		vert_walls[x_coord][y_coord] = 1; //right wall
		}
	if (y_coord < Y_MAZE_SIZE - 1 && rf_side >= RIGHT_THRESHOLD) { //right sensor
		horiz_walls[x_coord][y_coord] = 1; //down wall
	}
	if (y_coord > 0 && lf_side >= LEFT_THRESHOLD) {  //left sensor
		horiz_walls[x_coord][y_coord - 1] = 1; //up wall
	}
	break;

}
}

void Switch_Direction() {

	switch (cur_dir) {

	case NORTH: cur_dir = SOUTH; break; //update the direction that the mouse is pointing
	case SOUTH: cur_dir = NORTH; break;
	case WEST: cur_dir = EAST; break;
	case EAST: cur_dir = WEST; break;
	}
}

void Update_Position_New() {

	visited_squares[x_coord][y_coord] = TRUE;

	if (next_move != DEAD) {
		prevx = x_coord; //save previous values. Used for dead end
		prevy = y_coord;
	}

	switch (cur_dir) {
	case NORTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case RIGHT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case DEAD:
			cur_dir = SOUTH;
			break;
		}
	break;

	case SOUTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case RIGHT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case DEAD:
			cur_dir = NORTH;
			break;
		}
	break; //bitch retard enema asshole kockface
		case WEST:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			break;
		case LEFT:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			cur_dir = SOUTH;
			break;
		case RIGHT:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			cur_dir = NORTH;
			break;
		case DEAD:
			cur_dir = EAST;
			break;
		}
		break;

		case EAST:
			switch(next_move) {
			case FWD:
				x_coord = x_coord + EAST_X;
				y_coord = y_coord + EAST_Y;
				cur_dir = EAST;
				break;
			case LEFT:
				x_coord = x_coord + NORTH_X;
				y_coord = y_coord + NORTH_Y;
				cur_dir = NORTH;
				break;
			case RIGHT:
				x_coord = x_coord + SOUTH_X;
				y_coord = y_coord + SOUTH_Y;
				cur_dir = SOUTH;
				break;
			case DEAD:
				cur_dir = WEST;
				break;
			}
			break;
		}

		if (next_move == DEAD) {
			sprintf(tx_buffer, "|||DEAD END||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
			Transmit(tx_buffer);
			x_coord = prevx;
			y_coord = prevy;

		}

		if (x_coord < 0 || y_coord < 0 || x_coord >= X_MAZE_SIZE || y_coord >= Y_MAZE_SIZE) {
			stop_flag = TRUE;
			sprintf(tx_buffer, "Position out of Boundary! X VALUE: %d  Y VALUE %d  \r\n", x_coord, y_coord);
			Transmit(tx_buffer);
		}

		sprintf(tx_buffer, "|||DECISION  %d||| X VALUE: %d  Y VALUE: %d  DIRECTION: %d  NEXT: %d \r\n", dbg_count + 1, x_coord, y_coord, cur_dir, next_move);
		Transmit(tx_buffer);

}

void Update_Position() { //updates position and direction so read walls is good

	visited_squares[x_coord][y_coord] = TRUE;

	if (next_move != DEAD) {
		prevx = x_coord; //save previous values. Used for dead end
		prevy = y_coord;
	}

	switch (cur_dir) {
	case NORTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case RIGHT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case DEAD:
			cur_dir = SOUTH;
			break;
		}
		break;

	case SOUTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case RIGHT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case DEAD:
			cur_dir = NORTH;
			break;
		}
		break; //bitch retard enema asshole kockface

	case WEST:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			break;
		case LEFT:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			cur_dir = SOUTH;
			break;
		case RIGHT:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			cur_dir = NORTH;
			break;
		case DEAD:
			cur_dir = EAST;
			break;
		}
		break;

	case EAST:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case LEFT:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			cur_dir = NORTH;
			break;
		case RIGHT:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			cur_dir = SOUTH;
			break;
		case DEAD:
			cur_dir = WEST;
			break;
		}
		break;
	}

	if (next_move == DEAD) {
#if DEBUG == TRUE
		sprintf(tx_buffer, "|||DEAD END||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
		Transmit(tx_buffer);
#endif
		x_coord = prevx;
		y_coord = prevy;
		return;
	}

	if (maze[x_coord][y_coord] == 0) { //reached target square
		if (search_flag == FALSE) { //reached center
			Switch_Direction();
			done_flag = TRUE; //tell loop to stop
		}
		/*
		else { //looking along for the optimal path still

			visited_squares[x_coord][y_coord] = TRUE;
			Calc_Optimal(); //recalculate optimal path
			Get_Coordinate(); //get any coordinates that haven't been seen yet
			if (coordinate_count == 0) { //if none, go back to start
				sprintf(tx_buffer, "All Coordinates Found! \r\n");
				Transmit(tx_buffer);
				search_flag = FALSE;
			}
			Floodfill(reverse_flag, search_flag, FALSE); // go back to center
		}
		*/
#if DEBUG == TRUE
		sprintf(tx_buffer, "|||FINAL||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
		Transmit(tx_buffer);
#endif
		//visited_squares[x_coord][y_coord] = TRUE;
		return;
	}

	else if (x_coord < 0 || y_coord < 0 || x_coord >= X_MAZE_SIZE || y_coord >= Y_MAZE_SIZE) {
		stop_flag = TRUE;
#if DEBUG == TRUE
		sprintf(tx_buffer, "Position out of Boundary! X VALUE: %d  Y VALUE %d  \r\n", x_coord, y_coord);
		Transmit(tx_buffer);
#endif
	}
#if DEBUG == TRUE
	sprintf(tx_buffer, "|||DECISION  %d||| X VALUE: %d  Y VALUE: %d  DIRECTION: %d  NEXT: %d \r\n", dbg_count + 1, x_coord, y_coord, cur_dir, next_move);
	Transmit(tx_buffer);
#endif

}

void SetSpeed(int left_motor, int right_motor) {


}

void Measure_Speed() {

	int debug_count = 0;
	uint32_t left_speed = 0;
	uint32_t right_speed = 0;
	int left_acceleration = 0;
	int right_acceleration = 0;
	static uint32_t prev_left_speed = 0;
	static uint32_t prev_right_speed = 0;


	Reset_Counters();
	HAL_TIM_Base_Start(&htim5);
	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);

	while (1) {

		Update_Sensors(TIME);
		if (l_count > F_ENC2*4  && r_count > F_ENC2*4) {
			break;
		}

		if (time_count - prev_time_count > 10000) { //sample every 10 ms

			left_speed = 180*1000000/700*(l_count - temp_l)/(time_count - prev_time_count); // in mm/s
			right_speed = 180*1000000/700*(r_count - temp_r)/(time_count - prev_time_count); // in mm/s
			left_acceleration =  1000000*((int) left_speed - prev_left_speed)/(time_count - prev_time_count);
			right_acceleration = 1000000*((int) right_speed - prev_right_speed)/(time_count - prev_time_count);

#if DEBUG == TRUE
			if (debug_count > 5) {

				sprintf(tx_buffer, "Left Acceleration: %d mm/s/s  Right Acceleration: %d mm/s/s \r\n", left_acceleration, right_acceleration);
				Transmit(tx_buffer);
				sprintf(tx_buffer, "Left Speed %u mm/s   Right Speed %u mm/s \r\n", left_speed, right_speed);
				Transmit(tx_buffer);
				sprintf(tx_buffer, "Prev Left Speed %u mm/s  Prev Right Speed %u mm/s \r\n-------------------\r\n", prev_left_speed, prev_right_speed);
				Transmit(tx_buffer);
				//sprintf(tx_buffer, "PREV TIME %d   NOW TIME: %d \r\n", prev_time_count, time_count);
				//Transmit(tx_buffer);
				//sprintf(tx_buffer, "TEMP L %d   TEMP R: %d \r\n", temp_l, temp_r);
				//Transmit(tx_buffer);
				//sprintf(tx_buffer, "L Count %d    R Count: %d \r\n-------------------------- \r\n \r\n", l_count, r_count);
				//Transmit(tx_buffer);
				debug_count = 0;
			}
			debug_count++;
#endif
			prev_time_count = time_count;
			temp_l = l_count;
			temp_r = r_count;
			prev_left_speed = left_speed;
			prev_right_speed = right_speed;
		}
	}
	Stop();
	HAL_TIM_Base_Stop(&htim5);
}

void Test_Clock() {
	HAL_TIM_Base_Start(&htim5);
	int count = 0;
	while(count < 10) {

		time_count = __HAL_TIM_GET_COUNTER(&htim5);
		if (time_count % 1000000 == 0) {
			Transmit("One Second \r\n");
			count++;
		}
	}
	HAL_TIM_Base_Stop(&htim5);
}

void micros(uint32_t microseconds) {

	uint32_t delay = 0;
	uint32_t temp = 0;

	__HAL_TIM_SET_COUNTER(&htim5, 0);
	//HAL_TIM_Base_Start(&htim5);

	while(delay < microseconds) {
		delay = __HAL_TIM_GET_COUNTER(&htim5);
		temp++;
	}
	//HAL_TIM_Base_Stop(&htim5);

}

void Calc_Optimal() {

const int N[2] = {0, -1}; //directions in maze
const int S[2] = {0, 1};
const int W[2] = {-1, 0};
const int E[2] = {1, 0};

int cur_dir = START_DIR; //start direction
int counter = 0; //counter for counting the number of squares

int x = X_START; //initialize cursor to start square
int y = Y_START;

Floodfill(FALSE, FALSE, TRUE);  //floodfill from final value to start value

int val = maze[x][y]; //get start maze value
int next_values[4] = {-1, -1, -1, -1}; //array to hold the next values
int index = 0;

optimal_x[counter] = x;
optimal_y[counter] = y;

while(maze[x][y] != 0) {

	next_values[0] = 127;
	next_values[1] = 127;
	next_values[2] = 127;
	next_values[3] = 127;

	val = maze[x][y];

	if (y > 0 && horiz_walls[x][y-1] != TRUE) { //if row is greater than zero
		next_values[0] = maze[x][y-1]; //up square
	}

	if (y < Y_MAZE_SIZE - 1 && horiz_walls[x][y] != TRUE) {
		next_values[1] = maze[x][y+1]; //down square
	}

	if (x > 0 && vert_walls[x - 1][y] != TRUE) {
		next_values[2] = maze[x - 1][y]; //left square
	}

	if (x < X_MAZE_SIZE - 1 && vert_walls[x][y] != TRUE) {
		next_values[3] = maze[x+1][y]; //right square
	}

	for (int i = 0; i < 4; i++) {
		if (next_values[i] == val - 1) {
			index = i;
		}
	}

	switch (cur_dir) {

	case NORTH:
		switch(index) {
		case(0):
			optimal_path[counter] = FOR;
		break;
		case(1):
		//NO
		break;
		case(2):
			optimal_path[counter] = LEF;
			cur_dir = WEST;
		break;
		case(3):
				optimal_path[counter] = RIGH;
				cur_dir = EAST;
		break;
		}
	break;

	case SOUTH:
		switch(index) {
		case(0):
				//NO
		break;
		case(1):
				optimal_path[counter] = FOR;
		break;
		case(2):
				optimal_path[counter] = RIGH;
				cur_dir = WEST;
		break;
		case(3):
				optimal_path[counter] = LEF;
				cur_dir = EAST;
		break;
		}
	break;

	case WEST:
		switch(index) {
		case(0):
				optimal_path[counter] = RIGH;
				cur_dir = NORTH;
		break;
		case(1):
				optimal_path[counter] = LEF;
				cur_dir = SOUTH;
		break;
		case(2):
				optimal_path[counter] = FOR;
		break;
		case(3):
				//NO
		break;
		}
	break;

	case EAST:
		switch(index) {
		case(0):
				optimal_path[counter] = LEF;
				cur_dir = NORTH;
		break;
		case(1):
				optimal_path[counter] = RIGH;
				cur_dir = SOUTH;
		break;
		case(2):
				//NO
		break;
		case(3):
				optimal_path[counter] = FOR;
		break;
		}
	break;
	}

	switch (cur_dir) { //update the cursor
	case NORTH:
		x = x + N[0];
		y = y + N[1];
	break;
	case SOUTH:
		x = x + S[0];
		y = y + S[1];
	break;
	case WEST:
		x = x + W[0];
		y = y + W[1];
	break;
	case EAST:
		x = x + E[0];
		y = y + E[1];
	break;
	}
counter++;
optimal_x[counter] = x;
optimal_y[counter] = y;

}

optimal_count = counter; //DEBUG
optimal_path[counter] = 's';

#if DEBUG == TRUE
Transmit("Optimal Path: ");
Transmit(optimal_path);
for (int i = 0; i <= optimal_count; i++) {
	sprintf(tx_buffer, "(%d, %d) \r\n", optimal_x[i], optimal_y[i]);
	Transmit(tx_buffer);
}
Transmit("\r\n");
#endif

}

void Mark_Center() {
	visited_squares[X_FINAL][Y_FINAL] = TRUE;
#if X_MAZE_SIZE == 16
	visited_squares[X_FINAL+1][Y_FINAL] = TRUE;
	visited_squares[X_FINAL][Y_FINAL] = TRUE;
	visited_squares[X_FINAL + 1][Y_FINAL + 1] = TRUE;
#endif


}

void Fill_Center() {

	final_x = x_coord;
	final_y = y_coord;
	 #if X_MAZE_SIZE == 16

	switch(final_x) {

	case X_FINAL:
		switch(final_y) {
		case Y_FINAL: //top left
				if (horiz_walls[final_x][final_y - 1] == TRUE) {

					horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE; //above left
					horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE; //above right
					horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE; //below left
					horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE; //below right

					//vert_walls[X_FINAL - 1][Y_FINAL] = TRUE; //top left
					vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE; //bottom left
					vert_walls[X_FINAL + 1][Y_FINAL] = TRUE; //top right
					vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE; //bottom right

				}
				else {
					//horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
					horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
					horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
					horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

					vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
					vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
					vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
					vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;
				}
		break;

		case Y_FINAL + 1: //bottom left
		if (horiz_walls[final_x][final_y] == TRUE) {

			horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

			vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
			//vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;

		}
		else {
			horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
			//horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

			vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;
		}
		break;

	}
	break;

	case X_FINAL + 1:
		switch(final_y) { //top right
		case Y_FINAL:
			if (horiz_walls[final_x][final_y - 1] == TRUE) {

				horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
				horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
				horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
				horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

				vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
				vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
				//vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
				vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;

			}
			else {
				horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
				//horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
				horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
				horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

				vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
				vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
				vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
				vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;
			}
			break;

	case Y_FINAL + 1: //bottom right
		if (horiz_walls[final_x][final_y] == TRUE) {

			horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
			horiz_walls[X_FINAL+1][Y_FINAL + 1] = TRUE;

			vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
			//vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;
		}
		else {
			horiz_walls[X_FINAL][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL + 1][Y_FINAL - 1] = TRUE;
			horiz_walls[X_FINAL][Y_FINAL + 1] = TRUE;
			//horiz_walls[X_FINAL + 1][Y_FINAL + 1] = TRUE;

			vert_walls[X_FINAL - 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL - 1][Y_FINAL + 1] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL] = TRUE;
			vert_walls[X_FINAL + 1][Y_FINAL+ 1] = TRUE;

		}
		break;
		}
	break;
	}
#endif

}

void Get_Coordinate() { //gets next coordinate to visit
	coordinate_count = 0;
	for (int i = optimal_count; i > 0; i--) {

		if (visited_squares[optimal_x[i]][optimal_y[i]] == FALSE) {
			search_x = optimal_x[i];
			search_y = optimal_y[i];
			coordinate_count++;
			//search_flag = TRUE;
#if DEBUG == TRUE
			//sprintf(tx_buffer, "Going to (%d, %d) \r\n", search_x, search_y);
			//Transmit(tx_buffer);
#endif
			return;
		}
	}
	Transmit("All coordinates on optimal path found! \r\n");
}

void Fast_Straights() {

char command; //command to be processed
int counter = 0; //counter to cycle through generated path
int new_count = 0; //counter to cycle through NEW generated path
int fwd_count = 0; //count how many straights there are in a row
int right_count = 0; //count how many right turns there are in a row
int left_count = 0; //count left turns
char cmd_buff = FOR; //command buffer for appending to new path

do {
	command = optimal_path[counter]; //read command
	if (command == FOR) { //forward
		fwd_count++; //increase count

		if (right_count > 0) { //if previous was left or right, then this is the first fwd
			fast_path[new_count] = R90; //previous was right
			right_count = 0;
			new_count++;
		}
		else if (left_count > 0) {
			fast_path[new_count] = L90; //previous was left
			left_count = 0;
			new_count++;
		}
	}
	else {
		if (fwd_count > 0) { //check to see if there was fwd count
			switch(fwd_count) {
			case 1: cmd_buff = FWD1; break;
			case 2:	cmd_buff = FWD2; break;
			case 3: cmd_buff = FWD3; break;
			case 4: cmd_buff = FWD4; break;
			case 5: cmd_buff = FWD5; break;
			case 6:	cmd_buff = FWD6; break;
			case 7:	cmd_buff = FWD7; break;
			case 8: cmd_buff = FWD8; break;
			case 9:	cmd_buff = FWD9; break;
			case 10: cmd_buff = FWD10; break;
			case 11: cmd_buff = FWD11; break;
			case 12: cmd_buff = FWD12; break;
			case 13: cmd_buff = FWD13; break;
			case 14: cmd_buff = FWD14; break;
			case 15: cmd_buff = FWD15; break;
			}
			fast_path[new_count] = cmd_buff;
			fwd_count = 0;
			new_count++;

		}
		if (command == LEF) {
			switch(left_count) {
			case(0):
				left_count++;
				if (right_count > 0) {
					fast_path[new_count] = R90;
					right_count = 0;
					new_count++;
				}
			break;
			case(1):
				fast_path[new_count] = L180;
				left_count = 0;
				new_count++;
				break;
			}
		}
		else if (command == RIGH) {
			switch(right_count) {
			case(0):
				right_count++;
				if (left_count > 0) {

					fast_path[new_count] = L90;
					left_count = 0;
					new_count++;
				}
			break;
			case(1):
					fast_path[new_count] = R180;
					right_count = 0;
					new_count++;
			break;
			}
		}
	fwd_count = 0;
	}
counter++;
} while (command != STOP);


if (right_count > 0) {
	cmd_buff = R90;
	fast_path[new_count] = cmd_buff;
}

else if (left_count > 0) {
	cmd_buff = L90;
	fast_path[new_count] = cmd_buff;
}

else if (fwd_count > 0) {

	switch(fwd_count) {
	case 0: break;
	case 1: cmd_buff = FWD1; break;
	case 2:	cmd_buff = FWD2; break;
	case 3: cmd_buff = FWD3; break;
	case 4: cmd_buff = FWD4; break;
	case 5: cmd_buff = FWD5; break;
	case 6:	cmd_buff = FWD6; break;
	case 7:	cmd_buff = FWD7; break;
	case 8: cmd_buff = FWD8; break;
	case 9:	cmd_buff = FWD9; break;
	case 10: cmd_buff = FWD10; break;
	case 11: cmd_buff = FWD11; break;
	case 12: cmd_buff = FWD12; break;
	case 13: cmd_buff = FWD13; break;
	case 14: cmd_buff = FWD14; break;
	case 15: cmd_buff = FWD15; break;
	fast_path[new_count] = cmd_buff;
	}
}

fast_path[new_count+1] = STOP;

#if DEBUG == TRUE
Transmit("Fast Path: ");
Transmit(fast_path);
Transmit("\r\n");
#endif
}

void Speed_Test() {

	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

	Clear_Buffers();
	Reset_Counters();

	HAL_Delay(1000); //delay before start to get finger out of the way

	int l_speed = 400;
	int r_speed = 400 + l_speed/15;
	Reset_Counters();
	Set_Left(l_speed , FORWARD);
	Set_Right(r_speed, FORWARD);

	do {
		l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
		r_count = __HAL_TIM_GET_COUNTER(&htim4);
		time_count = __HAL_TIM_GET_COUNTER(&htim5);
		lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
		renc_diff = r_count - prev_r_count;
		m_correction = Motor_Correction(TRUE);
		Set_Left(l_speed + m_correction, FORWARD);
		Set_Right(r_speed + m_correction, FORWARD);
		Speed_Profiler(&l_speed, 5);
	} while (lenc_diff < F_ENC2*4 && renc_diff < F_ENC2*4);

	Stop();

}


void Speed_Run(char path[]) {

//take optimal path and convert to fast straightaways
start_flag = TRUE;
speedrunturn = FALSE;
int counter = 0;
char ps =  path[counter];
counter++;
char ns = path[counter];
//l_count and r_count

HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

Clear_Buffers();
Reset_Counters();

HAL_TIM_Base_Start(&htim5); //start counter for speed

HAL_Delay(1000); //delay before start to get finger out of the way

while (ps != STOP) {

switch (ps) {

case L90:
	Left_Spd(1, ns);
	break;
case R90:
	Right_Spd(1, ns);
	break;
case L180:
	Left_Spd(2, ns);
	break;
case R180:
	Right_Spd(2, ns);
	break;
case FWD1:
	Forward_Spd(1, ns, speedrunturn);
	break;
case FWD2:
	Forward_Spd(2, ns, speedrunturn);
	break;
case FWD3:
	Forward_Spd(3, ns, speedrunturn);
	break;
case FWD4:
	Forward_Spd(4, ns, speedrunturn);
	break;
case FWD5:
	Forward_Spd(5, ns, speedrunturn);
	break;
case FWD6:
	Forward_Spd(6, ns, speedrunturn);
	break;
case FWD7:
	Forward_Spd(7, ns, speedrunturn);
	break;
case FWD8:
	Forward_Spd(8, ns, speedrunturn);
	break;
case FWD9:
	Forward_Spd(9, ns, speedrunturn);
	break;
case FWD10:
	Forward_Spd(10, ns, speedrunturn);
	break;
case FWD11:
	Forward_Spd(11, ns, speedrunturn);
	break;
case FWD12:
	Forward_Spd(12, ns, speedrunturn);
	break;
case FWD13:
	Forward_Spd(13, ns, speedrunturn);
	break;
case FWD14:
	Forward_Spd(14, ns, speedrunturn);
	break;
case FWD15:
	Forward_Spd(15, ns, speedrunturn);
	break;
}
counter++;
ps = ns;
ns = path[counter];
start_flag = FALSE;
}

HAL_TIM_Base_Stop(&htim5); //stop counting seconds
Stop();
}


int Get_Next_Dumb() {

int next = 0;

front_l = 0;
front_r = 0;
lf_side = 0;
rf_side = 0;

	for (int i = 0; i < WALL_SAMPLES; i++) {
		Get_IR(FALSE, FALSE, FALSE, FALSE);
		front_l = front_l + dif_l;
		front_r = front_r + dif_r;
		lf_side = lf_side + dif_lf;
		rf_side = rf_side + dif_rf;
	}

	front_l = front_l/WALL_SAMPLES;
	front_r = front_r/WALL_SAMPLES;
	lf_side = lf_side/WALL_SAMPLES;
	rf_side = rf_side/WALL_SAMPLES;

	switch (cur_move) {

		  case FWD:
			  if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && rf_side <= RIGHT_THRESHOLD) //if front and right side is not blocked
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side <= LEFT_THRESHOLD) //if front and right side is blocked, but left is not
			  {next = LEFT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side > LEFT_THRESHOLD && rf_side > RIGHT_THRESHOLD)
			  {next = DEAD;}
			  //the default next_move is the cur_move, so if the front isn't blocked, keep going straight
			  else
			  {next = FWD;}
			  break;

		  case RIGHT:
			  if (front_l <= FRONT_THRESHOLD || front_r <= FRONT_THRESHOLD) //r_turnflag means the second part of the turn
			  {next = FWD;}
			  else if (rf_side >= RIGHT_THRESHOLD && lf_side < LEFT_THRESHOLD)
			  {next = LEFT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side > LEFT_THRESHOLD && rf_side > RIGHT_THRESHOLD)
			  {next = DEAD;}
			  else
			  {next = RIGHT;}
			  break;

		  case LEFT:
			  if (front_l <= FRONT_THRESHOLD || front_r <= FRONT_THRESHOLD)
			  {next = FWD;}
			  else if (lf_side >= LEFT_THRESHOLD && rf_side < RIGHT_THRESHOLD)
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side >= LEFT_THRESHOLD && rf_side >= RIGHT_THRESHOLD)
			  {next = DEAD;}
			  else
			  {next = LEFT;}
			  break;
		  case DEAD:
			  if (front_l < FRONT_THRESHOLD || front_r < FRONT_THRESHOLD)
			  {next = FWD;}
			  if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && rf_side <= RIGHT_THRESHOLD) //if front and right side is not blocked
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side <= LEFT_THRESHOLD) //if front and right side is blocked, but left is not
			  {next = LEFT;}
			  break;
		  }

	if (debug_flag == FALSE) //read walls if not being debugged
	{
	switch(cur_dir) {

	case NORTH: //facing up
		if (y_coord > 0 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		if (x_coord < X_MAZE_SIZE - 1 && rf_side > RIGHT_THRESHOLD) { //right sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
		}
		if (x_coord > 0 && lf_side > LEFT_THRESHOLD) {  //left sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		break;

	case SOUTH: //facing down
		if (y_coord < Y_MAZE_SIZE - 1 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) {  //front sensor
			horiz_walls[x_coord][y_coord] = 1; //down_wall
			}
		if (x_coord > 0 && rf_side > RIGHT_THRESHOLD) {  //right sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		if (x_coord < Y_MAZE_SIZE - 1 && lf_side > LEFT_THRESHOLD) { //left sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
		}
		break;

	case WEST: //facing left
		if (x_coord > 0 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		if (y_coord > 0 && rf_side > RIGHT_THRESHOLD) {  //right sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		if (y_coord < Y_MAZE_SIZE - 1 && lf_side > LEFT_THRESHOLD) {//left sensor
			horiz_walls[x_coord][y_coord] = 1; //down wall
		}
		break;

	case EAST: //facing right
		if (x_coord < X_MAZE_SIZE - 1 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
			}
		if (y_coord < Y_MAZE_SIZE - 1 && rf_side > RIGHT_THRESHOLD) { //right sensor
			horiz_walls[x_coord][y_coord] = 1; //down wall
		}
		if (y_coord > 0 && lf_side > LEFT_THRESHOLD) {  //left sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		break;

	}
}

return next;
}

void Turn_On_Lights() {

	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, ON);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, ON);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ON);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ON);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ON);


}

void Turn_Off_Lights() {

	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, OFF);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, OFF);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, OFF);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, OFF); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

}

int Get_Next_Move() {

 //index of lowest move
int next = FWD;
int min = 0;
int values[5] = {127, 127, 127, 127, 127};

if (y_coord > 0 && horiz_walls[x_coord][y_coord-1] != TRUE) { //if row is greater than zero
	values[1] = maze[x_coord][y_coord-1]; //up square
}

if (y_coord < Y_MAZE_SIZE - 1 && horiz_walls[x_coord][y_coord] != TRUE) {
	values[2] = maze[x_coord][y_coord+1]; //down square
}

if (x_coord > 0 && vert_walls[x_coord - 1][y_coord] != TRUE) {
	values[3] = maze[x_coord - 1][y_coord]; //left square
}

if (x_coord < X_MAZE_SIZE - 1 && vert_walls[x_coord][y_coord] != TRUE) {
	values[4] = maze[x_coord+1][y_coord]; //right square
}

for (int i = 1; i < 5; i++) {
	if(values[i] < values[min]) {
		min = i;
	}
}

if (values[min] >= maze[x_coord][y_coord]) { //if lowest available square is higher than current
	//floodfill and recursive call
	Floodfill(reverse_flag, search_flag, FALSE);
	next = Get_Next_Move();
	//Print_Maze();
	return next;
}

switch(cur_dir) {

case NORTH:
	switch(min) {
	case 1:
		next = FWD;
		break;
	case 2:
		next = DEAD;
		break;
	case 3:
		next = LEFT;
		break;
	case 4:
		next = RIGHT;
		break;
	}
break;

case SOUTH:
	switch(min) {
	case 1:
		next = DEAD;
		break;
	case 2:
		next = FWD;
		break;
	case 3:
		next = RIGHT;
		break;
	case 4:
		next = LEFT;
		break;
	}
break;

case EAST:
	switch(min) {
	case 1:
		next = LEFT;
		break;
	case 2:
		next = RIGHT;
		break;
	case 3:
		next = DEAD;
		break;
	case 4:
		next = FWD;
		break;
	}
break;


case WEST:
	switch(min) {
	case 1:
		next = RIGHT;
		break;
	case 2:
		next = LEFT;
		break;
	case 3:
		next = FWD;
		break;
	case 4:
		next = DEAD;
		break;
		}
break;
}

return next;
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

#if CLOCK_SPEED == 108
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
#elif CLOCK_SPEED = 216
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#endif

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = CLOCK_SPEED;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
#if CLOCK_SPEED == 108
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
#elif CLOCK_SPEED == 216
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
#endif
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void SystemClock_Config_old(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	  }

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	  }

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Configure the Systick interrupt time
	    */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	    /**Configure the Systick
	    */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  sConfig.Channel = ADC_CHANNEL_14; //l receiver
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15; //lf receiver
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8; //rf receiver
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_9; //r receiver
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

//LEFT ENCODER CHANNELS
static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;

  sConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
}

//MOTOR CONTROL PINS
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  //uhPrescalerValue = (uint32_t)((SystemCoreClock /2) / 18000000) - 1;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 665;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}
/*
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 665;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}
*/
//RIGHT ENCODER CHANNELS
static void MX_TIM4_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.RepetitionCounter = 0;

	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC1Filter = 0;

	sConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC2Filter = 0;

	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
	{
	  Error_Handler();
	}



}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = CLOCK_SPEED; //or 2160000 microsecond counter
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

//BUZZER OUTPUT CHANNEL
static void MX_TIM9_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 3375;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1790;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim9);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}


#if X_MAZE_SIZE == 16
void Test_Floodfill() {

	for (int x = X_FINAL; x <= X_FINAL + 1; x++) {
			  for (int y = Y_FINAL; y <= Y_FINAL + 1; y++) {

				  for (int i = 0; i <= 1; i++) {
					  if (y > Y_FINAL) {
						  horiz_walls[x][y] = i;

					  }
					  else {
						  horiz_walls[x][y - 1] = i;
					  }
					  final_x = x;
					  final_y = y;
					  sprintf(tx_buffer, "FINAL X: %d    FINAL Y: %d    HORIZ: %d ----------------  \r\n", final_x, final_y, i);
					  Transmit(tx_buffer);
					  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
					  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

					  Fill_Center();
					  Floodfill(FALSE, FALSE, TRUE);
					  Print_Maze();
					  Reset_Maze();

				  }

			  }
		  }
}
#endif
//button interrupt routine (THIS IS RUN WHEN BUTTONS ARE PRESSED)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if mouse_rev == 1
  if (GPIO_Pin == BUTTON2_Pin)
  {
	  //if (debug_flag == FALSE) {
	  stop_flag = !stop_flag;
	  //reverse_flag = FALSE;
	  //search_flag = FALSE;
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  //}
	  //else
	  //{
	//	send_debug = !send_debug;
	  //}
  }

  if (GPIO_Pin == BUTTON1_Pin)
  {
	 //Floodfill(FALSE, FALSE, TRUE);
	 //Print_Maze();
	 //Test_Floodfill();
	 HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	 HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	 __HAL_TIM_SET_COUNTER(&htim1, 0);
	 __HAL_TIM_SET_COUNTER(&htim4, 0);
	 debug_flag = !debug_flag;
  }
#elif MOUSE_REV == 69
  if (GPIO_Pin == GPIO_PIN_5)
  {
	  switch(button_state) {
	  case 0:
		  send_debug = 1;
		  stop_flag = 1;
		  dem1 = 0;
		  dem2 = 0;
		  dem3 = 0;
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, OFF);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, OFF);
		  button_state = 1;
		  break;
	  case 1:
		  stop_flag = 0;
		  send_debug = 0;
		  dem1 = 0;
		  dem2 = 0;
		  dem3 = 0;
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, OFF);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, OFF);

		  button_state = 2;
		  break;

	  case 2:
		  send_debug = 0;
		  stop_flag = 1;
		  dem1 = 1;
		  dem2 = 0;
		  dem3 = 0;
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ON);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, OFF);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, OFF);
		  button_state = 3;
		  break;
	  case 3:
		  send_debug = 0;
		  stop_flag = 1;
		  dem1 = 0;
		  dem2 = 1;
		  dem3 = 0;
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ON);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, OFF);
		  button_state = 4;
		  break;

	  case 4:
		  send_debug = 0;
		  stop_flag = 1;
		  dem1 = 0;
		  dem2 = 0;
		  dem3 = 1;
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, OFF);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, ON);
		  button_state = 0;
		  break;
	  }

  }
#endif
}

//Configure all digital input/output pins
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LDIC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, RDIC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

#if MOUSE_REV == 69
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(GPIOD, LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin , GPIO_PIN_RESET);
#endif


  /*Configure GPIO pins : RDIC_Pin LDIC_Pin GYRO_CS_Pin */
  GPIO_InitStruct.Pin = RDIC_Pin|LDIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if MOUSE_REV == 1
  HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pin : BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON2_GPIO_Port, &GPIO_InitStruct);

#endif

#if MOUSE_REV == 69
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
#endif

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  //Configure GPIO pins : PC8 PC9
#if MOUSE_REV == 69

  GPIO_InitStruct.Pin = LF_EMIT_PIN | L_EMIT_PIN | R_EMIT_PIN | RF_EMIT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED6_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED2_Pin |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

#else

  /*Configure GPIO pins : LED5_Pin LED4_Pin LED3_Pin LED2_Pin
                             LED1_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //Configure GPIO pins : PB4 PB5
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#endif

}

//ADC interrupt handler. Runs when all four channels have been converted
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* handle)
{
	Stop_IR();
	l = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 4];
	lf = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 3];
	rf = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 2];
	r = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 1];

}

//SPI Interrupt Handler
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
  //wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  //wTransferState = TRANSFER_ERROR;
  //HAL_GPIO_TogglePin(GPIOD, LED5_Pin);
}

void Error_Handler(void)
{
  while(1) 
  {
	  sprintf(tx_buffer, "State: %d", HAL_state);
	  Transmit(tx_buffer);
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
