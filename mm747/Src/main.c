/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "stm32f7xx_hal.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClock_Config_old(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
//static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//User functions and defines
#define MOUSE_NUM 1

/*MOUSE 1 PROPERTIES (FUCKED UP LF EMITTER) */
#if MOUSE_NUM == 1
//straightaway speed
#define FWD_L 130 //1
#define FWD_R 120 //135
//RIGHT turn speed
#define RIGHT_L 135
#define RIGHT_R 0
//LEFT turn speed
#define LEFT_L 0
#define LEFT_R 145
//left pivot
#define LPIVOT_L 110
#define LPIVOT_R 70
//right pivot
#define RPIVOT_L 110
#define RPIVOT_R 100
//how far to turn for dead ends
#define PIVOT_ENC 340
//dead end stop condidtion
#define STOP_CONDITION 3700
//read walls
#define LEFT_THRESHOLD 600
#define RIGHT_THRESHOLD 600
#define FRONT_THRESHOLD 400
//correction
#define RIGHT_CORRECTION 1300 //increase for closer to wall. decrease for farther from wall
#define LEFT_CORRECTION 1400 //increase for closer to wall
#define WALL_OFFSET 100

#elif MOUSE_NUM == 2 //the mouse with no fucked up emitters
//straightaway speed
#define FWD_L 130 //1
#define FWD_R 120 //135
//RIGHT turn speed
#define RIGHT_L 135
#define RIGHT_R 0
//LEFT turn speed
#define LEFT_L 0
#define LEFT_R 145
//left pivot
#define LPIVOT_L 110
#define LPIVOT_R 70
//right pivot
#define RPIVOT_L 110
#define RPIVOT_R 100
//how far to turn for dead ends
#define PIVOT_ENC 340
//dead end stop condidtion
#define STOP_CONDITION 3700
//read walls
#define LEFT_THRESHOLD 600
#define RIGHT_THRESHOLD 600
#define FRONT_THRESHOLD 400
//correction
#define RIGHT_CORRECTION 1300 //increase for closer to wall. decrease for farther from wall
#define LEFT_CORRECTION 1400 //increase for closer to wall
#define WALL_OFFSET 100

#endif //MOUSE_NUM == 1


//STUFF THATS SAME FOR BOTH
#define WALL_SAMPLES 10
#define F_LENC1 400
#define F_RENC1 400

//execute next move
#define F_LENC2 695
#define F_RENC2 695

#define RT_RENC_1 40
#define RT_LENC_1 460

#define RT_RENC_2 405
#define RT_LENC_2 405

#define LT_RENC_1 460
#define LT_LENC_1 40

#define LT_RENC_2 405
#define LT_LENC_2 405

#define DEAD_RENC1 140
#define DEAD_LENC1 140

#define DEAD_RENC2 295
#define DEAD_LENC2 295


//Stop Searching
void Stop(void);

//Motor Control
void Set_Left(int speed, int direction);  //set left motor
void Set_Right(int speed, int direction); //set right motor
int Motor_Correction(int l_wall, int r_wall, int debug); //motor correction

void Send_Buffers();
void Clear_Buffers();
void Reset_Flags();
void Reset_Maze();

//Sensors and Feedback
void Start_IR(void); //start DMA conversion of ADC values
void Stop_IR(void); //stop DMA conversion. used in interrupt routine
void Get_IR(int front, int side); //function to get all the IR values
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

//Maze Functions
void Floodfill();
int Get_Lowest_Square(int x, int y);
//int* Get_Surrounding(int x, int y);
void Read_Walls();
void Update_Position();
int Get_Next_Move();
int Get_Next_Dumb();
void Print_Maze();

void Calc_Optimal();
void Fast_Straights();
void Speed_Run();


void Side_Calibrate(); //for sending side walls
void Front_Calibrate(); //for sensing front walls
void Deadend_Calibrate(); //for moving to the end of the dead end
void Correction_Calibrate();
void Adjuster();

void Forward_Search();
void Left_Search();
void Right_Search();
void Dead_End();

void Forward_Spd(int num);
void Left_Spd(int num);
void Right_Spd(int num);

int HAL_state = 0; //debug state
char tx_buffer[200]; //UART buffers
char rx_buffer[200];

uint32_t l_count = 0; //encoder counts
uint32_t r_count = 0;
uint32_t prev_l_count;
uint32_t prev_r_count;

uint32_t l_count_corr = 0;
uint32_t r_count_corr = 0;

uint32_t lenc_diff_corr = 0;
uint32_t renc_diff_corr = 0;

uint32_t lenc_diff = 0; //l_count - prev_l_count
uint32_t renc_diff = 0;

int m_correction = 0; //correction variable for motor speed

//maze shit
#define X_MAZE_SIZE 6
#define Y_MAZE_SIZE 3

#define X_FINAL 5
#define Y_FINAL 0

#define X_START 0 //
#define Y_START 2 //or [Y_MAZE_SIZE-1]

#define START_DIR NORTH

/* NOTE ON MAZE: X = COLUMN NUMBER, Y = ROW NUMBER */

uint32_t maze[X_MAZE_SIZE][Y_MAZE_SIZE];
static uint32_t horiz_walls[X_MAZE_SIZE][Y_MAZE_SIZE - 1]; //horizontal walls have one less row
static uint32_t vert_walls[X_MAZE_SIZE - 1][Y_MAZE_SIZE]; //vertical walls have one less column

uint32_t x_coord = X_START; //maze position in maze
uint32_t y_coord = Y_START;
uint32_t prevx = 0;
uint32_t prevy = 0;
int cur_dir = START_DIR;
char optimal_path[100]; //regular path
char new_path[100]; //new path with straightaways

//Mouse movement
int cur_move = FWD;
int next_move = FWD;

//on - off
int dif_l = 0;
int dif_r = 0;
int dif_rf = 0;
int dif_lf = 0;
#define IR_BUFFER 1000

static int lf_buffer[IR_BUFFER];
static int rf_buffer[IR_BUFFER];
static int l_buffer[IR_BUFFER];
static int r_buffer[IR_BUFFER];

//values sampled multiple times
int front_l = 0;
int front_r = 0;
int lf_side = 0;
int rf_side = 0;

//int ir_values[4] = {0, 0, 0, 0};

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
int r_turnflag = FALSE;
int l_turnflag = FALSE;
int dead_flag = ARRIVE;
int fwd_flag = FALSE;
int l_transition_flag = FALSE;
int r_transition_flag = FALSE;
int transition_flag = FALSE;
int transition_set = FALSE;
int done_flag = FALSE;

//system flags. stop_flag stops everything. Press button to enable everything. Debug flag send debug data while sitting
int stop_flag = TRUE;
int debug_flag = FALSE;
int buzzer_flag = FALSE;

//SPI Buffer
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Interrupt **** SPI Message ******** SPI Message ******** SPI Message ****";
uint8_t aRxBuffer[BUFFERSIZE];

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  //MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();

  //buzzer
  //HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  //start PWM for motors
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  Set_Left(0, FORWARD); //set mouse to sit
  Set_Right(0, FORWARD);
  //END STARTUP


  //MAIN INFINITE PROGRAM LOOP aka ready loop
  while (1)
  {
	  if (stop_flag == FALSE || debug_flag == TRUE) { //press top button to activate IR sensors
	  Get_IR(FALSE, FALSE); //get IR sensor readings. diff_x = on_x - off_x
	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //check left and right encoder counts for debug
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);
	  //Adjuster();
	  //Send_Debug();
	  //m_correction = Motor_Correction(dif_lf, dif_rf, TRUE);
	  //sprintf(tx_buffer, "L Speed: %d   \r\nR Speed: %d   \r\nCorrection %d \r\n-----------------\r\n", FWD_L + m_correction, FWD_R - m_correction, m_correction);
	  //Transmit(tx_buffer);
 	  //}
	  }

	  if (stop_flag == FALSE && dif_l > 2000) { //start searching (place finger in front)

      HAL_GPIO_WritePin(GPIOD, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
      HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
      HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
      HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
      HAL_GPIO_WritePin(GPIOD, LED1_Pin, OFF);

      Clear_Buffers();
      Reset_Counters();
      Reset_Flags();
      Reset_Maze();

	  HAL_Delay(1000); //delay before start to get finger out of the way
	  if (debug_flag == TRUE){
		  Read_Walls();
	  }
	  Floodfill(x_coord, y_coord);
	  Update_Position();

	  Set_Left(FWD_L, FORWARD); //start going straight. get moving
	  Set_Right(FWD_R, FORWARD);

	  while(1) { //searching loop //while(maze[x_coord][y_coord] != 0)

	  Get_IR(FALSE, TRUE); //get IR readings. True is for saving the buffers

	  if  (cur_move == FWD || r_turnflag == TRUE || l_turnflag == TRUE || dead_flag == EXECUTE || dead_flag == PEEK) {
	  m_correction = Motor_Correction(dif_lf, dif_rf, FALSE);
	  Set_Left(FWD_L + m_correction, FORWARD);
	  Set_Right(FWD_R - m_correction, FORWARD);
	  //motor correction for straight part of turn and FWD moving
	  }

	  if (Emergency_Stop()) {
		  Stop();
		  break;
	  }

	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);

	  lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
	  renc_diff = r_count - prev_r_count;

	  //wall to no wall transition, or no wall to wall transition
	  if (cur_move == FWD && fwd_flag == FALSE && transition_set == FALSE && (r_transition_flag == TRUE || l_transition_flag == TRUE)) {
		  l_count_corr = l_count;
		  r_count_corr = r_count;
		  transition_flag = TRUE;
		  HAL_GPIO_WritePin(GPIOD, LED2_Pin, ON);
		  transition_set = TRUE;
	  }

	  if (transition_flag == TRUE) {
	  lenc_diff_corr = l_count - l_count_corr;
	  renc_diff_corr = r_count - r_count_corr;
	  }

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
//place in middel of mouse
Clear_Buffers();

int lf = 0;
int rf = 0;

for (int i = 0; i < IR_BUFFER; i++) {
	Get_IR(FALSE, TRUE);
}

for (int i = 0; i < IR_BUFFER; i++) {
lf = lf + lf_buffer[i];
rf = rf + rf_buffer[i];
}

lf = lf/IR_BUFFER;
rf = rf/IR_BUFFER;

sprintf(tx_buffer, "Correction: \r\nLEFT MIDDLE %d    RIGHT MIDDLE %d      OFFSET(LEFT - RIGHT) %d \r\n", lf, rf, lf - rf);
Transmit(tx_buffer);

}

void Deadend_Calibrate() {

	Clear_Buffers();
	buff_count = 0;
	int l_test = 0;
	int r_test = 0;

	for (int i = 0; i < IR_BUFFER; i++) {
		Get_IR(TRUE, FALSE);
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

void Forward_Search() {

	if (transition_flag == TRUE) { //if theres a transition sense. use this position instead
		if (fwd_flag == FALSE && (lenc_diff_corr >=  300 || renc_diff_corr >= 300)) { //330

			fwd_flag = TRUE;

			if (debug_flag == TRUE) { //read walls at entrance to square. Get the next move if need be. Will automatically floodfill if there are new walls
				Read_Walls();
				next_move = Get_Next_Move();
				Update_Position();
			}

			else {
				next_move = Get_Next_Dumb();
				Update_Position();
			}
			Save_State();
		}

		if (fwd_flag == TRUE && (lenc_diff_corr >=  620 || renc_diff_corr >= 620)) { //620

			fwd_flag = FALSE;
			transition_flag = FALSE;
			transition_set = FALSE;

			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

			prev_l_count = l_count; // Save encoder values
			prev_r_count = r_count;

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

	if (fwd_flag == FALSE && (lenc_diff >= F_LENC1 || renc_diff >= F_RENC1))
	{
		fwd_flag = TRUE;
		if (debug_flag == TRUE) {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
		}
		else {
			next_move = Get_Next_Dumb();
			Update_Position();
		}
		Save_State();
	}

	if (fwd_flag == TRUE && (lenc_diff >= F_LENC2 || renc_diff >= F_RENC2))
	{ //left and right wheel moving at same speed. If statement checks if distance has been covered

		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

		prev_l_count = l_count; //no change. keep going straight. Save encoder values
		prev_r_count = r_count;
		fwd_flag = FALSE;
		transition_flag = FALSE;

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

void Left_Search() {

	if ((l_turnflag == FALSE) && (lenc_diff >= LT_LENC_1 || renc_diff >= LT_RENC_1)) { //finished making turn. left and right wheel don't travel at same speeds

		Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		Set_Right(FWD_R, FORWARD);

		if (debug_flag == TRUE) {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
		}

		else {
			next_move = Get_Next_Dumb();
			Update_Position();
		}

		l_turnflag = TRUE;
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;

		Save_State();
	}

	if ((l_turnflag == TRUE) && (lenc_diff >= LT_LENC_2 || renc_diff >= LT_RENC_2)) { //made it to same point

		l_turnflag = FALSE;
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
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
void Right_Search() {

	if ((r_turnflag == FALSE) && (lenc_diff >= RT_LENC_1 || renc_diff >= RT_RENC_1)) { //finished making turn. left and right wheel don't travel at same speeds

		if (debug_flag == TRUE) {
			Read_Walls();
			next_move = Get_Next_Move();
			Update_Position();
		}
		else {
			next_move = Get_Next_Dumb();
			Update_Position();
		}

		Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		Set_Right(FWD_R, FORWARD);

		r_turnflag = TRUE;
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;

		Save_State();
	}

	if ((r_turnflag == TRUE) && (lenc_diff >= RT_LENC_2 || renc_diff >= RT_RENC_2)) { //made it to same point. execute next direction

		r_turnflag = FALSE;
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
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

void Dead_End() {

	if (dead_flag == ARRIVE) {

		Dead_End_Correct(); //should be aligned in middle of square

		Reset_Counters();
		if (done_flag == TRUE) {
			cur_move = PAUSE;
		}

		else {
		Set_Left(FWD_L, FORWARD);
		Set_Right(FWD_R, FORWARD); //move forward to next square
		dead_flag = PEEK;
		}
	}

	if (dead_flag == PEEK && (lenc_diff >= DEAD_LENC1 || renc_diff >= DEAD_RENC1)) { //get next state

		dead_flag = EXECUTE;
		if (debug_flag == TRUE) {
			next_move = Get_Next_Move();
			Update_Position();
		}

		else {
			next_move = Get_Next_Dumb();
			Update_Position();
		}
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;
		lenc_diff = 0;
		renc_diff = 0;
	}
	         //after correction
	if (dead_flag == EXECUTE && (lenc_diff > DEAD_LENC2 || renc_diff >= DEAD_RENC2)) {
		dead_flag = ARRIVE;
		prev_l_count = l_count;
		prev_r_count = r_count;

		switch (next_move) { //need to change direction or nah

		case FWD:
			//already going straight out of turn
			break;

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
	{HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);}
}

int Emergency_Stop() {

if (dif_l > STOP_CONDITION || dif_r > STOP_CONDITION || cur_move == PAUSE) //Emergency STOP conditions
	  {
		  if (cur_move == FWD || cur_move == PAUSE) //|| l_turnflag == TRUE || r_turnflag == TRUE)
		  {return TRUE;} //stops motors, and breaks out of searching loop
		  //else if (cur_move == RIGHT && r_turnflag == TRUE)
		  //{return TRUE;}
		  //else if (cur_move == LEFT && l_turnflag == TRUE)
		  //{return TRUE;}
		  //else if (abs(dif_l - dif_r) > 3600) {
			//  return TRUE;
		  //}
	  }
return FALSE;
}

int Motor_Correction(int l_wall, int r_wall, int debug) {

//int D = 100; //parameters
int P = 40;

int errorP = 0;
//int errorD = 0;
//int oldErrorP = 0;

int correction = 0;

if (l_wall > LEFT_THRESHOLD && r_wall > RIGHT_THRESHOLD) { //both walls available
	errorP = (l_wall - r_wall - WALL_OFFSET); //dif_rf + 500 for primary
	//errorD = errorP - oldErrorP;
}
else if (l_wall >= LEFT_THRESHOLD && r_wall <= RIGHT_THRESHOLD) { //only left wall to correct. Optimal reading should be 1600
	errorP = (l_wall - LEFT_CORRECTION); //75 is correction factor. Left side needs more corrections for some reason
	//errorD = errorP - oldErrorP;
}
else if (l_wall <= LEFT_THRESHOLD && r_wall >= RIGHT_THRESHOLD) {//only right wall to correct. Optimal reading should be 1500. (200 is offset)
	errorP = (RIGHT_CORRECTION - r_wall); //100 is correction factor. Right side needs less correction
	//errorD = errorP - oldErrorP;
}
else if (l_wall <= LEFT_THRESHOLD && r_wall <= RIGHT_THRESHOLD) { //use encoders when there's no walls available

	errorP = ((r_count - prev_r_count) - (l_count - prev_l_count))*10;

}

//motor correction adds to left motor
correction = errorP/P; //+ errorD/D; //P and D are tuning parameters
//oldErrorP = errorP;
if (abs(correction) < 120) {
	return correction;
}
else
{return 0;}

}

#define EQUAL_VAL 50

void Dead_End_Correct(void) {



//3700 for primary
 //align while going into square
do {
Get_IR(FALSE, FALSE);
m_correction = (dif_l - dif_r)/75;
Set_Left(FWD_L - m_correction, FORWARD);
Set_Right(FWD_R + m_correction, FORWARD);
} while (dif_r < 3700 && dif_l < 3700);

Set_Left(0, FORWARD);
Set_Right(0, FORWARD);

HAL_Delay(100);

//Turn to the right if right side is greater
Get_IR(FALSE, FALSE);
if (abs(dif_l - dif_r) > EQUAL_VAL) {
if (dif_r > dif_l) {
Set_Left(RPIVOT_L, FORWARD); //100
Set_Right(RPIVOT_R, BACKWARD); //140
}

else { //Turn to the left if the left side is greater
Set_Left(LPIVOT_L, FORWARD);
Set_Right(LPIVOT_R, BACKWARD);
}

do { //should be perfectly facing the back wall
Get_IR(FALSE, FALSE);
} while (abs(dif_l - dif_r) > EQUAL_VAL);

Set_Left(0, FORWARD);
Set_Right(0, FORWARD);

HAL_Delay(100);
}

Reset_Counters();

//rotate left
Set_Left(LPIVOT_L, BACKWARD); //100
Set_Right(LPIVOT_R, FORWARD); //140

do {
	r_count = __HAL_TIM_GET_COUNTER(&htim4);
} while (r_count < PIVOT_ENC);


Set_Left(0, FORWARD);
Set_Right(0, FORWARD);
//Floodfill(x_coord, y_coord);
HAL_Delay(100);
Reset_Counters();
Set_Left(FWD_L, BACKWARD);
Set_Right(FWD_R, BACKWARD);

do {
	r_count = __HAL_TIM_GET_COUNTER(&htim4);
} while ((r_count - 65536) < 50);

Set_Left(0, FORWARD);
Set_Right(0, FORWARD);
HAL_Delay(100);

}

void Reset_Counters() {

__HAL_TIM_SET_COUNTER(&htim1, 0); //reset counters
__HAL_TIM_SET_COUNTER(&htim4, 0);
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

void Get_IR(int front, int side) { //int* val_array

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

	//left sensor
	Start_IR();
	while (adc_conv == FALSE);
	off_l = l;
	HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, ON);
	Start_IR();
	while (adc_conv == FALSE);
	on_l = l;
	HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);

	//right sensor
	Start_IR();
	while (adc_conv == FALSE);
	off_r = r;
	HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, ON);
	Start_IR();
	while (adc_conv == FALSE);
	on_r = r;
	HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);

	//left front
	Start_IR();
	while (adc_conv == FALSE);
	off_lf = lf;
	HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, ON);
	Start_IR();
	while (adc_conv == FALSE);
	on_lf = lf;
	HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);

	//right front
	Start_IR();
	while (adc_conv == FALSE);
	off_rf = rf;
	HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, ON);
	Start_IR();
	while (adc_conv == FALSE);
	on_rf = rf;
	HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);

	dif_l = on_l - off_l; //val_array[0]
	dif_r = on_r - off_r; //val_array[1]
	dif_rf = on_rf - off_rf; //val_array[2]
	dif_lf = on_lf - off_lf; //val_array[3]

	//lf and rf transitions
	if (side == TRUE) {
	lf_buffer[buff_count] = dif_lf;
	rf_buffer[buff_count] = dif_rf;

	comp_lf = lf_buffer[((buff_count - 200) + IR_BUFFER) % IR_BUFFER];
	comp_rf = rf_buffer[((buff_count - 200) + IR_BUFFER) % IR_BUFFER];

	if (comp_lf > 10 && abs(dif_lf  - comp_lf)  > 275) {
		l_transition_flag = TRUE;
	}
	else {
		l_transition_flag = FALSE;
	}
	if (comp_rf > 10 && abs(dif_rf - comp_rf)  > 275) {
		r_transition_flag = TRUE;
	}
	else {
		r_transition_flag = FALSE;
	}

	buff_count = (buff_count+1) % IR_BUFFER;
	}

	if (front == TRUE) { //used only for calibration
		//buff_count = 0;
		l_buffer[buff_count] = dif_l;
		r_buffer[buff_count] = dif_r;
		buff_count = (buff_count + 1) % IR_BUFFER;
	}

}

void Clear_Buffers() {
	for (int i = 0; i < IR_BUFFER; i++) {

		lf_buffer[i] = 0;
		rf_buffer[i] = 0;
		r_buffer[i] = 0;
		l_buffer[i] = 0;

	}
}

void Reset_Flags() {

	r_turnflag = FALSE; //reset turn flags
	l_turnflag = FALSE;
	dead_flag = ARRIVE;
	fwd_flag = FALSE;
	l_transition_flag = FALSE;
	r_transition_flag = FALSE;
	transition_flag = FALSE;
	transition_set = FALSE;
	done_flag = FALSE;

	cur_dir = START_DIR;
	cur_move = FWD; //reset to default direction
	next_move = FWD;
	x_coord = X_START;
	y_coord = Y_START;
	prevx = X_START;
	prevy = Y_START;
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

	if (direction == FORWARD) {
			speed = 665 - speed;
	}

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
	HAL_GPIO_WritePin(GPIOA, RDIC_Pin, direction);

}

void Send_State(void) {

	int i = 0;
	//for(int i = 0; i < DBG_BUFFER; i++)
    while (l_debug[i] != 0 && i < dbg_count)
	{
	sprintf(tx_buffer, "Decision %d: \r\nL: %d   \r\nR: %d   \r\nRF: %d    \r\nLF: %d   \r\nCur Direc: %d \r\nNext Direction: %d  \r\nL Count: %d   R Count: %d \r\nPrev L: %d    Prev R: %d  \r\n---------------------\r\n", i, l_debug[i], r_debug[i], rf_debug[i], lf_debug[i], cur_debug[i], turn_debug[i], l_count_debug[i], r_count_debug[i], prev_l_debug[i], prev_r_debug[i]);
	Transmit(tx_buffer);
	i++;
	}
	dbg_count = 0;
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

void Readline(void) {

	int len = strlen(rx_buffer);
	HAL_UART_Receive(&huart1, rx_buffer, len, 5000);
	Transmit("HAHA");
}

void Stop(void) {

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD); //STOP
	if (debug_flag == TRUE) {
	//Floodfill(X_START, Y_START);
	Print_Maze();
	}

	debug_flag = FALSE;
	Send_State();
	//Send_Buffers();
	HAL_GPIO_WritePin(GPIOD, LED5_Pin, OFF);
	HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
	HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
	HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
	HAL_GPIO_WritePin(GPIOD, LED1_Pin, ON);
	stop_flag = TRUE;
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

	HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_SET);

	if(HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
	{
	    /* Transfer error in transmission process */
	    Error_Handler();
	}

	HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_RESET);

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

void Floodfill(int x, int y) { //int target x, int target y, position is global

int x_buffer[100]; //buffer to store coordinates
int y_buffer[100];

int temp_x = 0;
int temp_y = 0;

int buffer_counter = 0;
int pathdist = 1;

for (int i = 0; i < X_MAZE_SIZE; i++) { //initialize all values to 127
	for (int j = 0; j < Y_MAZE_SIZE; j++) {
		maze[i][j] = 127;
	}
}

//will be target x and target y
maze[X_FINAL][Y_FINAL] = 0;

/*center squares and test walls
maze[X_FINAL+1][Y_FINAL] = 0;
maze[X_FINAL][Y_FINAL+1] = 0;
maze[X_FINAL+1][Y_FINAL+1] = 0;

horiz_walls[13][0] = 1;
horiz_walls[13][1] = 1;
*/

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

	if (maze[x][y] != 127) { //if the algorithm has reached the desired position
		break;
	}
	pathdist++;
	}
//end algorithm while
}

void Reset_Maze() {

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
	Floodfill(X_START, Y_START);
}

void Read_Walls() {

front_l = 0;
front_r = 0;
lf_side = 0;
rf_side = 0;

for (int i = 0; i < WALL_SAMPLES; i++) {
	Get_IR(FALSE, FALSE);
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

void Update_Position() { //updates position and direction so read walls is good

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
			//x_coord = x_coord + NORTH_X;
			//y_coord = y_coord + NORTH_Y;
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
			//x_coord = x_coord + SOUTH_X;
			//y_coord = y_coord + SOUTH_Y;
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
			//x_coord = x_coord + WEST_X;
			//y_coord = y_coord + WEST_Y;
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
			//x_coord = x_coord + EAST_X;
			//y_coord = y_coord + EAST_Y;
			break;
		}
		break;
	}

	if (x_coord == X_FINAL && y_coord == Y_FINAL) {
		done_flag = TRUE;
		//next_move = DEAD;
		sprintf(tx_buffer, "|||FINAL||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
		Transmit(tx_buffer);
		return;
	}

	else if (next_move == DEAD) {
	x_coord = prevx;
	y_coord = prevy;
	sprintf(tx_buffer, "|||DEAD END||| \r\nX VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
	Transmit(tx_buffer);
	return;
	}

	sprintf(tx_buffer, "X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
	Transmit(tx_buffer);

}

void Calc_Optimal() {

const int N[2] = {0, -1};
const int S[2] = {0, 1};
const int W[2] = {-1, 0};
const int E[2] = {1, 0};

int cur_dir = START_DIR;
int counter = 0;

int x = X_START;
int y = Y_START;

int val = maze[x][y];
int next_values[4] = {-1, -1, -1, -1};
int index = 0;

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

switch (cur_dir) {
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
//sprintf(tx_buffer, "X VALUE: %d    Y VALUE: %d    DIRECTION: %d   VALUE: %d \r\n", x, y, cur_dir, val);
//Transmit(tx_buffer);
//while loop

}

optimal_path[counter] = 's';
//sprintf(tx_buffer, optimal_path);
//sprintf(tx_buffer, "FINAL:   X VALUE: %d    Y VALUE: %d    DIRECTION: %d   VALUE: %d \r\n", x, y, cur_dir, val);
//Transmit(tx_buffer);
Transmit(optimal_path);
Transmit("\r\n");
}

void Fast_Straights() {

char command;
int counter = 0;
int new_count = 0;
int fwd_count = 0;
int right_count = 0;
int left_count = 0;
char cmd_buff = FOR;

do {
	command = optimal_path[counter];
	if (command == FOR) {
		fwd_count++;
		if (right_count > 0) {
			new_path[new_count] = R90;
			right_count = 0;
			new_count++;
		}
		else if (left_count > 0) {
			new_path[new_count] = L90;
			left_count = 0;
			new_count++;
		}
	}
	else {
	if (fwd_count > 0) {
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
			new_path[new_count] = cmd_buff;
			new_count++;

		}
	if (command == LEF) {
		switch(left_count) {
		case(0):
			left_count++;
			if (right_count > 0) {
				new_path[new_count] = R90;
				right_count = 0;
				new_count++;
			}
			break;
		case(1):
			new_path[new_count] = L180;
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

				new_path[new_count] = L90;
				left_count = 0;
				new_count++;
			}
		break;
		case(1):
				new_path[new_count] = R180;
				right_count = 0;
				new_count++;
		break;
		}
	}
	else if (command == STOP) {
		switch(right_count) {

		case 0:
			break;
		case 1:
			cmd_buff = R90;
			break;
		}

		switch(left_count) {
		case 0:
			break;
		case 1:
			cmd_buff = L90;
			break;
		}

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
		new_path[new_count] = cmd_buff;
		new_path[new_count+1] = STOP;
	}
	fwd_count = 0;
	}
counter++;
} while (command != STOP);

Transmit(new_path);
}


void Speed_Run() {

//take optimal path and convert to fast straightaways
int counter = 0;
char ps =  new_path[counter];
counter++;
char ns = new_path[counter];
//l_count and r_count


while (ps != STOP) {

switch (ps) {

case L90:
	Left_Spd(1);
	break;
case R90:
	Right_Spd(1);
	break;
case FWD1:
	Forward_Spd(1);
	break;
case FWD2:
	Forward_Spd(2);
	break;
case FWD3:
	Forward_Spd(3);
	break;
case FWD4:
	Forward_Spd(4);
	break;
case FWD5:
	Forward_Spd(5);
	break;
case FWD6:
	Forward_Spd(6);
	break;
case FWD7:
	Forward_Spd(7);
	break;
case FWD8:
	Forward_Spd(8);
	break;
case FWD9:
	Forward_Spd(9);
	break;
case FWD10:
	Forward_Spd(10);
	break;
case FWD11:
	Forward_Spd(11);
	break;
case FWD12:
	Forward_Spd(12);
	break;
case FWD13:
	Forward_Spd(13);
	break;
case FWD14:
	Forward_Spd(14);
	break;
case FWD15:
	Forward_Spd(15);
	break;
}
counter++;
ps = ns;
ns = new_path[counter];
}
}

void Forward_Spd(int num) {

l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;

Set_Left(FWD_L+num*10, FORWARD);
Set_Right(FWD_R+num*10, FORWARD);

while (lenc_diff < F_LENC2*num/2 && renc_diff < F_RENC2*num/2) {
Get_IR(TRUE, TRUE);
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
}

Set_Left(FWD_L, FORWARD);
Set_Right(FWD_R, FORWARD);

while (lenc_diff < F_LENC2*num && renc_diff < F_RENC2*num) {
Get_IR(TRUE, TRUE);
l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
r_count = __HAL_TIM_GET_COUNTER(&htim4);
lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
renc_diff = r_count - prev_r_count;
}

}

void Left_Spd(int num) {

}

void Right_Spd(int num) {

}

int Get_Next_Dumb() {

int next = 0;

front_l = 0;
front_r = 0;
lf_side = 0;
rf_side = 0;

	for (int i = 0; i < WALL_SAMPLES; i++) {
		Get_IR(FALSE, FALSE);
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
	Floodfill(x_coord, y_coord);
	next = Get_Next_Move();
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 108;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15; //lf receiver
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8; //rf receiver
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_9; //r receiver
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
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
  htim2.Init.Prescaler = 5;
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

//button interrupt routine (THIS IS RUN WHEN BUTTONS ARE PRESSED)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON2_Pin)
  {
	  stop_flag = !stop_flag;
	  HAL_GPIO_TogglePin(GPIOD, LED4_Pin);

  }

  if (GPIO_Pin == BUTTON1_Pin)
  {
	  //Correction_Calibrate();
	  //Deadend_Calibrate();
	  //horiz_walls[5][1] = TRUE;

	  HAL_GPIO_TogglePin(GPIOD, LED3_Pin);

	  //Calc_Optimal();
	  // Fast_Straights();
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	 debug_flag = !debug_flag;
	 //HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
	 //buzzer_flag = !buzzer_flag;
	 //Set_Buzzer(3375, buzzer_flag);
  }
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
  HAL_GPIO_WritePin(GPIOA, RDIC_Pin|LDIC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_SET);


  /*Configure GPIO pins : RDIC_Pin LDIC_Pin GYRO_CS_Pin */
  GPIO_InitStruct.Pin = RDIC_Pin|LDIC_Pin|GYRO_CS_Pin;
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

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /*Configure GPIO pins : LED5_Pin LED4_Pin LED3_Pin LED2_Pin 
                           LED1_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //Configure GPIO pins : PC8 PC9
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   //Configure GPIO pins : PB4 PB5
      GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

//ADC interrupt handler. Runs when all four channels have been converted
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* handle)
{
	Stop_IR();
	l = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 4];
	r = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 1];
	rf = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 2];
	lf = ADC_valbuffer[ADC_VAL_BUFFER_LENGTH - 3];

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
