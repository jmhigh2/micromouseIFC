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
#include <string.h>
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

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

//User functions
void Set_Buzzer(int freq);
void Stop(void);
void Set_Left(int speed, int direction);  //set left motor
void Set_Right(int speed, int direction); //set right motor
void Transmit(char message[]); //send data
void Start_IR(void); //start DMA conversion of ADC values
void Stop_IR(void); //stop DMA conversion. used in interrupt routine
void Get_IR(void); //function to get all the IR values
void Dead_End_Correct(void); //algorithm to correct position in a dead end
void Send_Debug(void); //send debug stuff...counters + ir data
void Readline(void);
void Read_Gyro(void);
void Save_State(void); //save state into debug buffer for analysis
void Send_State(void); //send debug

int HAL_state = 0; //debug state
char tx_buffer[200]; //UART buffers
char rx_buffer[200];

uint32_t l_count = 0; //encoder counts
uint32_t r_count = 0;
uint32_t prev_l_count;
uint32_t prev_r_count;
uint32_t lenc_diff = 0; //l_count - prev_l_count
uint32_t renc_diff = 0;

int m_correction = 0; //correction variable for motor speed

//straightaway speed
#define FWD_L 130 //120
#define FWD_R 145 //135

//RIGHT pivot speed
#define RIGHT_L 200 //200
#define RIGHT_R 65 //65

//LEFT pivot speed
#define LEFT_L 50
#define LEFT_R 240

/*NOTE: FWD SPEED is run in second part of turn */

//encoder counts for straightaway
#define N_LENC 685
#define N_RENC 685

//encoder counts for turns
//1 is first part of turn when mouse is pivoting
//2 is second part of turn when mouse is going straight
//second encoder parts should technically be the first counts + number

#define E_RENC_1 230
#define E_LENC_1 630

#define E_RENC_2 530
#define E_LENC_2 830

#define W_RENC_1 550
#define W_LENC_1 230

#define W_RENC_2 850
#define W_LENC_2 530


//maze shit
uint32_t maze[3][6];
uint32_t x_coord = 5;
uint32_t y_coord = 2;
int cur_dir = NORTH;
int next_dir = NORTH;

//Mouse movement
int cur_move = FWD;
int next_move = FWD;

//sensor readings
int on_l = 0; //with emitter on
int on_r = 0;
int on_rf = 0;
int on_lf = 0;

int off_l = 0; //with emitter off. detect ambient light
int off_r = 0;
int off_rf = 0;
int off_lf = 0;

//on - off
int dif_l = 0;
int dif_r = 0;
int dif_rf = 0;
int dif_lf = 0;

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
static unsigned int turn_debug[DBG_BUFFER];
static unsigned int l_count_debug[DBG_BUFFER];
static unsigned int r_count_debug[DBG_BUFFER];
static unsigned int prev_l_debug[DBG_BUFFER];
static unsigned int prev_r_debug[DBG_BUFFER];
int dbg_count = 0;

//turn flags. These will be set when mouse is done pivoting and should be going straight. Can correct during this time
int r_turnflag = FALSE;
int l_turnflag = FALSE;

//system flags. stop_flag stops everything. Press button to enable everything. Debug flag send debug data while sitting
int stop_flag = TRUE;
int debug_flag = FALSE;

//for testing emitters. Not used in normal use
int ir_flag = 0;

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
	  if (stop_flag == FALSE) { //press top button to activate IR sensors
	  Get_IR(); //get IR sensor readings. diff_x = on_x - off_x
	  }

	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //check left and right encoder counts for debug
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);

	  if (stop_flag == FALSE && dif_l > 1500) { //start searching (place finger in front)

      HAL_GPIO_WritePin(GPIOD, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	  HAL_Delay(1000); //delay before start to get finger out of the way

	  __HAL_TIM_SET_COUNTER(&htim1, 0); //reset counters
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  prev_l_count = 0;
	  prev_r_count = 0;

	  r_turnflag = FALSE; //reset turn flags
	  l_turnflag = FALSE;

	  cur_move = FWD; //reset to default direction
	  next_move = FWD;

	  Set_Left(FWD_L, FORWARD); //start going straight
	  Set_Right(FWD_R, FORWARD);

	  while(1) { //searching loop

	  Get_IR(); //get IR readings

	  //motor correction for straight part of turn and FWD moving
	  if  (cur_move == FWD || r_turnflag == TRUE || l_turnflag == TRUE) {
		  if (dif_lf > 400 && dif_rf > 400) { //both walls available
		  	  m_correction = (dif_lf - (dif_rf+500))/100;
		  	  Set_Left(FWD_L + m_correction, FORWARD);
		  	  Set_Right(FWD_R - m_correction, FORWARD);
		  	  }

		  else if (dif_lf >= 250 && dif_rf <= 250) //only left wall to correct. Optimal reading should be 1600
		  {
		  m_correction = (dif_lf - 1600)/75; //75 is correction factor. Left side needs more corrections for some reason
		  Set_Left(FWD_L + m_correction, FORWARD);
		  Set_Right(FWD_R - m_correction, FORWARD);
		  }

		  else if (dif_lf <= 250 && dif_rf >= 250) //only right wall to correct. Optimal reading should be 1500. (200 is offset)
		  {
			  m_correction = (1500 - 200 - dif_rf)/100; //100 is correction factor. Right side needs less correction
			  Set_Left(FWD_L + m_correction, FORWARD);
			  Set_Right(FWD_R - m_correction, FORWARD);
		  }

		  else if (dif_lf < 250 || dif_rf < 250) //no correction when nothing available
		  {
			  Set_Left(FWD_L, FORWARD);
			  Set_Right(FWD_R, FORWARD);
		  }
	  }


	  if (dif_l > 3700 || dif_r > 3700) //Emergency STOP conditions
	  {
		  if (cur_move == FWD)
		  {Stop(); break;} //stops motors, and breaks out of searching loop
		  else if (cur_move == RIGHT && r_turnflag == TRUE)
		  {Stop(); break;}
		  else if (cur_move == LEFT && l_turnflag == TRUE)
		  {Stop(); break;}
		  //else if (dif_lf > 700 && dif_rf > 700)
		  //{Stop(); break;}
	  }

	  //get next direction
	  switch (cur_move) {

	  case FWD:
		  if (((dif_l >= 500) || (dif_r >= 500)) && (dif_rf <= 250)) //if front and right side is not blocked
		  {next_move = RIGHT;}
		  else if (((dif_l >= 500) || (dif_r >= 500)) && (dif_lf <= 250)) //if front and right side is blocked, but left is not
		  {next_move = LEFT;}
		  else if (((dif_l >= 500) || (dif_r >= 500)) && (dif_lf > 500) && dif_rf > 500)
		  {next_move = DEAD;}
		  //the default next_move is the cur_move, so if the front isn't blocked, keep going straight
		  break;

	  case RIGHT:
		  if (r_turnflag == TRUE && ((dif_l <= 250) || (dif_r <= 250))) //r_turnflag means the second part of the turn
		  {next_move = FWD;}
		  else if (r_turnflag == TRUE && dif_rf > 250)
		  {next_move = LEFT;}
		  else if (((dif_l >= 500) || (dif_r >= 500)) && (dif_lf > 500) && dif_rf > 500)
		  {next_move = DEAD;}
		  break;

	  case LEFT:
		  if (l_turnflag == TRUE && ((dif_l <= 250) || (dif_r <= 250)))
		  {next_move = FWD;}
		  else if (l_turnflag == TRUE && dif_lf > 250)
		  {next_move = RIGHT;}
		  else if (((dif_l >= 500) || (dif_r >= 500)) && (dif_lf > 500) && dif_rf > 500)
		  {next_move = DEAD;}
		  break;
	  }

	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);
	  lenc_diff = l_count - prev_l_count; //get difference between last encoder counts. prev_l and prev_r are updated when traveling 1 unit
	  renc_diff = r_count - prev_r_count;

	  switch (cur_move) { //main case statement. While moving, check distance traveled. If 1 unit has been covered, execute next move
	  	  	  	  	  	  //will eventually combine with above statement
	  case FWD:

	  if (lenc_diff >= N_LENC || renc_diff >= N_RENC) { //left and right wheel moving at same speed. If statement checks if distance has been covered

		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		Save_State();
		prev_l_count = l_count; //no change. keep going straight. Save encoder values
		prev_r_count = r_count;

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
	  break;

	  case RIGHT: //break up turn into turn and accelerate

	  if ((r_turnflag == FALSE) && (lenc_diff >= E_LENC_1 || renc_diff >= E_RENC_1)) { //finished making turn. left and right wheel don't travel at same speeds

		  Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		  Set_Right(FWD_R, FORWARD);
		  r_turnflag = TRUE;
	  }

	  if ((r_turnflag == TRUE) && (lenc_diff >= E_LENC_2 || renc_diff >= E_RENC_2)) { //made it to same point. execute next direction

		r_turnflag = FALSE;
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		Save_State();
		prev_l_count = l_count; //save current counters
		prev_r_count = r_count;

        switch (next_move) { //need to change direction or nah

        	    case FWD:
        	    //already going straight out of turn
        	    break;

        	    case RIGHT:
        	    Set_Left(RIGHT_L, FORWARD); //need to make right turn again
        	    Set_Right(RIGHT_R, FORWARD);
        		break;

        	    case LEFT:
        	    Set_Left(LEFT_L, FORWARD); //need to make right turn again
        	    Set_Right(LEFT_R, FORWARD);
        	    break;

        	    case DEAD:
        	    break;
        }
        cur_move = next_move;
	  } //case RIGHT
	  break;

	  case LEFT:

		  if ((l_turnflag == FALSE) && (lenc_diff >= W_LENC_1 || renc_diff >= W_RENC_1)) { //finished making turn. left and right wheel don't travel at same speeds

		  		  Set_Left(FWD_L, FORWARD); //finish turn by accelerating forward
		  		  Set_Right(FWD_R, FORWARD);
		  		  l_turnflag = TRUE;
		  	  }

		  if ((l_turnflag == TRUE) && (lenc_diff >= W_LENC_2 || renc_diff >= W_RENC_2)) { //made it to same point

		  	l_turnflag = FALSE;
		  	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  	Save_State();
		  	prev_l_count = l_count; //save current counters
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

		    case DEAD:
		    break;
		    }
		    cur_move = next_move;
     	  } //case LEFT
		  break;

	  case DEAD:
		  	 if (lenc_diff >= 60 || renc_diff >= 60) {

		  	 Set_Left(0, FORWARD); //stop
		  	 Set_Right(0, FORWARD);
		  	 Dead_End_Correct();
		  	 cur_move = next_move;
		  	 }
		  break;
	  } //switch

	  } //searching loop
	  } //if front wall > 1500

	  //ready loop again
	  HAL_Delay(500); //ONLY CHECK FOR FINGER every half second. If you check to quickly it'll never start

	  //DEBUG SHIT. It'll only transmit when it is waiting. Won't take up time while searching
	  if (debug_flag == TRUE){
	  m_correction = (dif_lf - dif_rf)/50;
	  sprintf(tx_buffer, "L Speed: %d   \r\nR Speed: %d   \r\nCorrection %d \r\n-----------------\r\n", FWD_L + m_correction, FWD_R -m_correction, m_correction);
	  Transmit(tx_buffer);
	  Send_Debug();
	  }
  } //ready loop
} //main function

//DONT USE YET
void Set_Buzzer(int freq) {

	int duty = freq/2;

	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);

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

void Dead_End_Correct(void) {


Get_IR();
//make front emitters equal (turn motors)
//Get_IR():
//whichever front sensor is higher. Turn the opposite way and go straight for a certain distance
//will need to do some encoder math for distance to travel based on difference
//when correction is applied,do u-turn and reset encoders


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

void Get_IR() {

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

	dif_l = on_l - off_l;
	dif_r = on_r - off_r;
	dif_rf = on_rf - off_rf;
	dif_lf = on_lf - off_lf;

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
    while (l_debug[i] != 0 && i < DBG_BUFFER)
	{
	sprintf(tx_buffer, "Decision %d: \r\nL: %d   \r\nR: %d   \r\nRF: %d    \r\nLF: %d   \r\nNext Direction: %d  \r\nL Count: %d   R Count: %d \r\nPrev L: %d    Prev R: %d  \r\n---------------------\r\n", i, l_debug[i], r_debug[i], rf_debug[i], lf_debug[i], turn_debug[i], l_count_debug[i], r_count_debug[i], prev_l_debug[i], prev_r_debug[i]);
	Transmit(tx_buffer);
	i++;
	}
	dbg_count = 0;
}

void Send_Debug(void) {

	sprintf(tx_buffer, "L Value: %d  LF Value: %d \r\nRF Value: %d R Value: %d \r\n--------------------- \r\n", on_l, on_lf, on_rf, on_r); //lf, rf, r);
	Transmit(tx_buffer); //transmitm the message above
	sprintf(tx_buffer, "OFF_L: %d  OFF_R: %d \r\nOFF RF: %d OFF LF: %d \r\n--------------------- \r\n", off_l, off_lf, off_rf, off_r); //lf, rf, r);
	Transmit(tx_buffer); //transmitm the message above
	sprintf(tx_buffer, "Left Count Value: %d \r\nRight Count Value %d \r\n-----------------\r\n", l_count, r_count);
	Transmit(tx_buffer); //transmit the message above
	sprintf(tx_buffer, "Prev_L: %d \r\nPrev R %d \r\n-----------------\r\n", prev_l_count, prev_r_count);
	Transmit(tx_buffer); //transmit the message above
	sprintf(tx_buffer, "cur_move: %d \r\nnext_move: %d\r\n-----------------\r\n", cur_move, next_move);
	Transmit(tx_buffer);
	sprintf(tx_buffer, "RIGHT Turn Flag: %d \r\nLEFT Turn Flag: %d \r\n-----------------------", r_turnflag, l_turnflag);
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
	Send_Debug();
	}
	Send_State();
	HAL_GPIO_WritePin(GPIOD, LED5_Pin, OFF);
	HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
	stop_flag = TRUE;

}

void Save_State(void) {

	l_debug[dbg_count] = dif_l;
	r_debug[dbg_count] = dif_r;
	rf_debug[dbg_count] = dif_rf;
	lf_debug[dbg_count] = dif_lf;
	turn_debug[dbg_count] = cur_move;
	l_count_debug[dbg_count] = l_count;
	r_count_debug[dbg_count] = r_count;
	prev_l_debug[dbg_count] = prev_l_count;
	prev_r_debug[dbg_count] = prev_r_count;

	dbg_count++;

	if (dbg_count >= DBG_BUFFER)
		{dbg_count = 0;}
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

void SystemClock_Config(void)
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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


  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	//state = 1;
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	//state = 2;
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	//state = 3;
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_9;
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
	  //HAL_GPIO_TogglePin(L_EMIT_PORT, L_EMIT_PIN);
	  //HAL_GPIO_TogglePin(LF_EMIT_PORT, LF_EMIT_PIN);
	  //Set_Left(100, FORWARD);
	  //Set_Right(120, FORWARD);


  }

  if (GPIO_Pin == BUTTON1_Pin)
  {
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	 debug_flag = !debug_flag;
	 HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
	  /*
	  HAL_GPIO_TogglePin(L_EMIT_PORT, L_EMIT_PIN);
	  //HAL_GPIO_TogglePin(R_EMIT_PORT, R_EMIT_PIN);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);
	  HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);
	  HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
		*/
	  /*
	 switch(ir_flag) {

	 case 0:
		 HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, ON);
		 HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
		 ir_flag = 1;
     break;


	 case 1:
		 HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, ON);
		 HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
		 ir_flag = 2;
     break;

	 case 2:
		 HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, ON);
		 HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, OFF);
		 ir_flag = 3;
     break;
	 case 3:
		 HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, OFF);
		 HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, ON);
		 ir_flag = 0;
     break;

	 }
	 */
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
	//HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
	l = ADC_valbuffer[28];
	r = ADC_valbuffer[31];
	rf = ADC_valbuffer[30];
	lf = ADC_valbuffer[29];

}

//SPI Interrupt Handler
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
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
