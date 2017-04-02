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
void Stop_Motor(void);
void Set_Left(int speed, int direction);
void Set_Right(int speed, int direction);
void Transmit(char message[]);
void Start_IR(void);
void Stop_IR(void);
void Get_IR(void);
void Send_Debug(void);
void Readline(void);
void Read_Gyro(void);
void Save_State(void);
void Send_State(void);

int HAL_state = 0; //debug state
char tx_buffer[200]; //UART buffers
char rx_buffer[200];

uint32_t l_count = 0; //encoder counts and m_speed variable
uint32_t r_count = 0;
uint32_t prev_l_count;
uint32_t prev_r_count;
uint32_t m_speed = 100;
unsigned int m_correction = 0;


//maze shit
uint32_t maze[3][6];
uint32_t x_coord = 5;
uint32_t y_coord = 2;
int cur_dir = NORTH;
int next_dir = NORTH;

//states and flags
int on_l = 0;
int on_r = 0;
int on_rf = 0;
int on_lf = 0;

int off_l = 0;
int off_r = 0;
int off_rf = 0;
int off_lf = 0;

volatile unsigned int l = 0;
volatile unsigned int r = 0;
volatile unsigned int rf = 0;
volatile unsigned int lf = 0;

static unsigned int l_debug[DBG_BUFFER];
static unsigned int r_debug[DBG_BUFFER];
static unsigned int rf_debug[DBG_BUFFER];
static unsigned int lf_debug[DBG_BUFFER];
static unsigned int turn_debug[DBG_BUFFER];
int dbg_count = 0;


volatile int adc_conv = FALSE;

int e_turnflag = FALSE;
int w_turnflag = FALSE;

int stop_flag = FALSE;
int debug_flag = FALSE;

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
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //turn on emitters at startup CHECK main.h for #defines
  //HAL_GPIO_WritePin(L_EMIT_PORT, L_EMIT_PIN, ON);
  //HAL_GPIO_WritePin(LF_EMIT_PORT, LF_EMIT_PIN, ON);
  //HAL_GPIO_WritePin(RF_EMIT_PORT, RF_EMIT_PIN, ON);
  //HAL_GPIO_WritePin(R_EMIT_PORT, R_EMIT_PIN, ON);
  Set_Left(0, FORWARD);
  Set_Right(0, FORWARD);


  //END STARTUP


  //MAIN INFINITE PROGRAM LOOP aka ready loop
  while (1)
  {
	  Get_IR(); //on_xx and off_xx are globals for all the variables

	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //check left and right encoder counts for debug
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);

	  if ((stop_flag == FALSE) && (on_l > 4000)) { //start searching (place finger in front)

      HAL_GPIO_WritePin(GPIOD, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	  HAL_Delay(1000); //delay before start to get finger out of the way

	  __HAL_TIM_SET_COUNTER(&htim1, 0); //reset counters
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  prev_l_count = 0;
	  prev_r_count = 0;

	  int cur_dir = NORTH;
	  int next_dir = NORTH;

	  Set_Left(m_speed, FORWARD); //start going straight
	  Set_Right(m_speed+15, FORWARD); //quite the offset there mate

	  while(1) { //searching loop

	  Get_IR();

	  //motor correction
	  if (cur_dir == NORTH && on_lf > 600 && on_rf > 600) {

	  m_correction = ((on_lf - off_lf) - (on_rf - off_rf))/100;
	  Set_Left(m_speed + m_correction, FORWARD);
	  Set_Right(m_speed + 15 - m_correction, FORWARD);
	  }

	  else if (cur_dir == NORTH && on_lf) {


	  }

	  //STOPxzxz
	  if (cur_dir == NORTH && (on_l-off_l > 3600 || on_r-off_r > 3600)) { //you're about to crash
	 	Stop_Motor();
	 	break; //exit the searching loop and go back to ready loop
	  }

	  //get next direction
	  switch (cur_dir) {

	  case NORTH:

		  if (((on_l - off_l >= 500) || (on_r - off_r >= 500)) && (on_rf- off_rf <= 250))
		  {next_dir = EAST;}

		  else if (((on_l - off_l >= 500) || (on_r - off_r >= 500)) && (on_lf- off_lf <= 250))
		  {next_dir = WEST;}

		  break;

	  case EAST:
		  if (e_turnflag == TRUE && ((on_l-off_l <= 250) || (on_r - off_r <= 250)))
		  {next_dir = NORTH;}

		  else if (e_turnflag == TRUE && on_rf - off_rf > 250)
		  {next_dir = WEST;}
		  break;


	  case WEST:
		  if (w_turnflag == TRUE && ((on_l-off_l <= 250) || (on_r - off_r <= 250)))
		  {next_dir = NORTH;}

		  else if (w_turnflag == TRUE && on_lf - off_lf > 250)
		  {next_dir = EAST;}
		  break;
	  }


	  l_count = __HAL_TIM_GET_COUNTER(&htim1); //get encoder counts. Encoders working in background and are automatically updated
	  r_count = __HAL_TIM_GET_COUNTER(&htim4);

	  switch (cur_dir) { //main case statement. While moving, check distance traveled. If 1 unit has been covered, execute next move

	  case NORTH:
	  if ((l_count-prev_l_count) >= 700 || ((r_count-prev_r_count) >= 700)) { //left and right wheel moving at same speed. If statement checks if distance has been covered
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		cur_dir = next_dir; //execute next move

		Save_State();

	    switch (cur_dir) { //check if motor speeds have to change with next move
	    case NORTH:
	    prev_l_count = l_count; //no change. keep going straight. Save encoder values
	    prev_r_count = r_count;
	    break;

	    case EAST:
		Set_Left(125, FORWARD); //need to make right turn
		Set_Right(30, FORWARD);
		prev_l_count = l_count;
		prev_r_count = r_count;

		break;

	    case WEST:
	    Set_Left(15, FORWARD); //need to make left turn
	    Set_Right(125, FORWARD);
	    prev_l_count = l_count;
	    prev_r_count = r_count;

	    }

        //x_coord = x_coord + NORTH_X;
        //y_coord = y_coord + NORTH_Y;
        //sprintf(tx_buffer, "X Location: %u   Y Location: %u \r\n", x_coord, y_coord);
        //Transmit(tx_buffer);
	  }
	  break;

	  case EAST: //break up turn into turn and accelerate

	  if ((e_turnflag == FALSE) && ((l_count-prev_l_count) >= 480 || ((r_count-prev_r_count) >= 65))) { //finished making turn. left and right wheel don't travel at same speeds

		  Set_Left(m_speed, FORWARD); //finish turn by accelerating forward
		  Set_Right(m_speed+15, FORWARD);
		  e_turnflag = TRUE;
	  }

	  if ((e_turnflag == TRUE) && ((l_count-prev_l_count) >= 750 || ((r_count-prev_r_count) >= 330))) { //made it to same point
		cur_dir = next_dir;
		e_turnflag = FALSE;

		Save_State();

        switch (cur_dir) { //need to change direction or nah

        	    case NORTH:
        	    prev_l_count = l_count; //save current counters
        	    prev_r_count = r_count;
        	    //already going straight out of turn
        	    break;

        	    case EAST:
        	    Set_Left(125, FORWARD); //need to make right turn again
        	    Set_Right(30, FORWARD);
        		prev_l_count = l_count; //save current counters
        		prev_r_count = r_count;

        		break;

        	    case WEST:

        	    Set_Left(30, FORWARD); //need to make right turn again
        	    Set_Right(125, FORWARD);
        	    prev_l_count = l_count; //save current counters
        	    prev_r_count = r_count;

        	    break;
        	    }

		//get nearest neighbor


	  } //case east
	  break;

	  case WEST:

		  if ((w_turnflag == FALSE) && ((l_count-prev_l_count) >= 65 || ((r_count-prev_r_count) >= 480))) { //finished making turn. left and right wheel don't travel at same speeds

		  		  Set_Left(m_speed, FORWARD); //finish turn by accelerating forward
		  		  Set_Right(m_speed+15, FORWARD);
		  		  w_turnflag = TRUE;
		  	  }

		  if ((w_turnflag == TRUE) && ((l_count-prev_l_count) >= 330 || ((r_count-prev_r_count) >= 750))) { //made it to same point
		  	cur_dir = next_dir;
		  	w_turnflag = FALSE;

		  	Save_State();

		    switch (cur_dir) { //need to change direction or nah

		    case NORTH:
		        prev_l_count = l_count; //save current counters
		        prev_r_count = r_count;
		     	    //already going straight out of turn
		        break;

		    case WEST:
		    Set_Left(15, FORWARD); //need to make right turn again
		    Set_Right(125, FORWARD);
		    prev_l_count = l_count; //save current counters
		    prev_r_count = r_count;

		    	break;

		    case EAST:
		    Set_Left(125, FORWARD); //need to make right turn again
		    Set_Right(30, FORWARD);
		    prev_l_count = l_count; //save current counters
		    prev_r_count = r_count;

		    	break;
		    }

		  		//get nearest neighbor

     	  } //case west
		  break;


	  } //switch

	  } //searching loop
} //if front wall > 1500


	  //ready loop again
	  HAL_Delay(500); //ONLY CHECK FOR FINGER every half second. If you check to quickly it'll never start

	  //DEBUG SHIT. It'll only transmit when it is waiting. Won't take up time while searching
	  if (debug_flag == TRUE){
	  m_correction = ((on_lf - off_lf) - (on_rf - off_rf))/50;
	  sprintf(tx_buffer, "L Speed: %d   \r\nR Speed: %d   \r\nCorrection %d \r\n-----------------\r\n", m_speed + m_correction, m_speed+15-m_correction, m_correction);
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

	for(int i = 0; i < DBG_BUFFER; i++)
	{
	sprintf(tx_buffer, "Decision %d: \r\nL: %d   \r\nR: %d   \r\nRF: %d    \r\nLF: %d   \r\nDirection: %d  \r\n---------------------\r\n", i, l_debug[i], r_debug[i], rf_debug[i], lf_debug[i], turn_debug[i]);
	Transmit(tx_buffer);
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
	sprintf(tx_buffer, "Cur_Dir: %d \r\nNext_Dir: %d\r\n-----------------\r\n", cur_dir, next_dir);
	Transmit(tx_buffer);
	sprintf(tx_buffer, "East Turn Flag: %d \r\nWest Turn Flag: %d \r\n-----------------------", e_turnflag, w_turnflag);
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

void Stop_Motor(void) {

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD); //STOP
	if (debug_flag == TRUE) {
	Send_Debug();
	}
	Send_State();
	HAL_GPIO_WritePin(GPIOD, LED5_Pin, OFF);
	stop_flag = TRUE;

}

void Save_State(void) {

	l_debug[dbg_count] = on_l - off_l;
	r_debug[dbg_count] = on_r - off_r;
	rf_debug[dbg_count] = on_rf - off_rf;
	lf_debug[dbg_count] = on_lf - off_lf;
	turn_debug[dbg_count] = cur_dir;

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
	  //HAL_GPIO_TogglePin(L_EMIT_PORT, L_EMIT_PIN);
	  //HAL_GPIO_TogglePin(LF_EMIT_PORT, LF_EMIT_PIN);
	  //Set_Left(100, FORWARD);
	  //Set_Right(120, FORWARD);
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  debug_flag = !debug_flag;

  }

  if (GPIO_Pin == BUTTON1_Pin)
  {
	  stop_flag = FALSE;
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
