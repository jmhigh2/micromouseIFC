/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

//IMPORTANT PARAMETERS
#define DEBUG TRUE
#define CLOCK_SPEED 216
#define MOUSE_REV 69 //69 is chode mouse, 2 is original rigged mouse

/* Private define ------------------------------------------------------------*/
#if MOUSE_REV == 1

#define RPWM_Pin GPIO_PIN_2
#define RPWM_GPIO_Port GPIOA
#define LDIC_Pin GPIO_PIN_1
#define RDIC_GPIO_Port GPIOA
#define RDIC_Pin GPIO_PIN_3
#define LDIC_GPIO_Port GPIOA
#define LPWM_Pin GPIO_PIN_0
#define LPWM_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_4
#define GYRO_CS_GPIO_Port GPIOA
#define GYRO_SCLK_Pin GPIO_PIN_5
#define GYRO_SCLK_GPIO_Port GPIOA
#define GYRO_MISO_Pin GPIO_PIN_6
#define GYRO_MISO_GPIO_Port GPIOA
#define GYRO_MOSI_Pin GPIO_PIN_7
#define GYRO_MOSI_GPIO_Port GPIOA
#define L_REC_Pin GPIO_PIN_4
#define L_REC_GPIO_Port GPIOC
#define LF_REC_Pin GPIO_PIN_5
#define LF_REC_GPIO_Port GPIOC
#define RF_REC_Pin GPIO_PIN_0
#define RF_REC_GPIO_Port GPIOB
#define R_REC_Pin GPIO_PIN_1
#define R_REC_GPIO_Port GPIOB
#define LENC_CHB_Pin GPIO_PIN_9
#define LENC_CHB_GPIO_Port GPIOE
#define LENC_CHA_Pin GPIO_PIN_11
#define LENC_CHA_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE
#define BUTTON1_Pin GPIO_PIN_15
#define BUTTON1_GPIO_Port GPIOD
#define BUTTON2_Pin GPIO_PIN_6
#define BUTTON2_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_1
#define LED5_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOD

#define LF_EMIT_PIN GPIO_PIN_4
#define L_EMIT_PIN GPIO_PIN_5
#define RF_EMIT_PIN GPIO_PIN_8
#define R_EMIT_PIN GPIO_PIN_9

//7 and 12 for mine
//6 and 13 for allens

#define RENCB_Pin GPIO_PIN_6
#define RENCB_GPIO_Port GPIOB
#define RENCA_Pin GPIO_PIN_13
#define RENCA_GPIO_Port GPIOD

#define LF_EMIT_PORT GPIOB
#define L_EMIT_PORT GPIOB
#define RF_EMIT_PORT GPIOC
#define R_EMIT_PORT GPIOC

#elif MOUSE_REV == 69

#define RPWM_Pin GPIO_PIN_2
#define RPWM_GPIO_Port GPIOA
#define RDIC_GPIO_Port GPIOA
#define RDIC_Pin GPIO_PIN_3

#define LDIC_Pin GPIO_PIN_1
#define LDIC_GPIO_Port GPIOA
#define LPWM_Pin GPIO_PIN_0
#define LPWM_GPIO_Port GPIOA

#define L_REC_Pin GPIO_PIN_4
#define L_REC_GPIO_Port GPIOC
#define LF_REC_Pin GPIO_PIN_5
#define LF_REC_GPIO_Port GPIOC
#define RF_REC_Pin GPIO_PIN_0
#define RF_REC_GPIO_Port GPIOB
#define R_REC_Pin GPIO_PIN_1
#define R_REC_GPIO_Port GPIOB
#define LENC_CHB_Pin GPIO_PIN_9
#define LENC_CHB_GPIO_Port GPIOE
#define LENC_CHA_Pin GPIO_PIN_11
#define LENC_CHA_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE

#define BUTTON1_Pin GPIO_PIN_15
#define BUTTON1_GPIO_Port GPIOD
#define BUTTON2_Pin GPIO_PIN_6
#define BUTTON2_GPIO_Port GPIOC

#define LED9_Pin GPIO_PIN_13 //boot0
#define LED9_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_10
#define LED8_GPIO_Port GPIOC
#define LED7_Pin GPIO_PIN_11
#define LED7_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_12
#define LED6_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_0
#define LED5_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE

#define LF_EMIT_PIN GPIO_PIN_3
#define L_EMIT_PIN GPIO_PIN_4
#define R_EMIT_PIN GPIO_PIN_1
#define RF_EMIT_PIN GPIO_PIN_2


#define RENCB_Pin GPIO_PIN_7
#define RENCB_GPIO_Port GPIOB
#define RENCA_Pin GPIO_PIN_12
#define RENCA_GPIO_Port GPIOD

#define LF_EMIT_PORT GPIOD
#define L_EMIT_PORT GPIOD
#define RF_EMIT_PORT GPIOD
#define R_EMIT_PORT GPIOD

#define GYRO_CS_Pin GPIO_PIN_4
#define GYRO_CS_GPIO_Port GPIOA
#define GYRO_SCLK_Pin GPIO_PIN_5
#define GYRO_SCLK_GPIO_Port GPIOA
#define GYRO_MISO_Pin GPIO_PIN_6
#define GYRO_MISO_GPIO_Port GPIOA
#define GYRO_MOSI_Pin GPIO_PIN_7
#define GYRO_MOSI_GPIO_Port GPIOA

#endif

#define ADCx_IRQn ADC_IRQn

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define DBG_BUFFER 20
#define FORWARD 1
#define BACKWARD 0

//GPIO states
#define ON 1
#define OFF 0

//flags
#define TRUE 1
#define FALSE 0

//dead end states
#define PEEK 1
#define EXECUTE 2
#define ARRIVE 0

//mouse movements
#define FWD 0
#define LEFT 1
#define RIGHT 2
#define DEAD 3
#define PAUSE 4
#define DONE 5
#define FWD_SPEED 6

//floodfill directions
#define NORTH 0
#define SOUTH 1
#define WEST 2
#define EAST 3

//movements
#define NORTH_X 0
#define NORTH_Y -1

#define SOUTH_X 0
#define SOUTH_Y 1

#define WEST_X -1
#define WEST_Y 0

#define EAST_X 1
#define EAST_Y 0

#define TEST 6
#define NONE 5
#define TIME 3
#define FWD_SEARCH 2
#define TURN_SEARCH 1
#define BARE 0

#define FLOODFILL 0
#define LEFT_WALL 1
#define RIGHT_WALL 2
#define SPEED 3

//REGULAR TURNS
#define FOR 'f'
#define LEF 'l'
#define RIGH 'r'

//FAST STRAIGHTS and TURNS
#define FWD0 'a'
#define FWD1 'b'
#define FWD2 'c'
#define FWD3 'd'
#define FWD4 'e'
#define FWD5 'v'
#define FWD6 'g'
#define FWD7 'h'
#define FWD8 'i'
#define FWD9 'j'
#define FWD10 'k'
#define FWD11 'm'
#define FWD12 'n'
#define FWD13 'o'
#define FWD14 'p'
#define FWD15 'q'
#define L90 'l'
#define L180 't'
#define R180 'u'
#define R90 'r'
#define STOP 's'

#define FUCKEDUP 69


/* USER CODE END Private defines */



#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
