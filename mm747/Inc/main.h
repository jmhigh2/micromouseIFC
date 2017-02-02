/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define VOLTMETER_Pin GPIO_PIN_0
#define VOLTMETER_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_1
#define LED5_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOC
#define RPWMA_Pin GPIO_PIN_0
#define RPWMA_GPIO_Port GPIOA
#define RPWMB_Pin GPIO_PIN_1
#define RPWMB_GPIO_Port GPIOA
#define LPWMA_Pin GPIO_PIN_2
#define LPWMA_GPIO_Port GPIOA
#define LPWMB_Pin GPIO_PIN_3
#define LPWMB_GPIO_Port GPIOA
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
#define RENC_CHB_Pin GPIO_PIN_13
#define RENC_CHB_GPIO_Port GPIOE
#define RENC_CHA_Pin GPIO_PIN_14
#define RENC_CHA_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_6
#define SW1_GPIO_Port GPIOC
#define R_EMIT_Pin GPIO_PIN_8
#define R_EMIT_GPIO_Port GPIOC
#define RF_EMIT_Pin GPIO_PIN_9
#define RF_EMIT_GPIO_Port GPIOC
#define LED9_Pin GPIO_PIN_2
#define LED9_GPIO_Port GPIOD
#define LED8_Pin GPIO_PIN_3
#define LED8_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_4
#define LED7_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOD
#define LF_EMIT_Pin GPIO_PIN_4
#define LF_EMIT_GPIO_Port GPIOB
#define L_EMIT_Pin GPIO_PIN_5
#define L_EMIT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
