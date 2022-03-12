/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct Command{
	uint8_t inst;
	int16_t val1;
	int16_t val2;
}Command;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1416
#define ICM_ADR (0x68 << 1)
#define AK_ADR (0x0C << 1)
#define TIMEOUT 100
#define DEF_SPEED 400 // default is 400
#define CMD_ARR_SIZE 100
#define DEBUG_MSG 1
#define CORRIDOR 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SonicSensorTask */
osThreadId_t SonicSensorTaskHandle;
const osThreadAttr_t SonicSensorTask_attributes = {
  .name = "SonicSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRSensorsTask */
osThreadId_t IRSensorsTaskHandle;
const osThreadAttr_t IRSensorsTask_attributes = {
  .name = "IRSensorsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartUARTTask(void *argument);
void SonicSensor(void *argument);
void IRSensors(void *argument);

/* USER CODE BEGIN PFP */
void ICM_Init(void);
void UserInit(void);
void MotorControl_fine(int dutyL, int dutyR);
void ServoControl(uint8_t pwm);
uint8_t* MovementInit();
uint8_t MotorTurn(int16_t target, int16_t speed);
uint8_t MotorStraight(int16_t target, int16_t speed);
uint8_t TestFunc(int16_t val1, int16_t val2);
uint8_t CalFunc(int16_t val1, int16_t val2);
uint8_t CalFunc_Blank(int16_t val1, int16_t val2);
void ApplyCorrection();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[20];
int EncodACurrent, EncodAPrev, EncodBCurrent, EncodBPrev, EncodADiff, EncodBDiff; //for Encoder
//int servo_direction; //for Servomotor
int16_t theta;

int16_t gyro_val;
int16_t cum_gyro;
float gyro_offset = 0.294;
int32_t cum_enc;
uint8_t UART_buffer[50];

Command cmd_arr[CMD_ARR_SIZE];
uint8_t cur_cmd;
uint8_t nxt_cmd;

//uint16_t bearing = 0;
int8_t pre_correction = 0;
int8_t post_correction = 0;
int8_t cal_correction = 0;
uint8_t power_correction = 0;
uint8_t st_gy_gain;

//
//  |      |
//  | <--> | bot
//  |      |
// if 22-> cal_corr = 2 -> left 2

//	^	y (0 deg)
//	|
//--+-->
//	|	x (90 deg)
//	|

float voltage1, voltage2; //Voltage level detection for IRSensors
uint32_t irDist1, irDist2; //IRSensor
uint16_t ADC_VAL1, ADC_VAL2; //ADC conversion required for IR Sensors

uint8_t Is_First_Captured = 0; // is the first value captured?
uint32_t IC_Value1 = 0; //For Ultrasonic
uint32_t IC_Value2 = 0; //For Ultrasonic
uint32_t ICValDiff = 0; //For Ultrasonic
uint32_t ICValDist = 0; //For Ultrasonic

void ICM_Init(void)
{
	uint8_t reg;

	//init ICM
	//turn on
	HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x06,1,&reg,1, TIMEOUT);
	reg = (reg & ~0x47) | 0x01; //Clear Sleep mode and set timer to PLL
	HAL_I2C_Mem_Write(&hi2c1, ICM_ADR, 0x06,1,&reg,1, TIMEOUT);
	//enable bypass mode
	HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x0F,1,&reg,1, TIMEOUT);
	reg = reg|0x02; //set bypass mode
	HAL_I2C_Mem_Write(&hi2c1, ICM_ADR, 0x0F,1,&reg,1, TIMEOUT);

	//init AK
	//reset chip
	reg = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, AK_ADR, 0x32,1,&reg,1, TIMEOUT); //Reset compass
	osDelay(50);
	//timer mode
	reg = 0x08; //Set continuous measurement mode 4 (100Hz refresh)
	HAL_I2C_Mem_Write(&hi2c1, AK_ADR, 0x31,1,&reg,1, TIMEOUT);
}

void UserInit(){

	//osDelay(500); //delay to ensure all init previously

	//Init UART
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 1);

	//init Motor
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	MotorControl_fine(0,0);
	power_correction = 0;
	if(CORRIDOR)
		st_gy_gain = 2;
	else
		st_gy_gain = 3;

	//init servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	ServoControl(75);

	//init OLED
	OLED_Init();

	//init encoder
	EncodADiff = EncodBDiff = 0;
	cum_enc = 0;

	//init ICM
	ICM_Init();
	cum_gyro = 0;

	//init command arr
	for (cur_cmd = 0; cur_cmd<CMD_ARR_SIZE; cur_cmd++){
		cmd_arr[cur_cmd].inst = 0;
		cmd_arr[cur_cmd].val1 = 0;
		cmd_arr[cur_cmd].val2 = 0;
	}
	cur_cmd = 0;
	nxt_cmd = 0;

}

void MotorControl_fine(int dutyL, int dutyR)
{
	//Left Motor - TIM_CHANNEL_1
	//Forward A1 = 1, A2 = 0
	//Reverse A1 = 0, A2 = 1

	//Right Motor - TIM_CHANNEL_2
	//Forward B1 = 1, B2 = 0
	//Reverse B1 = 0, B2 = 1

	//CLEAR pins then SET to prevent shorts through H bridge


//	if(CORRIDOR){
//		//for corridor settings
////		dutyL = dutyL*0.6;
////		dutyR = dutyR*0.6;
////
////		if(dutyL > -50 && dutyL<0) dutyL = -50;
////		if(dutyL < 50 && dutyL>0) dutyL = 50;
////
////		if(dutyR > -50 && dutyR<0) dutyR = -50;
////		if(dutyR < 50 && dutyR>0) dutyR = 50;
//		dutyL = dutyL*0.75;
//		dutyR = dutyR*0.75;
//
//		if(dutyL > -75 && dutyL<0) dutyL = -75;
//		if(dutyL < 75 && dutyL>0) dutyL = 75;
//
//		if(dutyR > -75 && dutyR<0) dutyR = -75;
//		if(dutyR < 75 && dutyR>0) dutyR = 75;
//		//end corridor settings
//	}

	if(dutyL > 1000) dutyL = 1000;
	if(dutyL < -1000) dutyL = -1000;

	if(dutyR > 1000) dutyR = 1000;
	if(dutyR < -1000) dutyR = -1000;

	if(dutyL >= 0){ //forward
		HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1_Pin, GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 7*dutyL);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2_Pin, GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, -7*dutyL);
	}

	if(dutyR >= 0){
		HAL_GPIO_WritePin(GPIOA, MOTOR_BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_BIN1_Pin, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 7*dutyR);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, MOTOR_BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOTOR_BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, -7*dutyR);
	}
}

void ServoControl(uint8_t pwm)
{
	//servo 50 left, 75 center, 105 right
	if(pwm>105) pwm = 105;
	if(pwm<50) pwm = 50;
	if(pwm == 75) pwm = 76;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
}

uint8_t MotorTurn(int16_t target, int16_t speed)
{
	static int16_t dampen;
	static uint8_t UART_count = 0;

	if(DEBUG_MSG && ((UART_count++)%2 == 0)){
		sprintf(UART_buffer, "gyro_val %03d cum %05d target %05d\r\n", gyro_val,cum_gyro,target);
		HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
	}

	if((EncodBDiff < 10 && EncodBDiff > -10)||(EncodADiff < 10 && EncodADiff > -10))
		power_correction+=10;
	else
		power_correction=0;

	if(target>0){ //left turn
		if(cum_gyro>target){ //stop
			MotorControl_fine(0,0);
			ServoControl(78);
			return 2;
		}
		else if(cum_gyro>(target-6600)){ //dampen last
			dampen = (target-cum_gyro)/20;
			if(dampen < 0) dampen = 0;

			if(speed>0) //forward
				MotorControl_fine(speed-330+dampen+power_correction,speed-330+dampen+power_correction);
			else //reverse
				MotorControl_fine(speed+330-dampen-power_correction,speed+330-dampen-power_correction);
		}
		else //running
			MotorControl_fine(speed,speed);
	}
	else{ //right turn
		if(cum_gyro<target){
			MotorControl_fine(0,0);
			ServoControl(70);
//			ServoControl(75);
			return 2;
		}
		else if(cum_gyro<(target+6600)){ //dampen last
			dampen = (cum_gyro-target)/20;
			if(dampen < 0) dampen = 0;

			if(speed>0) //forward
				MotorControl_fine(speed-330+dampen+power_correction,speed-330+dampen+power_correction);
			else //reverse
				MotorControl_fine(speed+330-dampen-power_correction,speed+330-dampen-power_correction);
		}
		else //running
			MotorControl_fine(speed,speed);
	}

	return 1;
}

//returns function to run
uint8_t* MovementInit(){

	switch(cmd_arr[cur_cmd].inst){
			case 0: //idle
				if (cmd_arr[cur_cmd].val2 == 1){ //emergency interrupt
					cum_gyro = 0;
					cum_enc = 0;
					ServoControl(75);
					MotorControl_fine(0,0);

					cmd_arr[cur_cmd].val2 = 0; //go to idle mode

					if(DEBUG_MSG){
						strcpy(&UART_buffer,"\r\n--reset cmd--\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
				}
				else{ // not emergency interrupt
//					cum_gyro = 0;
//					cum_enc = 0;
					ServoControl(75);
					MotorControl_fine(0,0);
				}
				return NULL;
				break;
			case 1: //straight movement
				//init vals
				cum_gyro = 0;
				cum_enc = 0;
				ServoControl(75);

				if(cmd_arr[cur_cmd].val2 == 1){ //forward
					cmd_arr[cur_cmd].val1 = 7433/100.0*cmd_arr[cur_cmd].val1;
				}
				else{
					cmd_arr[cur_cmd].val1 = -7850/100.0*cmd_arr[cur_cmd].val1;
				}

//				cmd_arr[cur_cmd].val1 = 7491/100.0*cmd_arr[cur_cmd].val1;
//				cmd_arr[cur_cmd].val1 *= cmd_arr[cur_cmd].val2; //invert if reverse

				if(DEBUG_MSG){
					sprintf(UART_buffer, "\r\n--cmd %d.%d: target %05d--\r\n", cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2,cmd_arr[cur_cmd].val1);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}

				//set function
				return &MotorStraight;
				break;
			case 2: //left turn
				//init vals
				cum_gyro = 0;
				ServoControl(55); //left turns

//				cmd_arr[cur_cmd].val1 = 3520*cmd_arr[cur_cmd].val1/180/2;
//				cmd_arr[cur_cmd].val1 = 3520*cmd_arr[cur_cmd].val1/180;
//				cmd_arr[cur_cmd].val1 = 16600*cmd_arr[cur_cmd].val1/90;
				//cmd_arr[cur_cmd].val1 = 17370*cmd_arr[cur_cmd].val1/90;
				//floor
				if(CORRIDOR){
					if(cmd_arr[cur_cmd].val2 == 1)//left front
						cmd_arr[cur_cmd].val1 = 17760*cmd_arr[cur_cmd].val1/90;
					else
						cmd_arr[cur_cmd].val1 = 18100*cmd_arr[cur_cmd].val1/90;
				}
				else
					cmd_arr[cur_cmd].val1 = 17845*cmd_arr[cur_cmd].val1/90; //lounge

				if(cmd_arr[cur_cmd].val2 == -1) //left back
					cmd_arr[cur_cmd].val1 *= -1;

				if(DEBUG_MSG){
					sprintf(UART_buffer, "\r\n--cmd %d.%d: target %d--\r\n", cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2,cmd_arr[cur_cmd].val1);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}

				return &MotorTurn;
				break;
			case 3: //right turn
				//init vals
				cum_gyro = 0;
				ServoControl(105); //right turns

//				cmd_arr[cur_cmd].val1 = 3520*cmd_arr[cur_cmd].val1/180/2;
//				cmd_arr[cur_cmd].val1 = 3520*cmd_arr[cur_cmd].val1/180;
//				cmd_arr[cur_cmd].val1 = 16600*cmd_arr[cur_cmd].val1/90;
				//cmd_arr[cur_cmd].val1 = 17370*cmd_arr[cur_cmd].val1/90;
				//floor
				if(CORRIDOR){
					if(cmd_arr[cur_cmd].val2 == 1)//right front
						cmd_arr[cur_cmd].val1 = 17750*cmd_arr[cur_cmd].val1/90; //corridor
					else
						cmd_arr[cur_cmd].val1 = 17883*cmd_arr[cur_cmd].val1/90;
				}
				else
					cmd_arr[cur_cmd].val1 = 17770*cmd_arr[cur_cmd].val1/90; //lounge
				if(cmd_arr[cur_cmd].val2 == 1) //right front
					cmd_arr[cur_cmd].val1 *= -1;

				if(DEBUG_MSG){
					sprintf(UART_buffer, "\r\n--cmd %d.%d: target %d--\r\n", cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2,cmd_arr[cur_cmd].val1);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}

				return &MotorTurn;
				break;
			case 4:
				cum_gyro = 0;
				cum_enc = 0;
				if(DEBUG_MSG){
					sprintf(&UART_buffer, "\r\n--cmd %d.%d: CalFunc--\r\n",cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}
//
				if(cmd_arr[cur_cmd].val1 == 0) //stationary cal
					return &CalFunc_Blank;
				else
					return &CalFunc;
				break;
			case 5:
				cum_gyro = 0;
				cum_enc = 0;
				if(DEBUG_MSG){
					sprintf(&UART_buffer, "\r\n--cmd %d.%d: TestFunc--\r\n",cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}

				return &TestFunc;
				break;
		}
	return NULL;
}


uint8_t MotorStraight(int16_t target, int16_t speed){
	static int16_t damping;
	static uint8_t UART_count = 0;
	static int16_t speedL,speedR;
	ServoControl(75);
	if(DEBUG_MSG && ((UART_count++) %2 == 0)){
		sprintf(UART_buffer, "gyro_val %03d cum %05d cum_enc %05d T %05d\r\n", gyro_val, cum_gyro, cum_enc, target);
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
	}

	if((EncodBDiff < 10 && EncodBDiff > -10)||(EncodADiff < 10 && EncodADiff > -10))
		power_correction+=10;
	else
		power_correction=0;

	if(target > 0){ //forward
		if(cum_enc > target){ //hit
			MotorControl_fine(0,0);
			ServoControl(75);
			power_correction=0;
			return 2;
		}
		else if(cum_enc > (target- 1750)){ //damping
			damping = (target - cum_enc) / 5;
			//if(damping < 0) damping = 0;
//			MotorControl_fine(speed - 330 + damping + cum_gyro/4, speed - 330 + damping - cum_gyro/4);
			speedL = speed - 330 + damping + cum_gyro/st_gy_gain + power_correction;
			speedR = speed - 330 + damping - cum_gyro/st_gy_gain + power_correction;
			if (speedL<0) speedL = 0;
			if (speedR<0) speedR = 0;

			MotorControl_fine(speedL, speedR);
		}
		else{ //running
//			MotorControl_fine(speed + cum_gyro/4 , speed - cum_gyro/4);
			speedL = speed + cum_gyro/st_gy_gain + power_correction;
			speedR = speed - cum_gyro/st_gy_gain + power_correction;
			if (speedL<0) speedL = 0;
			if (speedR<0) speedR = 0;
			MotorControl_fine(speedL, speedR);
		}
	}
	else { // backwards
		if(cum_enc < target){//stop
			MotorControl_fine(0,0);
			ServoControl(75);
			power_correction=0;
			return 2;
		}
		else if(cum_enc < (target+1750)){ //damping
			damping = (cum_enc - target) / 5;
//			if(damping < 0) damping = 0;
//			MotorControl_fine(speed + 330 - damping + cum_gyro/4, speed + 330 - damping - cum_gyro/4);

			speedL = speed + 330 - damping + cum_gyro/st_gy_gain - power_correction;
			speedR = speed + 330 - damping - cum_gyro/st_gy_gain - power_correction;
			if (speedL>0) speedL = 0;
			if (speedR>0) speedR = 0;
			MotorControl_fine(speedL, speedR);
		}
		else{ //running
//			MotorControl_fine(speed + cum_gyro/4, speed - cum_gyro/4);
			speedL = speed + cum_gyro/st_gy_gain - power_correction;
			speedR = speed - cum_gyro/st_gy_gain - power_correction;
			if (speedL>0) speedL = 0;
			if (speedR>0) speedR = 0;
			MotorControl_fine(speedL, speedR);
		}
	}

	return 1;
}


uint8_t CalFunc(int16_t val1, int16_t val2){
	//val2 stores state
	//val1 stores seek direction -> 1 forward,-1 backward
	//cal_correction stores detected dist - 20

	static uint8_t UART_count = 0;
//	static uint8_t state;
	const uint8_t dist = 20;

	if((EncodBDiff < 10 && EncodBDiff > -10)||(EncodADiff < 10 && EncodADiff > -10))
		power_correction+=10;
	else
		power_correction=0;

	if(DEBUG_MSG && (UART_count++)%2 == 0)
	{
		//sprintf(UART_buffer, "gyro_val %03d cum %05d\r\n", gyro_val, cum_gyro);
		sprintf(UART_buffer, "cmd %01d enc %05d side %05d\r\n",cmd_arr[cur_cmd].val2 ,cum_enc, irDist2);
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
	}

	MotorControl_fine(0,0);
	ServoControl(75);
	cum_enc = 0;
	cal_correction = 0;
	//initally move up 20 cm to seek block
	//if block alr found - reverse to edge (ONLY ANGLE CAL)
	//move until expose

	if(cmd_arr[cur_cmd].val2 == 0){ //seek block
		if(irDist2 > (dist+15)){ //not on block
			if(cmd_arr[cur_cmd].val1 == 1){ //seek forward
				if(cum_enc < 749*2.5){ //seek max 25cm forward
					MotorControl_fine(100+power_correction,100+power_correction);
				}
				else{ //fail to find block -> reverse
					cum_enc = 0;
					MotorControl_fine(0,0);
					ServoControl(75);
					pre_correction-=25; //reverse 25 cm
					cmd_arr[cur_cmd].val2 = 2; //fail state
				}
			}
			else if(cmd_arr[cur_cmd].val1 == -1){ //seek backward
				if(cum_enc > -749*2.5){ //seek max 25cm backward
					MotorControl_fine(-100-power_correction,-100-power_correction);
				}
				else{ //fail to find block -> reverse
					cum_enc = 0;
					MotorControl_fine(0,0);
					ServoControl(75);
					pre_correction-=25; //reverse 25 cm
					cmd_arr[cur_cmd].val2 = 2; //fail state
				}
			}
			else if(cmd_arr[cur_cmd].val1 == 0){ //no image alr fail
				cmd_arr[cur_cmd].val2 = 2; //fail state
			}
			else{ //weird input
				cmd_arr[cur_cmd].val2 = 2; //fail state
			}
		}
		else{//found
			MotorControl_fine(0,0);
			osDelay(100);
			MotorControl_fine(100+power_correction,100+power_correction);
			osDelay(10);
			cmd_arr[cur_cmd].val2 = 1;
//			cal_correction = irDist2-dist;
			cal_correction = 0;
			cum_enc = 0;
		}
	}
	else if(cmd_arr[cur_cmd].val2 == 1){ //search expose
		//seek up to 15cm forward
		if(irDist2 > (dist+15)){ //expose - Success
			MotorControl_fine(0,0);
			ServoControl(75);
			pre_correction-=10;
			if(DEBUG_MSG)
			{
				sprintf(UART_buffer, "Calibrate Success\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
			}

			return 2; //terminate sccessfully
		}
		else if(cum_enc < 749*20){ //seek max 20 cm forward
			MotorControl_fine(100+power_correction,100+power_correction);
		}
		else{ //fail to find
			cum_enc = 0;
			MotorControl_fine(0,0);
			ServoControl(75);
			pre_correction-=20; //reverse 20 cm -> back to original
			pre_correction-=5; //compensate for ir position
			cmd_arr[cur_cmd].val2 = 2; //fail state
		}
	}
	else{ //fail state
		if(DEBUG_MSG)
		{
			sprintf(UART_buffer, "cmd %01d enc %05d side %05d\r\n",cmd_arr[cur_cmd].val2 ,cum_enc, irDist2);
			HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
			sprintf(UART_buffer, "Calibrate Fail\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
		}

		return 2; //terminate
	}

	return 1;
}

uint8_t CalFunc_Blank(int16_t val1, int16_t val2){
	//val2 stores state
	//val1 stores seek direction -> 1 forward,-1 backward
	//cal_correction stores detected dist - 20

	//states
	//0 - seek front
	//1 - reverse back
	//2 - seek back (fail set offset)
	//3 - found->find edge
	//4 - found->reverse center
	//4 - fail

	static uint8_t UART_count = 0;
//	static uint8_t state;
	const uint8_t dist = 20;

	if((EncodBDiff < 10 && EncodBDiff > -10)||(EncodADiff < 10 && EncodADiff > -10))
		power_correction+=10;
	else
		power_correction=0;

	if(DEBUG_MSG && (UART_count++)%2 == 0)
	{
		//sprintf(UART_buffer, "gyro_val %03d cum %05d\r\n", gyro_val, cum_gyro);
		sprintf(UART_buffer, "cmd %01d enc %05d side %05d\r\n",cmd_arr[cur_cmd].val2 ,cum_enc, irDist2);
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
	}
//	MotorControl_fine(0,0);
//	ServoControl(75);
//	cum_enc = 0;
//	cal_correction = 0;

	if(cmd_arr[cur_cmd].val2 == 0){ //seek block - front
		if(irDist2 > (dist+15)){ //not on block
			if(cum_enc < 749*2.5){ //seek max 25cm forward
				MotorControl_fine(100+power_correction,100+power_correction);
			}
			else{ //fail to find block -> reverse
				cum_enc = 0;
				MotorControl_fine(0,0);
				ServoControl(75);
				//pre_correction-=25; //reverse 25 cm
				cmd_arr[cur_cmd].val2 = 1; //position return
			}
		}
		else{//found
			MotorControl_fine(0,0);
			osDelay(100);
			MotorControl_fine(100+power_correction,100+power_correction);
			osDelay(10);
			cmd_arr[cur_cmd].val2 = 3; //block found state
//			cal_correction = irDist2-dist;
			cal_correction = 0;
			cum_enc = 0;
		}
	}
	else if(cmd_arr[cur_cmd].val2 == 1){ //reverse
		if(cum_enc > -749*2.5){ //reverse 25 cm
			MotorControl_fine(-100-power_correction,-100-power_correction);
		}
		else{
			cum_enc = 0;
			MotorControl_fine(0,0);
			ServoControl(75);
			cmd_arr[cur_cmd].val2 = 2;
		}
	}
	else if(cmd_arr[cur_cmd].val2 == 2){ //seek block - back
		if(irDist2 > (dist+15)){ //not on block
			if(cum_enc > -749*2.5){ //seek max 25cm back
				MotorControl_fine(-100-power_correction,-100-power_correction);
			}
			else{ //fail to find block -> forward
				cum_enc = 0;
				MotorControl_fine(0,0);
				ServoControl(75);
				pre_correction+=25; //forward 25 cm
				cmd_arr[cur_cmd].val2 = 5; //fail state
			}
		}
		else{//found
			MotorControl_fine(0,0);
			osDelay(100);
			MotorControl_fine(100+power_correction,100+power_correction);
			osDelay(10);
			cmd_arr[cur_cmd].val2 = 3; //block found state
//			cal_correction = irDist2-dist;
			cal_correction = 0;
			cum_enc = 0;
		}
	}
	else if(cmd_arr[cur_cmd].val2 == 3){ //block found state
		//seek up to 15cm forward
		if(irDist2 > (dist+15)){ //expose - Success
			cum_enc = 0;
			MotorControl_fine(0,0);
			ServoControl(75);
			//pre_correction-=10;

			cmd_arr[cur_cmd].val2 = 4; //reverse center state
		}
		else if(cum_enc < 749*20){ //seek max 20 cm forward
			MotorControl_fine(100+power_correction,100+power_correction);
		}
		else{ //fail to find
			cum_enc = 0;
			MotorControl_fine(0,0);
			ServoControl(75);
			pre_correction+=20; //forward 20 cm -> back to original
			pre_correction-=5; //compensate for ir position
			cmd_arr[cur_cmd].val2 = 5; //fail state
		}
	}
	else if(cmd_arr[cur_cmd].val2 == 4){ //reverse to center
		if(cum_enc > -749){ //reverse 10 cm
			MotorControl_fine(-100-power_correction,-100-power_correction);
		}
		else{ //distacnce complete
			cum_enc = 0;
			MotorControl_fine(0,0);
			ServoControl(75);
			if(DEBUG_MSG)
			{
				sprintf(UART_buffer, "Calibrate Success\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
			}
			return 2; //terminate successfully
		}
	}
	else{ //fail state
		if(DEBUG_MSG)
		{
			sprintf(UART_buffer, "cmd %01d enc %05d side %05d\r\n",cmd_arr[cur_cmd].val2 ,cum_enc, irDist2);
			HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
			sprintf(UART_buffer, "Calibrate Fail\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
		}

		return 2; //terminate
	}

	return 1;
}

uint8_t TestFunc(int16_t val1, int16_t val2){
	static uint8_t UART_count = 0;

	if(DEBUG_MSG && (UART_count++)%2 == 0)
	{
		sprintf(&UART_buffer, "IR: %05d\r\n", irDist2);
//		sprintf(UART_buffer, "gyro_val %03d cum %05d\r\n", gyro_val, cum_gyro);
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
	}

	//Trial auto gyro cal - not working lol
	/*
	if(cmd_arr[cur_cmd].val2 == 0){
		sprintf(&UART_buffer, "Starting gyro cal\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
		cum_gyro = 0;
	}
	if(cmd_arr[cur_cmd].val2++ < 500){

	}
	else{
		gyro_offset -= 0.5*cum_gyro/500;
		sprintf(&UART_buffer, "Finished gyro cal, new val: %d\r\n",gyro_offset*1000);
		HAL_UART_Transmit(&huart3, (uint8_t *)&UART_buffer, strlen(UART_buffer), TIMEOUT);
		cum_gyro = 0;
		return 2;
	}
	*/
	return 1;
}

void ApplyCorrection(){
	if(pre_correction != 0){
		if(cmd_arr[nxt_cmd].inst != 1){ //if the previous cmd is not straight
			//insert new cmd
			nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
			cmd_arr[nxt_cmd].inst = 1;
			if(pre_correction > 0){
				cmd_arr[nxt_cmd].val1 = pre_correction;
				cmd_arr[nxt_cmd].val2 = 1;
			}
			else{
				cmd_arr[nxt_cmd].val1 = -pre_correction;
				cmd_arr[nxt_cmd].val2 = -1;
			}
		}
		else{
			if(cmd_arr[nxt_cmd].val2 == -1) pre_correction*=-1;
			cmd_arr[nxt_cmd].val1 += pre_correction;
			if (cmd_arr[nxt_cmd].val1 <0){
				cmd_arr[nxt_cmd].val1 *=-1;
				cmd_arr[nxt_cmd].val2 *=-1;
			}
		}
	}
	pre_correction = post_correction;
	post_correction = 0;
}

void delay (uint16_t duration)
{
	//Create a manual delay specifically for ADC-relevant task
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	//Run repeatedly till the counter for TIM4 is greater than the duration inputed
	while (__HAL_TIM_GET_COUNTER (&htim4) < duration);
}

void HAL_TIM_IC_CaptureCallBack(TIM_HandleTypeDef *htim) //For Ultrasonic
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // interrupt source is channel 2
	{
		uint8_t ch = 'B';
		if(DEBUG_MSG)
			HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
		if(Is_First_Captured == 0) //if the first value is not captured
		{
			IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //read the first value
			Is_First_Captured = 1;	// Toggle to true when captured
			// Capture falling edge polarity
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if(Is_First_Captured == 1)//if the first is already captured
		{
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//read second value
			__HAL_TIM_SET_COUNTER(htim, 0);	//resets counter

			if(IC_Value2 > IC_Value1)
			{
				ICValDiff = IC_Value2 - IC_Value1;
			}
			else if(IC_Value1 > IC_Value2)
			{
				ICValDiff = (0xffff - IC_Value1) + IC_Value2;
			}

			ICValDist = ICValDiff * .034/2;
			//End of adding calibration section
			Is_First_Captured = 0;	//reset to false state

			//Capture rising edge polarity
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			//Compare/Capture 2 DMA request
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);
		}
	}
}

void UltrasonicSensor_Read (void)
{
	//Model number: HCSR04SEN0001

	//uint8_t ch = 'A'; //for sending message to UART
	//HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	HAL_GPIO_WritePin(US_Output_GPIO_Port, US_Output_Pin, GPIO_PIN_SET); // Set the TRIG pin High
	delay(10);	//wait for 10 us
	HAL_GPIO_WritePin(US_Output_GPIO_Port, US_Output_Pin, GPIO_PIN_RESET); //Set the TRIG pin Low

	//Compare/Capture 2 Interrupt
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  UserInit();

  //Ultrasonic
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* creation of SonicSensorTask */
  SonicSensorTaskHandle = osThreadNew(SonicSensor, NULL, &SonicSensorTask_attributes);

  /* creation of IRSensorsTask */
  IRSensorsTaskHandle = osThreadNew(IRSensors, NULL, &IRSensorsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 319;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2_Pin|MOTOR_AIN1_Pin|MOTOR_BIN1_Pin|MOTOR_BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US_Output_GPIO_Port, US_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_AIN2_Pin MOTOR_AIN1_Pin */
  GPIO_InitStruct.Pin = MOTOR_AIN2_Pin|MOTOR_AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_BIN1_Pin MOTOR_BIN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_BIN1_Pin|MOTOR_BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_USER_Pin */
  GPIO_InitStruct.Pin = BTN_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US_Output_Pin */
  GPIO_InitStruct.Pin = US_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_Output_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *haurt)
{
	static uint8_t state = 0;
	uint8_t in;

	UNUSED(haurt);

	in = aRxBuffer[0];

	if(in == '?'){
		if(cmd_arr[cur_cmd].inst == 0){ //nothing running
			sprintf(&UART_buffer,"1\0",in);
		}
		else{
			sprintf(&UART_buffer,"0\0",in);
		}
		HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,1,TIMEOUT);
		HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,1);

		return;
	}

//	sprintf(&UART_buffer,"%d\n\r\0",aRxBuffer[0]);
//	HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);

	nxt_cmd %=CMD_ARR_SIZE;

	if(in == ' ') state = 0;

	switch(state){
		case 1:
			if(in >= '0' && in <= '9'){
				if(DEBUG_MSG){
					sprintf(&UART_buffer,"%c\0",in);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}

				cmd_arr[nxt_cmd].val1*=10;
				cmd_arr[nxt_cmd].val1+=in-'0';
				break;
			}
			//if non numerical input - flow through
		case 0:
			if(cmd_arr[cur_cmd].inst != 0)//if something is running
				cur_cmd = nxt_cmd;

			switch(in){
				case ' ': //emergency interrupt
					pre_correction = 0;
					post_correction = 0;
					cal_correction = 0;

					state = 0;
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 0; // instruction code to do what
					cmd_arr[nxt_cmd].val1 = 0; // Distance/Angle Value
					cmd_arr[nxt_cmd].val2 = 1; // Store state
					//cur_cmd++;
					cur_cmd = nxt_cmd;
					cur_cmd%=CMD_ARR_SIZE;
					break;
				case '/': //execute commands
//					ApplyCorrection();

					state = 0;
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 0;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 0;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nExecuting Commands\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}

					cur_cmd++;
					cur_cmd%=CMD_ARR_SIZE;
					break;
				case 'f': //forward
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 1;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nForward command\r\nEnter Encoder Val:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'b': //backward
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 1;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = -1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nReverse command\r\nEnter Encoder Val:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'l': //front left
					pre_correction += 3;
					post_correction += 0;
					if(cal_correction != 0){
						post_correction+= cal_correction;
						cal_correction=0;
					}
//					ApplyCorrection();

					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nLeft front command\r\nEnter Turn Degree:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'v': //back left
					pre_correction += 0;
					post_correction += -6;
					if(cal_correction != 0){
						post_correction-= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = -1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nLeft back command\r\nEnter Turn Degree:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'r': //front right
					pre_correction += 3;
					post_correction += -3;
					if(cal_correction != 0){
						post_correction-= cal_correction;
						cal_correction=0;
					}
					//ApplyCorrection();

					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nRight front command\r\nEnter Turn Degree:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'n': //back right
					pre_correction += 3;
					post_correction += -3;
					if(cal_correction != 0){
						post_correction+= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = -1;
					state = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nRight back command\r\nEnter Turn Degree:");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 't': //test func
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 5;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 0;
					state = 0;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nTest Command");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'a': //spot turn left front - 45 left forward then 45 right reverse
					pre_correction += -5;
					post_correction += 6;
					if(cal_correction != 0){
						post_correction+= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					//front left
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 45;
					cmd_arr[nxt_cmd].val2 = 1;

					//back right
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					cmd_arr[nxt_cmd].val1 = 44;
					cmd_arr[nxt_cmd].val2 = -1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nSpot Turn Left Front\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}

					state = 0;
					break;
				case 'd': //spot turn right front
					pre_correction += -6;
					post_correction += 5;
					if(cal_correction != 0){
						post_correction-= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					//front right
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					if(CORRIDOR)
						cmd_arr[nxt_cmd].val1 = 45;
					else
						cmd_arr[nxt_cmd].val1 = 44;
					cmd_arr[nxt_cmd].val2 = 1;

					//back left
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 45;
					cmd_arr[nxt_cmd].val2 = -1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nSpot Turn Right Front\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}

					state = 0;
					break;
				case 'A': //spot turn left back
					pre_correction += 23;
					post_correction += -22;
					if(cal_correction != 0){
						post_correction+= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					//back right
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					cmd_arr[nxt_cmd].val1 = 45;
					cmd_arr[nxt_cmd].val2 = -1;

					//front left
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 44;
					cmd_arr[nxt_cmd].val2 = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nSpot Turn Left Back\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}

					state = 0;
					break;
				case 'D': //spot turn right back
//					pre_correction += 1;
//					post_correction += -2;
					pre_correction += 21;
					post_correction += -22;
					if(cal_correction != 0){
						post_correction-= cal_correction;
						cal_correction=0;
					}
					ApplyCorrection();

					//back left
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 2;
					cmd_arr[nxt_cmd].val1 = 45;
					cmd_arr[nxt_cmd].val2 = -1;

					//front right
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 3;
					cmd_arr[nxt_cmd].val1 = 44;
					cmd_arr[nxt_cmd].val2 = 1;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nSpot Turn Right Back\r\n");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}

					state = 0;
					break;
				case 'z': //calibrate forward
					//val1 stores seek direction -> 1 forward,-1 back
					//val2 stores state
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 4;
					cmd_arr[nxt_cmd].val1 = 1;
					cmd_arr[nxt_cmd].val2 = 0;
					state = 0;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nForward Calibrate Command");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'x': //calibrate forward
					//val1 stores seek direction -> 1 forward,-1 back
					//val2 stores state
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 4;
					cmd_arr[nxt_cmd].val1 = 0;
					cmd_arr[nxt_cmd].val2 = 0;
					state = 0;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nStationary Calibrate Command");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
				case 'c': //calibrate backward
					//val1 stores seek direction -> 1 forward,-1 back
					//val2 stores state
					nxt_cmd = (nxt_cmd+1)%CMD_ARR_SIZE;
					cmd_arr[nxt_cmd].inst = 4;
					cmd_arr[nxt_cmd].val1 = -1;
					cmd_arr[nxt_cmd].val2 = 0;
					state = 0;

					if(DEBUG_MSG){
						strcpy(&UART_buffer, "\r\nBackward Calibrate Command");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					break;
			} //end instruction switch case
	}

	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t buffer[10];

	uint8_t line1[20];
	uint8_t line2[20];
	uint8_t line3[20];
	uint8_t line4[30];
	uint8_t line5[20];
//	uint8_t line6[20];

	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 100;

	xLastWakeTime = osKernelGetTickCount();

	osDelay(1000);

  /* Infinite loop */
  for(;;)
  {
//	  sprintf(line1, "cum_enc %08d\0", cum_enc);
//	  OLED_ShowString(0,0,line1);

//	  //read gyro
//	  HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x37, 1, (uint8_t *)&buffer, 2, TIMEOUT);
//	  osDelay(1);
//	  gyro_val = (int16_t)(buffer[0]<<8|buffer[1]);
////	  gyro_val = round(gyro_val/131 + 0.3);
//	  gyro_val = round((gyro_val + 0.326*131)/13.1);
//	  sprintf(&line2, "Gyro: %5d\0", gyro_val);
//	  OLED_ShowString(0,10,line2);

//	  cum_gyro += gyro_val;
//	  sprintf(&line3, "cum_gy:%07d\0", cum_gyro);
//	  OLED_ShowString(0,20,line3);

//	  sprintf(&line4, "cmd %d.%02d c%02d n%02d\0", cmd_arr[cur_cmd].inst,cmd_arr[cur_cmd].val2, cur_cmd, nxt_cmd);
//	  OLED_ShowString(0,30,line4);

//	  sprintf(&line5, "1 %03d %03d 2 %03d\0", pre_correction, post_correction, cal_correction);
//	  OLED_ShowString(0,40,line5);


	  OLED_Refresh_Gram();
	  xLastWakeTime += xFrequency;
	  osDelayUntil(xLastWakeTime);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 50;

	osDelay(1000);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

	EncodAPrev = EncodACurrent = __HAL_TIM_GET_COUNTER(&htim2);
	EncodBPrev = EncodBCurrent = __HAL_TIM_GET_COUNTER(&htim3);

	xLastWakeTime = osKernelGetTickCount();
	for(;;){
		EncodACurrent = __HAL_TIM_GET_COUNTER(&htim2);
		EncodBCurrent = __HAL_TIM_GET_COUNTER(&htim3);

		EncodADiff = EncodAPrev - EncodACurrent;
		EncodBDiff = EncodBCurrent - EncodBPrev;

		EncodAPrev = EncodACurrent;
		EncodBPrev = EncodBCurrent;

		if(EncodBDiff>60000)EncodBDiff-=65535;
		if(EncodBDiff<-60000)EncodBDiff+=65535;


		if(EncodBDiff<EncodADiff && EncodBDiff>0) cum_enc += EncodBDiff;
		else if(EncodADiff<EncodBDiff && EncodADiff>0) cum_enc += EncodADiff;
		else cum_enc += EncodBDiff;

//		if(HAL_GPIO_ReadPin(GPIOA, MOTOR_AIN1_Pin) == GPIO_PIN_SET){ //if set to forward
//			cum_enc += EncodBDiff;
////			if(EncodBDiff < 0) cum_enc += 65535;
//		}
//		if(HAL_GPIO_ReadPin(GPIOA, MOTOR_AIN2_Pin) == GPIO_PIN_SET){ // if set to reserve
//			cum_enc += EncodBDiff;
////			if(EncodBDiff > 0) cum_enc -= 65535;
//		}



		xLastWakeTime += xFrequency;
		osDelayUntil(xLastWakeTime);
	}

  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
	uint32_t xLastWakeTime;
	uint8_t running = 0;
	const uint32_t xFrequency = 50;

	uint8_t (*func_ptr)(int16_t, int16_t);
	func_ptr = NULL;

	osDelay(1000);

	xLastWakeTime = osKernelGetTickCount();
  /* Infinite loop */

  for(;;)
  {
	  switch(running){
			case 0: //init
				func_ptr = MovementInit();
				if (func_ptr){
					running = 1;
					power_correction = 0;

					xLastWakeTime+=500; //delay
					osDelayUntil(xLastWakeTime);
				}
				else running = 0;
				break;
			case 1: //loop running
				if(cmd_arr[cur_cmd].inst == 0) running = 0;
				else
					running = (*func_ptr)(cmd_arr[cur_cmd].val1,cmd_arr[cur_cmd].val2*DEF_SPEED);
				break;
			case 2: //finally
				MotorControl_fine(0,0);

				cur_cmd++; //next command
				cur_cmd%=CMD_ARR_SIZE;

				if(DEBUG_MSG){
					sprintf(UART_buffer, "--cmd complete gyro %05d enc %05d--\r\n",cum_gyro,cum_enc);
					HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
				}
				if(cmd_arr[cur_cmd].inst == 0){
					if(DEBUG_MSG){
						sprintf(UART_buffer, "--cmd set complete--\0");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);
					}
					else{
						sprintf(UART_buffer, "1\0");
						HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,1,TIMEOUT);
					}

				}
				xLastWakeTime = osKernelGetTickCount();
//				osDelay(500); //delay
				xLastWakeTime+=500;

				running = 0;
				break;
		}

		xLastWakeTime+=xFrequency;
		osDelayUntil(xLastWakeTime);
	}
  /* USER CODE END StartUARTTask */
}

/* USER CODE BEGIN Header_SonicSensor */
/**
* @brief Function implementing the SonicSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SonicSensor */
void SonicSensor(void *argument)
{
  /* USER CODE BEGIN SonicSensor */
  /* Infinite loop */

	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 50;

//	uint8_t line2[20];
//	uint8_t line3[20];
	uint8_t buffer[20];
	osDelay(1000);

	xLastWakeTime = osKernelGetTickCount();

	for(;;)
	{
		//UltrasonicSensor_Read();
		//read gyro
		HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x37, 1, (uint8_t *)&buffer, 2, TIMEOUT);
		osDelay(5);
		gyro_val = (int16_t)(buffer[0]<<8|buffer[1]);
		//	  gyro_val = round(gyro_val/131 + 0.3);
		gyro_val = round((gyro_val + gyro_offset*131)/13.1);
		//	  sprintf(&line2, "Gyro: %5d\0", gyro_val);
		//	  OLED_ShowString(0,10,line2);
		//
		cum_gyro += gyro_val;
		//	  sprintf(&line3, "cum_gyro: %5d\0", cum_gyro);
		//	  OLED_ShowString(0,20,line3);

		xLastWakeTime+=xFrequency;
		osDelayUntil(xLastWakeTime);
	}
  /* USER CODE END SonicSensor */
}

/* USER CODE BEGIN Header_IRSensors */
/**
* @brief Function implementing the IRSensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRSensors */
void IRSensors(void *argument)
{
  /* USER CODE BEGIN IRSensors */
	uint8_t display[20];
	uint8_t count = 0;
	uint32_t sum = 0;
	osDelay(1000);
  /* Infinite loop */
  for(;;)
  {

	//ADC1 = PC1 = front IR Sensor
	//Enables the ADC and start the conversion of the regular channels.
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10); //Timeout = 10
	ADC_VAL1 = HAL_ADC_GetValue(&hadc1);
	//Disables the ADC and ends the conversion
	HAL_ADC_Stop(&hadc1);


	//ADC2 = PC2 = side IR Sensor
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	ADC_VAL2 = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	/*Sort out the voltage and irDistance values
	*IRSensor model: GP2Y0A21YK
	*formula source: guillaume-rico/SharpIR
	*ADC Battery calculation: 12bits = 0 to 4095
	*With Vref = 5V
	*General formula: Vmeasure = ADCVal * VRef / 4095
	*Legend on Orientation: >> : side IR ,  > : front IR , | -- : robot general position
	*/

	//Front IR Sensor
//	voltage1 = (float) (ADC_VAL1*5) / 4095;
/*
	//Corridor between SPL and ABN												^
	// with side IR facing ABN													^
	// Reading:	10	22	31	34	35	37	35										-->
	// Actual:	10	20	30	35	40	45	50										SPL
	//irDist1 = roundf(49 * pow(voltage1 , -1.2));

	//Corridor between SPL and ABN  											|>>
	// Reading:	10	21	31	34	35	38	40					 			 		v
	// Actual:	10	20	30	35	40	45	50										SPL
	//irDist1 = roundf(48.5 * pow(voltage1, -1.2));

	//Corridor between SPL and ABN												< --
	// with side IR facing SPL													   v
 	// Reading:	10	21	31	35	37	38	38										   v
	// Actual:	10	20	30	35	40	45	50										  SPL
	//irDist1 = roundf(48 * pow(voltage1 , -1.2));

	//Corridor between SPL and ABN												   ^
	// Reading:	10	21	31	35	36	37	37										<< |
	// Actual:	10	20	30	35	40	45	50										  SPL
	//irDist1 = roundf(48 * pow(voltage1, -1.2));
*/

	//For Assessment at Corridor between SPL and ABN CLear weather no rain no sunlight on field
	// Calibrated suitable between 8.30am and 11am
//	irDist1 = roundf(48.375 * pow(voltage1 , -1.2));

/* -------------------------------------------------
	//Corridor between SPL and ABN 4.30pm drizzle with side IR facing SPL   -->
		// Reading:	9	19	31	36	43	48	52								 v
		 // Actual:	10	20	30	35	40	45	50								 v
		  //																SPL
	//irDist1 = roundf(42.5 * pow(voltage1 , -1.2));

	// SPL (Back)											win		< --
		// Reading: 9	19	30	35	40	46	50	54	61		dow		  v
		// Actual: 	10 	20 	30	35	40	45	50	55	60				  v
		//														back of lab
	//irDist1 = roundf(42.5 * pow(voltage1 , -1.2));
*/

	// Side IR Sensor
//	voltage2 = (float) (ADC_VAL2*5) / 4095;
/*
	//Corridor between SPL and ABN with Front IR facing SPL 	|>>
	// Reading:	9	20	30	35	37	37	36						v
	// 	Actual: 10 	20 	30	35	40	45	50						SPL
	//irDist2 = roundf(46 * pow(voltage2, -1.2));

	//Corridor between SPL and ABN								^
	//															^
	// Reading:	10	20	31	35	38	38	37						->
	//  Actual: 10 	20 	30	35	40	45	50					 	SPL
	//irDist2 = roundf(45 * pow(voltage2 , -1.2));

	//Corridor between SPL and ABN
	// With Front IR facing towards ABN 					  	^
	// Reading:	10	20	31	35	39	39	37					  <<|
	// 	Actual: 10 	20 	30	35	40	45	50					 	SPL
	//irDist2 = roundf(45 * pow(voltage2 , -1.18));

	//Corridor between SPL and ABN							<--
	//														  v
	// Reading:	10	20	31	35	38	38	35					  v
	// 	Actual: 10 	20 	30	35	40	45	50					 SPL
	//
*/
//	irDist2 = roundf(45 * pow(voltage2, -1.2));


	//For Assessment at Corridor between SPL and ABN Clear weather no rain no sunlight on field
	// Calibrated suitable between 8.30am and 11am
	//irDist2 = roundf(45.25 * pow(voltage2, -1.195));
/* -------------------------------------------------------
	// SPL (Back)												win		  ^
		// Reading: 10 	21	31	36	39	41	44					dow		<<|
		// 	Actual: 10 	20 	30	35	40	45	50						back of lab
	//irDist2 = roundf(45 * pow(voltage2, -1.18));

	// SPL (front)													  board
		// Reading: 9 	20	30	34	40	45	49	54	67	74		win	  	^
		// Actual: 10 	20 	30	35	40	45	50	55	60	65		dow	  <<|
	//irDist2 = roundf(45 * pow(voltage2 , -1.2));
*/

//	sprintf(&display, "f:%05d s:%05d\0", irDist1,ADC_VAL2);

	//SPL
	//irDist2 = roundf( 84.6931 + 543.061 / (1+pow (ADC_VAL2/1391.588, 4.613522) ));
	//SCSE Lounge
//	irDist2 = roundf( 68.67086 + (111129100-6.867086)/(1+pow(ADC_VAL2/1.795939,2.326289)));

	//	irDist2 = 0;
	sprintf(&display, "Front:%05d\0", ADC_VAL1);
	OLED_ShowString(0,10, display);
	sprintf(&display, "Left:%05d\0", ADC_VAL2);
	OLED_ShowString(0,20, display);

	sum+=ADC_VAL2;
	if((++count)%2 == 0){
//		irDist2 = round( 6.867086 + (111129100-6.867086)/(1+pow(sum/count/1.795939,2.326289))*10);
//		sprintf(&display, "si:%05d %05d\0", sum/count,irDist2);
		irDist2 = round(4.513446 + (106.251-4.513446)/(1+pow(sum/count/979.5944,2.68463))); //lounge
//		irDist2 = round(6.634305+(52.05891-6.634305)/(1+pow(sum/count/1368.686,3.334143))); //corridor
//		sprintf(UART_buffer, "si:%05d %010d\r\n",sum/count,irDist2);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&UART_buffer,strlen(UART_buffer),TIMEOUT);


//		sprintf(&display, "IR: %05d\0", temp);
//		OLED_ShowString(0,50, display);
		count = 0;
		sum = 0;
	}

    osDelay(50);
  }
  /* USER CODE END IRSensors */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

