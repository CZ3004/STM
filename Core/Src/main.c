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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Show */
osThreadId_t ShowHandle;
const osThreadAttr_t Show_attributes = {
  .name = "Show",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorMove */
osThreadId_t MotorMoveHandle;
const osThreadAttr_t MotorMove_attributes = {
  .name = "MotorMove",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Encoder */
osThreadId_t EncoderHandle;
const osThreadAttr_t Encoder_attributes = {
  .name = "Encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRSensor */
osThreadId_t IRSensorHandle;
const osThreadAttr_t IRSensor_attributes = {
  .name = "IRSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void showTask(void *argument);
void motormovetask(void *argument);
void encodertask(void *argument);
void irsensortask(void *argument);
void servoTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[20] = {0};
char tempBuffer[4];
char moveQueue[8000][4]; // can store up to 8000 movements
int Back = -1;
int Front = -1;
int flagStart = 1;

// Motor States
int Rstate = 0;
int Lstate = 0;
int MotorReady = 0;
int wheelRunning = 0;
// Gyro
#define PI 3.1416
#define ICM_ADR (0x68 << 1)
#define AK_ADR (0x0C << 1)
#define TIMEOUT 100
int16_t gyro_val = 0;
int16_t cum_gyro = 0;
// Motor Speed
int cntA1, cntA2, speedA;
int cntB1, cntB2, speedB;
// Motor Movements
const uint8_t MID = 147;
uint8_t strFlag = -1;
uint16_t pwmValA = 0; // Motor A pwmVal
uint16_t pwmValB = 0; // Motor B pwmVal
uint16_t TargetPwmValA = 0; // Motor A pwmVal
uint16_t TargetPwmValB = 0; // Motor B pwmVal
uint8_t direction = 147;
uint32_t diff = 0;

uint8_t indexRx = 0; // index value
uint8_t data_Rx = 0; // uart data

void ICM_Init(void)
	{
		uint8_t reg;

		//init ICM
		//turn on
		HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x06, 1, &reg, 1, TIMEOUT);
		reg = (reg & ~0x47) | 0x01; //Clear Sleep mode and set timer to PLL
		HAL_I2C_Mem_Write(&hi2c1, ICM_ADR, 0x06, 1, &reg, 1, TIMEOUT);
		//enable bypass mode
		HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x0F, 1, &reg, 1, TIMEOUT);
		reg = reg|0x02; //set bypass mode
		HAL_I2C_Mem_Write(&hi2c1, ICM_ADR, 0x0F, 1, &reg, 1, TIMEOUT);

		//init AK
		//reset chip
		reg = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, AK_ADR, 0x32, 1, &reg, 1, TIMEOUT); //Reset Compass
		osDelay(50);
		//timer mode
		reg = 0x08; //Set continuous measurement mode 4 (100Hz refresh)
		HAL_I2C_Mem_Write(&hi2c1, AK_ADR, 0x31, 1, &reg, 1, TIMEOUT);
	}
	void enqueue(char item[]) // get char inputs
	{
		if(Back == 8000 -1)
		{
			printf("Overflow\n");
		}
		else
		{
			if(Front == -1) // means no item
			{
				Front = 0; // means got item
			}
			Back = Back + 1; // add to queue
			strncpy(moveQueue[Back],item,4);
		}
	}

	char* dequeue()
	{
		if(Front == -1 || Front > Back) // if Front has no item or is higher than back means its not detecting the item
		{
			printf("Underflow\n");
			return;
		}
		else
		{
			Front = Front + 1;
			return moveQueue[Front - 1];
		}
	}

	int conCatValue(char dequeuedVal[])
	{
		if(!(dequeuedVal[1] >= '0' && dequeuedVal[1] <= '9')) // if not number
		{
			return 0;
		}
		return (dequeuedVal[1]-'0')*100+(dequeuedVal[2]-'0')*10+(dequeuedVal[3]-'0');
	}
	int encoderDiff(int diff)
	{
		int pwm = 0.711302*(diff) + 100.615; // doesnt work below 300
		return pwm;
	}
	void set_speed(char x) // 1: slow, 2: medium, 3: fast
	{
		switch(x){
			case '1':
				TargetPwmValA = 1500;
				TargetPwmValB = 1500;
				pwmValA = 1500; // we put higher as in actual runs, the pwm might start abit lower
				pwmValB = 1500; //
				break;
			case '2':
				TargetPwmValA = 2000;
				TargetPwmValB = 2000;
				pwmValA = 2000; // Motor A pwmVal, shows 2675 +-6 on encoder. So about 2.66 encoder per 1 pwmVal
				pwmValB = 2000; // Motor B pwmVal/ shows
				break;
			case '3':
				TargetPwmValA = 3000;
				TargetPwmValB = 3000;
				pwmValA = 3000; // Motor A pwmVal 4055 +- 6
				pwmValB = 3000; // Motor B pwmVal
				break;
			case '4':
				TargetPwmValA = 4000;
				TargetPwmValB = 4000;
				pwmValA = 4000; // Motor A pwmVal 4055 +- 6
				pwmValB = 4000; // Motor B pwmVal
				break;
			default:
				pwmValA = 0; // Motor A pwmVal
				pwmValB = 0; // Motor B pwmVal
				break;
		}
	}
	void wheel_stop(void)
	{
		pwmValA = 0;
		pwmValB = 0;
		Rstate = 0; // motor has stopped moving
		Lstate = 0; // motor has stopped moving
		wheelRunning= 0;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValB);
	}

	void set_Backward(void)
	{
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin,GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin,GPIO_PIN_RESET);
	}
	void set_Forward(void)
	{
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin,GPIO_PIN_SET);

		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin,GPIO_PIN_SET);
	}
	void turn_right(int angle) // 90 degrees
	{
		wheelRunning= 1;
		// to do
		//int time = 12400;
		//htim1.Instance->CCR4 = MID; // center
		//osDelay(1000);
		//uint8_t hello[15] = "         \0";
		int time = convertRTurn_to_Time((double)angle);
		strFlag = 0;
//		set_speed('1');
		set_Forward(); // going forward
		direction = 180;
		htim1.Instance->CCR4 = direction;
		//		//htim1.Instance->CCR4 = 110; // extreme left 120 is safe, 100 is the extreme. so aim 120-100
		//		//htim1.Instance->CCR4 = 145; // pretty much the center
		//		//htim1.Instance->CCR4 = 200; // 190 is good, 200 is the furthest i will go

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);

		osDelay(time); // run 1 second

//		sprintf(hello,"R90X30Y41\0",angle);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
		wheel_stop();
//		sprintf(hello,"R90X30Y41\n\r\0",angle);

		//osDelay(4000);
	}
	void back_right(int angle) // 90 degrees
	{
		wheelRunning= 1;
			// to do
			//int time = 12400;
			//htim1.Instance->CCR4 = MID; // center
			//osDelay(1000);
			//uint8_t hello[15] = "         \0";
			int time = convertRTurn_to_Time((double)angle);
			strFlag = 0;
	//		set_speed('1');
			set_Backward(); // going forward
			direction = 180;
			htim1.Instance->CCR4 = direction;
			//		//htim1.Instance->CCR4 = 110; // extreme left 120 is safe, 100 is the extreme. so aim 120-100
			//		//htim1.Instance->CCR4 = 145; // pretty much the center
			//		//htim1.Instance->CCR4 = 200; // 190 is good, 200 is the furthest i will go

			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);

			osDelay(time); // run 1 second
//			sprintf(hello,"R90X30Y41\0",angle);
//			HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
			direction = MID;
			htim1.Instance->CCR4 = direction; // center
			wheel_stop();
			//sprintf(hello,"R90X30Y41\n\r\0",angle);

			//osDelay(4000);
	}
	void turn_left(int angle) // 90 degrees
	{
		wheelRunning= 1;
		//uint8_t hello[15] = "         \0";
		int time = convertLTurn_to_Time((double)(angle+2));
		// to do
		//int time = 6000;
		//htim1.Instance->CCR4 = MID; // center
		//osDelay(1000);
		strFlag = 0;
//		set_speed('1');
		set_Forward(); // going forward
		direction = 121;
		htim1.Instance->CCR4 = direction;
		set_Forward(); // going forward
		//		//htim1.Instance->CCR4 = 110; // extreme left 120 is safe, 100 is the extreme. so aim 120-100
		//		//htim1.Instance->CCR4 = 145; // pretty much the center
		//		//htim1.Instance->CCR4 = 200; // 190 is good, 200 is the furthest i will go

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1100);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2100);
		osDelay(time); // run 1 second
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
//		if(angle == 90)
//		{
////			sprintf(hello,"L90X30Y41\n\r\0"); // consult with algo people on which spot to measure x
//			sprintf(hello,"L90X30Y41\0"); // consult with algo people on which spot to measure x
//		}
//		else
//		{
//			sprintf(hello,"L%d\0",angle); // consult with algo people on which spot to measure x
//		}
//		HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
		wheel_stop();
		//osDelay(4000);
	}
	void back_left(int angle) // 90 degrees
	//void turn_left()
	{
		wheelRunning= 1;
		//uint8_t hello[15] = "         \0";
		int time = convertLTurn_to_Time((double)(angle+2)) - 100; // -100 for spl lab ground
		// to do
		//int time = 6000;
		//htim1.Instance->CCR4 = MID; // center
		//osDelay(1000);
		set_Backward(); // going forward
		strFlag = 0;
//		set_speed('1');
		direction = 121;
		htim1.Instance->CCR4 = direction;
		//		//htim1.Instance->CCR4 = 110; // extreme left 120 is safe, 100 is the extreme. so aim 120-100
		//		//htim1.Instance->CCR4 = 145; // pretty much the center
		//		//htim1.Instance->CCR4 = 200; // 190 is good, 200 is the furthest i will go

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1100);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2100);
		osDelay(time); // run 1 second
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
//		if(angle == 90)
//		{
////			sprintf(hello,"L90X30Y41\n\r\0"); // consult with algo people on which spot to measure x
//			sprintf(hello,"L90X30Y41\0"); // consult with algo people on which spot to measure x
//		}
//		else
//		{
//			sprintf(hello,"L%d\0",angle); // consult with algo people on which spot to measure x
//		}
//		HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
		wheel_stop();
		//osDelay(4000);
	}
	void wheelMoveForward(int dist)
	{
		int time = convertDist_to_Time(dist+1); // actual code
		//int time = dist; // testing
		//uint8_t hello[15] = "         \0";
		//int distance = convertTime_to_Dist((double)time); // double check
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
		wheelRunning= 1;
		//osDelay(5000);
		strFlag = 1;
		if(flagStart == 0)
		{
			osDelay(1000);
			flagStart = 2;
		}
		set_speed('2');
		set_Forward();
		cum_gyro = 0;
		while(time > 0)
		{
			// only works for speed 1 the wheel corrections

			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

			  if(time > 1000) // if still more than 0.5 second remaining to move
			  {
				  osDelay(1000); // run 1 second
				  time = time - 1000; // reduce time by 1 second
			  }
			  else
			  {
				  osDelay(time); // run the remaining time
				  time = 0; // time run out
			  }
		}
		wheel_stop();
		strFlag = 0;
		//osDelay(1000);
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
		cum_gyro = 0; // reset gyro
		//sprintf(hello,"F%d\n\r\0",dist);

		//osDelay(5000); // for checking purposes
	}
	void wheelMoveBackward(int dist)
	{
		wheelRunning= 1;
		int time = convertDist_to_Time(dist);
		//uint8_t hello[15] = "         \0";
//		int distance = convertTime_to_Dist((double)time); // double check
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
		//osDelay(1000);
		//strFlag = 1;
		set_speed('2');
		set_Backward();
		while(time > 0)
		{
			// only works for speed 1 the wheel corrections

			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

			if(time > 1000) // if still more than 0.5 second remaining to move
			{
				osDelay(1000); // run 1 second
				time = time - 1000; // reduce time by 1 second
			}
			else
			{
				osDelay(time); // run the remaining time
				time = 0; // time run out
			}


		}
//		sprintf(hello,"B%d\0",dist);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
		wheel_stop();
		strFlag = 0;
		//osDelay(1000);
		direction = MID;
		htim1.Instance->CCR4 = direction; // center
		//sprintf(hello,"B%d\n\r\0",distance);
		//osDelay(5000); // for checking purposes
	}
	void adjustSpeed(int speedA, int speedB)
	{
		// only ajust after a set period handled by encoder
		if(speedA < TargetPwmValA - 10)
		{
			//pwmValA = TargetPwmValA + (TargetPwmValA - speedA);
			pwmValA = TargetPwmValA + 20;
		}
		else if(speedA > TargetPwmValA + 20)
		{
			//pwmValA = TargetPwmValA - (speedA -TargetPwmValA);
			pwmValA = TargetPwmValA - 20;

		}

		if(speedB < TargetPwmValB - 10)
		{
			//pwmValB = TargetPwmValB + (TargetPwmValB - speedB);
			pwmValB = TargetPwmValB + 20;

		}
		else if(speedB > TargetPwmValB + 20)
		{
			//pwmValB = TargetPwmValB - (speedB -TargetPwmValB);
			pwmValB = TargetPwmValB - 20;

		}
//			diff = speedA - speedB;
//			// adjust speed if moving straight, here will add a if case if robot is strictly moving straight
//			if(diff < -5)
//			{
//				pwmValB = pwmValB + diff;
//			}
//			else if(diff > 5)
//			{
//				pwmValB = pwmValB + diff;
//			}



	}

	// for checklists
	int convertDist_to_Time(int dist)
	{
		if(dist == 0)
		{
			return 0;
		}
		//int y = (0.0593*dist - 0.0399) * 1000; // pwm 1000
		int y = (-8.12*pow(10,-7)*pow(dist,2) + 2.94*pow(10,-2)*(dist) + 1.48*pow(10,-2)) * 1000;

		return y;

	}
	int convertTime_to_Dist(double time) // mainly for straight
	{
		//double timu = (double)time;
		if(time == 0)
		{
			return 0;
		}
		double y = roundf((time/1000 + 0.0399)/0.0593);
		return y;

	}
	int convertLTurn_to_Time(double angle)
	{
		double y;
		y= (-0.00000246*pow(angle,2)) + (0.0312*(angle)) + 0.143;
		int time = y*1000;
		if(angle == 360)
		{
			time = time + 300;
		}
		else if(angle == 270)
		{
			time = time + 150;
		}
		return time;
	}
	int convertRTurn_to_Time(double angle)
	{
		double y = -3.82*pow(10,-6)*pow(angle,2) + 3.59*pow(10,-2)*angle - 0.0784;
		int time = y * 1000;
		if (angle >= 270)
		{
			time = time - 200; // +300 if HWlab2,
		}
		if(time >= 300)
		{
			time = time +200;
		}
//		else if(angle > 270)
//		{
//			time = time + 400; // probably +200 if rocky terrain
//		}

		//360 undershot if -200
		return time;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init(); // initialize the OLED
  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,1); // maybe change this to 1 for single char
  //HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,4); // actual string 4 char
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  ICM_Init();
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

  /* creation of Show */
  ShowHandle = osThreadNew(showTask, NULL, &Show_attributes);

  /* creation of MotorMove */
  MotorMoveHandle = osThreadNew(motormovetask, NULL, &MotorMove_attributes);

  /* creation of Encoder */
  EncoderHandle = osThreadNew(encodertask, NULL, &Encoder_attributes);

  /* creation of IRSensor */
  IRSensorHandle = osThreadNew(irsensortask, NULL, &IRSensor_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(servoTask, NULL, &ServoTask_attributes);

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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  hi2c1.Init.ClockSpeed = 100000;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim8.Init.Period = 7199;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RES_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // work on clearing the message afterwards
{
	//prevent unused arguement(s) compilation warning
	//new method using queues
	UNUSED(huart);
	//HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 20, 0xFFFF);
	if (flagStart == 1)
	{
		flagStart = 0; // we got a command
	}
//	if(tempBuffer[0] == 'f' || tempBuffer[0] == 'r' || tempBuffer[0] == 'l' || tempBuffer[0] == 'b' || tempBuffer[0] == 'w' || tempBuffer[0] == 'v' || tempBuffer[0] == 'n' || tempBuffer[0] == 's' )
//	{
//		tempBuffer[1] = aRxBuffer[0];
//		tempBuffer[2] = aRxBuffer[1];
//		tempBuffer[3] = aRxBuffer[2];
//		enqueue(tempBuffer);
//		tempBuffer[0] = tempBuffer[1] = tempBuffer[2] = tempBuffer[3] = "\0";
//		HAL_UART_Receive_IT(&huart3, aRxBuffer , 1);
//	}
//	else
//	{
		if(aRxBuffer[0] == 'f' || aRxBuffer[0] == 'r' || aRxBuffer[0] == 'l' || aRxBuffer[0] == 'b' || aRxBuffer[0] == 'w' || aRxBuffer[0] == 'v' || aRxBuffer[0] == 'n' || aRxBuffer[0] == 's')
		{
			tempBuffer[0] = aRxBuffer[0];
			//HAL_UART_Receive_IT(&huart3, aRxBuffer , 3); // receive the next 3 digits for dist/angle
			tempBuffer[1] = aRxBuffer[1];
			tempBuffer[2] = aRxBuffer[2];
			tempBuffer[3] = aRxBuffer[3];
			enqueue(tempBuffer); // for one shot
			HAL_UART_Receive_IT(&huart3, aRxBuffer , 4); // receive the next command

		}
		else // if any other random char,re-read
		{

			HAL_UART_Receive_IT(&huart3, aRxBuffer , 1); // receive the next command
			//HAL_UART_Receive_IT(&huart3, aRxBuffer , 4); // receive the next command
		}

//	}
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
  /* Infinite loop */
	uint8_t buffer[10];
	int i = 0;
	uint8_t line2[20];
	uint8_t line3[20];
//	uint32_t xLastWakeTime;
//	const uint32_t xFrequency = 50;

	//xLastWakeTime = osKernelGetTickCount();
	osDelay(500);
	for(;;)
	{
	  if((flagStart == 1) && (i < 5)) //waiting for first command
	  {
		  HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x37, 1, (uint8_t *)&buffer, 2, TIMEOUT);
		  osDelay(5);
		  gyro_val = (int16_t)(buffer[0]<<8|buffer[1]);
		  gyro_val = round(gyro_val/131.0 + 0.3);
		 // sprintf(&line2, "Gyro: %5d\0", gyro_val);
		 // OLED_ShowString(0,10,line2);
		  if(gyro_val < 2 && gyro_val >-1)
		  {
			  gyro_val = 0;
		  }
		  cum_gyro += gyro_val;
		  sprintf(&line3, "cum_gyro: %5d\0", cum_gyro);
		  OLED_ShowString(0,20,line3);
		  OLED_Refresh_Gram();
		  i++;
		  osDelay(250);
	  }
	  else if(strFlag == 1)
	  {
		  HAL_I2C_Mem_Read(&hi2c1, ICM_ADR, 0x37, 1, (uint8_t *)&buffer, 2, TIMEOUT);
		  osDelay(5);
		  gyro_val = (int16_t)(buffer[0]<<8|buffer[1]);
		  gyro_val = round(gyro_val/131.0 + 0.3);
		 // sprintf(&line2, "Gyro: %5d\0", gyro_val);
		 // OLED_ShowString(0,10,line2);
		  if(gyro_val < 2 && gyro_val >-1)
		  {
			  gyro_val = 0;
		  }
		  cum_gyro += gyro_val;
		  sprintf(&line3, "cum_gyro: %5d\0", cum_gyro);
		  OLED_ShowString(0,20,line3);
	  }
//	  OLED_Refresh_Gram();
//	  xLastWakeTime += xFrequency;
//	  osDelayUntil(xLastWakeTime);
      osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_showTask */
/**
* @brief Function implementing the Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_showTask */
void showTask(void *argument)
{
  /* USER CODE BEGIN showTask */
  /* Infinite loop */
	uint8_t hello[15] = "Hello World!\0";
  for(;;)
  {
//	  sprintf(hello,"SpeedA: %5d\0",speedA);
//	  OLED_ShowString(10,30,hello);
//	  sprintf(hello,"SpeedB: %5d\0",speedB);
//	  OLED_ShowString(10,40,hello);
//	  sprintf(hello,"hello\n\0"); // testing, NOTE WE NEED THE \n for them to receive
//	  HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 7, 0xFFFF); //testing
	  sprintf(hello,"Dir: %4d\0",direction);
	  OLED_ShowString(10,50,hello);
	  OLED_Refresh_Gram(); // refresh the screen to show
	  osDelay(250);
  }
  /* USER CODE END showTask */
}

/* USER CODE BEGIN Header_motormovetask */
/**
* @brief Function implementing the MotorMove thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motormovetask */
void motormovetask(void *argument)
{
  /* USER CODE BEGIN motormovetask */
  /* Infinite loop */
	char dequeuedVal[4];
	int val = 0;
	int time = 0;
	uint8_t hello[20] = "No command\0"; // testing purposes
	osDelay(1000);
	for(;;)
	{
		// actual code
		hello[0] = "";
// =====================================================================================================
		while(flagStart == 1) // waiting for first command
		{
		  osDelay(1); // idle
		}

		if(Rstate == 0 && Lstate == 0 && MotorReady == 0) // when robot is not moving and is ready to move
		{
		  MotorReady = 1; // motor is ready to move
		  osDelay(1);
		}
		if(MotorReady == 1) // we are ready to move
		{
		  if(Front != -1 && Front <= Back) // if there are tasks
		  {
			  MotorReady = 0;
			  strncpy(dequeuedVal,dequeue(),4);
			  val = conCatValue(dequeuedVal);
				//sprintf(hello,"%c\0",dequeuedVal[0]); // testing purposes
				switch(dequeuedVal[0])
				{
					case 'f':
						wheelMoveForward(val);
						sprintf(hello,"ACK, Forward %d\n\0",val);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'b':
						wheelMoveBackward(val);
						sprintf(hello,"ACK, Back %d\n\0",val);
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'r':
						turn_right(val);
						sprintf(hello,"ACK, TRight %d\n\0",val);
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'l':
						turn_left(val);
						sprintf(hello,"ACK, TLeft %d\n\0",val);
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'v':
						back_left(val);
						sprintf(hello,"ACK, BLeft %d\n\0",val);
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'n':
						back_right(val);
						sprintf(hello,"ACK, BRight %d\n\0",val);
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						//OLED_ShowString(10,10,hello); // testing purposes
						break;
					case 'w':
						wheel_stop();
						sprintf(hello,"ACK, WAIT\n\0");
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
						osDelay(5000);
						break;
					case 's':
						wheel_stop();
						sprintf(hello,"ACK, STOP\n\0"); // testing purposes
//						HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 15 , 0xFFFF);
						osDelay(5000);
						break;
					default:
						wheel_stop();
						break;
				}
				HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 20, 0xFFFF);
				//osDelay(1000);

		  }
		  else // else no more task we stop;
		  {
			  wheel_stop();
		  }
		}
		osDelay(100); // do this every 1/4 seconds
//=======================================================================================================================
			//turn_right(90); // wooden is ok
			//back_right(90);

			//turn_left(270);
			//back_left(90);
		//wheelMoveForward(1000);
		//osDelay(500);
		//osDelay(10000); // testing stop
    //osDelay(1);
  }
  /* USER CODE END motormovetask */
}

/* USER CODE BEGIN Header_encodertask */
/**
* @brief Function implementing the Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encodertask */
void encodertask(void *argument)
{
  /* USER CODE BEGIN encodertask */
  /* Infinite loop */
	osDelay(1000);
//	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
//	uint32_t tick;
//	cntA1 = __HAL_TIM_GET_COUNTER(&htim2); //get the first tick
//	cntB1 = __HAL_TIM_GET_COUNTER(&htim3); //get the first tick
//	tick = HAL_GetTick();

	for(;;)
	{
		//dont think we need encoder anymore
//		if(HAL_GetTick()-tick >1000L)
//	    {
//			// get speed A
//			cntA2 = __HAL_TIM_GET_COUNTER(&htim2);
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
//			{
//			  if(cntA2<cntA1)
//			  {
//				  speedA = encoderDiff(cntA1 - cntA2);
//			  }
//			else
//			{
//				  speedA = (65535 - cntA2) + cntA1;
//				  if (speedA > 5500)
//				  {
//					  speedA = 0;
//				  }
//			}
//		}
//		else
//		{
//			  if(cntA2>cntA1)
//			  {
//				  speedA = encoderDiff(cntA2 - cntA1);
//			  }
//			  else
//			  {
//				  speedA = (65535 - cntA1) + cntA2;
//				  if (speedA > 5500)
//				  {
//					  speedA = 0;
//				  }
//			  }
//			}
//			// get speed B
//			cntB2 = __HAL_TIM_GET_COUNTER(&htim3);
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
//			{
//			  if(cntB2<cntB1)
//			  {
//				  speedB = encoderDiff(cntB1 - cntB2);
//			  }
//			  else
//			  {
//				  speedB = (65535 - cntB2) + cntB1;
//				  if (speedB > 5500)
//				  {
//					  speedB = 0;
//				  }
//			  }
//			}
//			else
//			{
//			  if(cntB2>cntB1)
//			  {
//				  speedB = encoderDiff(cntB2 - cntB1);
//			  }
//			  else
//			  {
//				  speedB = (65535 - cntB1) + cntB2;
//				  if (speedB > 5500)
//				  {
//					  speedB = 0;
//				  }
//			  }
//			}
//			cntA1 = __HAL_TIM_GET_COUNTER(&htim2);
//			cntB1 = __HAL_TIM_GET_COUNTER(&htim3);
//			tick = HAL_GetTick();
//	  	}
//		if(wheelRunning == 1) // if our wheel is running
//		{
//			adjustSpeed(speedA,speedB);
//		}
		osDelay(10000);
	}
  /* USER CODE END encodertask */
}

/* USER CODE BEGIN Header_irsensortask */
/**
* @brief Function implementing the IRSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_irsensortask */
void irsensortask(void *argument)
{
  /* USER CODE BEGIN irsensortask */
  /* Infinite loop */
	uint8_t hello[15] = "         \0";
	uint8_t oledIR[15] = "\0";
	uint8_t uartVal[20];
	int value = 0;
	float adcVal = 0;
	float dist = 0;
	char buf[100];
  for(;;)
  {
	  //not needed for checklists and week 8
//	  hello[15] = "               \0";
//
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1,10); // time out = 10
//	  value = HAL_ADC_GetValue(&hadc1);
//
//	  HAL_ADC_Stop(&hadc1);
//	  //OLED_ShowString(10,10,hello);
//	  if(value >= 3800)
//	  {
//		  dist = 5;
//	  }
//	  else
//	  {
//		  adcVal = pow(value,-1.19);
//
//		  dist = roundf(1.44*pow(10,5)* adcVal)-1;
//	  }
//	  if(dist < 200) // to remove infinity
//	  {
//		  gcvt(dist,5,buf);
//		  sprintf(hello,"IR: %s\n\r\0",buf);
//		  HAL_UART_Transmit(&huart3,(uint8_t *)&hello, 10 , 0xFFFF);
//		  //OLED_ShowString(10,0,"             ");
//		  //OLED_ShowString(10,0,buf);
//	  }
	  osDelay(2000);
	  osDelay(1);
    }
  /* USER CODE END irsensortask */
}

/* USER CODE BEGIN Header_servoTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servoTask */
void servoTask(void *argument)
{
  /* USER CODE BEGIN servoTask */
  /* Infinite loop */
  for(;;)
  {
		if(strFlag == 1) // if we are going straight
		{
			if(cum_gyro < 5 && cum_gyro > -5)
			{
				direction = MID;
				continue;
			}
			else
			{
				if(cum_gyro > 5 && direction < MID + 10) // swaying to the left
				{
					if(cum_gyro > 20) // need a drastic step
					{
						direction +=3;
					}
					else // correct to the right
					{
						direction+=2;
					}
				}
				else if(cum_gyro < -5 && direction > MID - 10) // swaying to the right
				{
					if(cum_gyro < -20) // need a drastic step
					{
						direction -=3;
					}
					else
					{
						direction-=2;
					}
				}
				htim1.Instance->CCR4 = direction;
				osDelay(50);
			}
		}
		osDelay(100);
  }
  /* USER CODE END servoTask */
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

