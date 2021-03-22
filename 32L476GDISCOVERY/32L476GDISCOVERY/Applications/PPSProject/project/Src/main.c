/**
  ******************************************************************************
  * @file    Src/main.c
  * @author  John Quach
  * @brief   External PPS Clock Synchroniziation Demo
	*	@course	 ELEN502 - Real-Time Systems
	* @date    3/16/2021
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
struct PPSTimeStamp_struct
{
	uint32_t TimeStamp;
	uint32_t Delta;
	uint32_t Prev;
	uint16_t DeltaBuffer[10];
	uint16_t Jitterbuffer[20];
	uint16_t Jitter[9];
	uint16_t JitterSum;
	uint16_t JitterAvg;
	uint16_t DeltaBufferSum;
	uint16_t DeltaBufferAvg;
	uint32_t TimeStampBuffer[10];
	uint8_t BufferPos;
	uint8_t SyncStage;
	double ClockCorrScaleFactor;
};
/* Private define ------------------------------------------------------------*/
#define PPS_Sync_Timeout	5000	//ms
#define PPS_Propagation_Timeout 17500 //ms
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LED_ThreadHandle, PPS_ThreadHandle, PPSTest_ThreadHandle;
osThreadId ClkMgr_ThreadHandle, LCD_ThreadHandle;
volatile uint32_t PPSIntrCnt;
volatile uint16_t PPS_Thread_Cnt;
volatile uint32_t SysTimeSec;
osSemaphoreId PPS_sem;
osSemaphoreDef(PPS_sem);
osSemaphoreId LCD_sem;
osSemaphoreDef(LCD_sem);
volatile uint32_t HalTicks;
struct PPSTimeStamp_struct PPSTimeStamp;
/* Private function prototypes -----------------------------------------------*/
static void LED_Thread(void const *argument);
static void PPS_Thread(void const *argument);
static void ClkMgr_Thread(void const *argument);
static void LCD_Thread(void const *argument);
static void PPSTest_Thread(void const *argument);
void SystemClock_Config(void);
static void EXTI0_IRQHandler_Config(void);
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	
	/******* I/O Initialization ROUTINES *******/
	HAL_Init();

  /* Configure the System clock to 4 MHz */
  SystemClock_Config();

	/* Initialize PPS Struct */
	memset(&PPSTimeStamp, 0, sizeof(PPSTimeStamp));
	PPSTimeStamp.ClockCorrScaleFactor = 1.0;
	
	/* Initialize LCD Display */
	BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_Clear();
	
  /* Initialize LEDs */
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
	BSP_LED_On(LED4);
	
	/* 4Mhz Clock... Don't think I'll use it.  */
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
	TIM5->CR1 = 1;

  /* Configure EXTI_Line0 (connected to PA.0 pin) in interrupt mode */
  EXTI0_IRQHandler_Config();	

	/* Create Semaphores for s */
	PPS_sem = osSemaphoreCreate(osSemaphore(PPS_sem), 1);
	LCD_sem = osSemaphoreCreate(osSemaphore(LCD_sem), 1);

	/******* THREAD/ CREATION ROUTINES *******/
	/* PPS ISR  */
	osThreadDef(PPS, PPS_Thread, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE);
	PPS_ThreadHandle = osThreadCreate(osThread(PPS), NULL);
	
	/* Clock Manager  */
	osThreadDef(ClkMgr, ClkMgr_Thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);
	ClkMgr_ThreadHandle = osThreadCreate(osThread(ClkMgr), NULL);
	
	/* LCD Display  */
	osThreadDef(LCD, LCD_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE);
	LCD_ThreadHandle = osThreadCreate(osThread(LCD), NULL);

  /* LED Actuator  */
  osThreadDef(LED, LED_Thread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  LED_ThreadHandle = osThreadCreate(osThread(LED), NULL);
	
	/* PPS Input Test  */
	osThreadDef(PPSTest, PPSTest_Thread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  PPSTest_ThreadHandle = osThreadCreate(osThread(PPSTest), NULL);
	
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);
}

static void PPS_Thread(void const *argument)
{
	int i;
	
		while (1)
    {
			//First Wait is released at boot-up.  It's OK for this app since 5 consistent PPS required for sync.
			osSemaphoreWait(PPS_sem, osWaitForever);
			PPS_Thread_Cnt++;
			
			/* PPS Logic  */
			PPSTimeStamp.TimeStamp = osKernelSysTick();
			PPSTimeStamp.Delta = PPSTimeStamp.TimeStamp - PPSTimeStamp.Prev;
			
			if(PPSTimeStamp.Delta >= 100 || PPSTimeStamp.Delta <= 5000) { 
				PPSTimeStamp.DeltaBuffer[PPSTimeStamp.BufferPos] = PPSTimeStamp.Delta;
				PPSTimeStamp.TimeStampBuffer[PPSTimeStamp.BufferPos] = PPSTimeStamp.TimeStamp;
				PPSTimeStamp.BufferPos++;
				
				//PPS State Machine
				//Stage 0: Read at least 10 consistent samples
				if (PPSTimeStamp.SyncStage == 0) if(PPSTimeStamp.BufferPos==10) PPSTimeStamp.SyncStage = 1;
				
				//Stage 1: Check for avg jitter within 10ms of local oscillator
				if (PPSTimeStamp.SyncStage == 1) {				
					  //To measure jitter, take the delta between samples, divide by samples-1
						memset(&PPSTimeStamp.Jitterbuffer, 0, sizeof(PPSTimeStamp.Jitterbuffer));
						memcpy(PPSTimeStamp.Jitterbuffer, PPSTimeStamp.DeltaBuffer, sizeof(PPSTimeStamp.DeltaBuffer));
						for(i=0;i<PPSTimeStamp.BufferPos;i++) {
							PPSTimeStamp.Jitterbuffer[i+10] = PPSTimeStamp.DeltaBuffer[i];
							PPSTimeStamp.Jitterbuffer[i] = 0;
						}
						
						for(i=0;i<9;i++)
							PPSTimeStamp.Jitter[i] = PPSTimeStamp.Jitterbuffer[PPSTimeStamp.BufferPos+i+1] - PPSTimeStamp.Jitterbuffer[PPSTimeStamp.BufferPos+i];
						PPSTimeStamp.JitterSum = 0;
						for(i=0;i<9;i++) PPSTimeStamp.JitterSum += PPSTimeStamp.Jitter[i];
						PPSTimeStamp.JitterAvg = PPSTimeStamp.JitterSum/9;
						
						if(PPSTimeStamp.JitterAvg <= 10) PPSTimeStamp.SyncStage++;
						//PPSTimeStamp.SyncStage++;
					}
					
					//Stage 2: Synced
					if (PPSTimeStamp.SyncStage == 2) {	
						//Find average Delta and use that to calculate scale factor between PPS and local clock
						PPSTimeStamp.DeltaBufferSum = 0;
						for(i=0;i<10;i++) PPSTimeStamp.DeltaBufferSum += PPSTimeStamp.DeltaBuffer[i];
						PPSTimeStamp.DeltaBufferAvg = PPSTimeStamp.DeltaBufferSum/10;
						PPSTimeStamp.ClockCorrScaleFactor = (double)PPSTimeStamp.DeltaBufferAvg/1000;
					}
					
					if(PPSTimeStamp.BufferPos == 10) PPSTimeStamp.BufferPos = 0;
			}
			else {
				PPSTimeStamp.BufferPos = 0;
			}
			
			PPSTimeStamp.Prev = PPSTimeStamp.TimeStamp;
		}
}

static void ClkMgr_Thread(void const *argument)
{
	uint32_t ClkMgrPrevWakeTime;
	
	ClkMgrPrevWakeTime = osKernelSysTick();
	
		while (1)
    {
			osDelayUntil(&ClkMgrPrevWakeTime, 1000*PPSTimeStamp.ClockCorrScaleFactor);
			
			//PPS Failout Logic
			if((osKernelSysTick() - PPSTimeStamp.Prev) > PPS_Sync_Timeout) PPSTimeStamp.SyncStage = 0;
			if((osKernelSysTick() - PPSTimeStamp.Prev) > PPS_Propagation_Timeout) PPSTimeStamp.ClockCorrScaleFactor = 1;
			
			SysTimeSec++;			
			
			//Semaphores for tasks that wait on Sys Clock Tick
			osSemaphoreRelease(LCD_sem);
		}
}

static void LCD_Thread(void const *argument)
{
	char SysTimeSec_char[6];
		while (1)
    {
			osSemaphoreWait(LCD_sem, osWaitForever);
			sprintf(&SysTimeSec_char[1], "%u", SysTimeSec);
			if(PPSTimeStamp.SyncStage == 2) SysTimeSec_char[0] = 'E';
			else if(PPSTimeStamp.ClockCorrScaleFactor != 1.0) SysTimeSec_char[0] = 'P';
			else SysTimeSec_char[0] = 'F';
			BSP_LCD_GLASS_DisplayString((uint8_t *)SysTimeSec_char);
		}
}

static void LED_Thread(void const *argument)
{
  uint32_t LEDPrevWakeTime;
	
	LEDPrevWakeTime = osKernelSysTick();
	
  while(1)
  {
			osDelayUntil(&LEDPrevWakeTime, 100*PPSTimeStamp.ClockCorrScaleFactor);
      //BSP_LED_On(LED4);
			if(PPSTimeStamp.SyncStage == 2) BSP_LED_On(LED5);
			else if(PPSTimeStamp.SyncStage == 0 && PPSTimeStamp.ClockCorrScaleFactor != 1.0) BSP_LED_Toggle(LED5);
			else BSP_LED_Off(LED5);
  }
}

static void PPSTest_Thread(void const *argument)
{
	uint32_t PPSTestPrevWakeTime;
	
	PPSTestPrevWakeTime = osKernelSysTick();
	
		while (1)
    {
			osDelayUntil(&PPSTestPrevWakeTime, 500);
			if(PPSIntrCnt < 30) osSemaphoreRelease(PPS_sem);
			PPSIntrCnt++;
		}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* The following clock configuration sets the Clock configuration sets after System reset                */
  /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
  /* and to be eventually adapted to new clock configuration                                               */

  /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}

/**
  * @brief  Configures EXTI line 0 (connected to PA.0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI0_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA.0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI line 0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0); //FreeRTOS lowest SAFE priority
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
		osSemaphoreRelease(PPS_sem);
		BSP_LED_Toggle(LED4);
		PPSIntrCnt++;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
