/*****************************************************************************
*	MULTICALOR NV
*	
*	File:	main.c
*		Update regularly the flags
*
******************************************************************************
*	Verion V1.0.0
*	Date: 21-01-2015
*	Main program body
******************************************************************************
*/

/*****************************************
*	Newlib libc
*
******************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/*****************************************
*	Include peripheral drivers
*
******************************************
*/
#include "main.h"
#include "stm32f4xx_it.h"

/*****************************************
*	Include exported libs
*
******************************************
*/
/*	Modbus includes --------------------------------------------------*/
#include "mb.h"
#include "mbport.h"


/*	Private Functions ------------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void init_leds(void);
static void sram_init(void);
static void I2C_init(void);
static void USART_init(void);
static void timer_init(void);

/*	Private macro -----------------------------------------------------*/
/*	Private variables -------------------------------------------------*/
SRAM_HandleTypeDef hsram;
FSMC_NORSRAM_TimingTypeDef SRAM_Timing;
I2C_HandleTypeDef I2cHandle;
UART_HandleTypeDef UartHandle;
TIM_HandleTypeDef TimHandle;

uint32_t baudrate = 115200;
uint32_t uwPrescalerValue = 0;

static uint16_t usRegInputStart = REG_INPUT_START;
static uint16_t usRegInputBuf[REG_INPUT_NREGS];

/*	Defines -----------------------------------------------------------*/
#define	I2C_ADDRESS		0xFE

//****************************************************************************
/*	Static variables -------------------------------------------------*/
static int Debug_ITMDebug = 0;

//****************************************************************************

void Debug_ITMDebugEnable(void){
	volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
	volatile unsigned int *SCB_DHCSR 		= (volatile unsigned int *)0xE000EDF0;
	volatile unsigned int *DBGMCU_CR 		= (volatile unsigned int *)0xE0042004;

	*DBGMCU_CR |= 0x27; // DBGMCU_CR

  if ((*SCB_DHCSR & 1) && (*ITM_TER & 1)) // Enabled?
    Debug_ITMDebug = 1;
}

//****************************************************************************

void Debug_ITMDebugOutputChar(char ch){
	static volatile unsigned int *ITM_STIM0 = (volatile unsigned int *)0xE0000000; // ITM Port 0
	static volatile unsigned int *SCB_DEMCR = (volatile unsigned int *)0xE000EDFC;

	if (Debug_ITMDebug && (*SCB_DEMCR & 0x01000000))
	{
		while(*ITM_STIM0 == 0);
  	*((volatile char *)ITM_STIM0) = ch;
	}
}

//****************************************************************************

void Debug_ITMDebugOutputString(char *Buffer){
	if (Debug_ITMDebug)
		while(*Buffer)
			Debug_ITMDebugOutputChar(*Buffer++);
}

//******************************************************************************


/**
  * @brief	main().
  * @param	None
  *	@note	Init the system.
  * 		return here after a call.
  *	@retval	None
  */
int main(void){
	int n;
	eMBErrorCode eStatus;
	uint32_t Uart_Error;
	
	/* STM32f4xx HAL library initialization:
	 - Cofigure Flash prefetch, flash preread and Buffer caches
	 - Systick timer config
	 - Low level init
	 */
	HAL_Init();
	/*********************************************************************/

	/* configure the system clock to 168 MHz */
	SystemClock_Config();

	/* Enable TRACE debug -----------------------------------------------*/
	Debug_ITMDebugEnable();
 	Debug_ITMDebugOutputString("SWV Enabled\n");
	/*********************************************************************/
	
	/* Init peripherals -------------------------------------------------*/
	sram_init();
	I2C_init();
	USART_init();
	timer_init();
	/*********************************************************************/
	
	/*	Show us some status leds ----------------------------------------*/
	init_leds();
	/*********************************************************************/
	
	/*	Init modbus	slave -----------------------------------------------*/
	eStatus = eMBInit(MB_RTU, 0x0A, 0, 115200, MB_PAR_EVEN);
	printf("eStatus: %s\n", eStatus ? "error": "no'error");
	/*********************************************************************/
	
	/*	Enable the Modbus Protocol Stack --------------------------------*/
	eStatus = eMBEnable();
	printf("eStatus: %s\n", eStatus ? "error": "no'error");
	
	while(1){

		//printf("HAL integrated...\n");
				
		//test_Cplusplus();
		
		if(gui_Exec == true){
			eStatus = eMBPoll();
			
			usRegInputBuf[499]++;
			usRegInputBuf[500]++;
			usRegInputBuf[501]++;
			usRegInputBuf[502]++;
			printf("eStatus: %s\n", eStatus ? "error": "no'error");
			printf("SystemCoreClock: %d\n", SystemCoreClock);
			
			/*Clear Flag*/
			gui_Exec = false;
		}
		
		if(Touch_Flagged == true){
			
			//Uart_Error = HAL_UART_GetError(&UartHandle);
			//printf("Uart_Error: %d\n", Uart_Error);
			
			Touch_Flagged = false;
		}
		
	}
}

/**
  * @brief	Builds the Input Registers frame.
  * @param	*pucRegBuffer
  * @param	usAddress
  * @param	usNRegs
  * @retval	eMBErrorCode
  */
eMBErrorCode
eMBRegInputCB(uint8_t* pucRegBuffer, uint16_t usAddress, uint16_t usNRegs)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( (usAddress >= REG_INPUT_START)
        && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief	Builds the Holding Registers frame.
  * @param	*pucRegBuffer
  * @param	usAddress
  * @param	usNRegs
  * @param	eMode
  * @retval	eMBErrorCode
  */
eMBErrorCode
eMBRegHoldingCB(uint8_t* pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNRegs;
    ( void )eMode;
    return MB_ENOREG;
}

/**
  * @brief	Builds the Coils frame.
  * @param	*pucRegBuffer
  * @param	usAddress
  * @param	usNCoils
  * @param	eMode
  * @retval	eMBErrorCode
  */
eMBErrorCode
eMBRegCoilsCB(uint8_t* pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNCoils;
    ( void )eMode;
    return MB_ENOREG;
}

/**
  * @brief	Builds the Discrete frame.
  * @param	*pucRegBuffer
  * @param	usAddress
  * @param	usNDiscrete
  * @retval	eMBErrorCode
  */
eMBErrorCode
eMBRegDiscreteCB(uint8_t* pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNDiscrete;
    return MB_ENOREG;
}


/**
  * @brief	System Clock configuration
  *		System clock Source	= PLL(HSE)
  *		SYSCLK(Hz)		= 168000000
  *		HCLK(Hz)		= 168000000
  *		AHB Prescale		= 1
  *		APB1 Prescale		= 4
  *		APB2 Prescale		= 2
  *		HSE frequency(Hz)	= 8000000
  *		PLL_M			= 8
  *		PLL_N			= 336
  *		PLL_P			= 2
  *		PLL_Q			= 7
  *		VDD(V)			= 3.3
  *		Main reg out Volt	= Scale1 mode
  *		Flash lat(ws)		= 5
  *	@param None
  *	@retval None
  */
static void SystemClock_Config(void){
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the
	   device is clocked below the maximum system frequency, to update the
	   voltage scaling value regarding system frequency refer to product
	   datasheet. */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = 4;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		/* Initialization Error */
		Error_Handler();
	}
	/* Select PLL as system clock source and configure the HCLK, PCLK1
	   and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK	| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK){
		/* Initialization Error */
		Error_Handler();
	}

	/* STM32F407x Revision Z devices: prefetch is supported */
	if(HAL_GetREVID() == 0x1001){
		/* Enable the Flash prefetch */
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	}
}


/**
  * @brief	Handles general errors.
  * @param	None
  * @retval	None
  */
static void Error_Handler(void){
	
	while(1){
		/* Put error on LED3 */
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		printf("Peripheral config error\n");
	}
}

/**
  * @brief	Init leds PD12-13
  * @param	None
  * @retval	None
  */
static void init_leds(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable AHB1per Clock
	__GPIOC_CLK_ENABLE();

	//Config pins
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief init SRAM
  * @param None
  * @retval None
  */
static void sram_init(void){
	/*	Configure the FSCM bus ---------------------------*/
	SRAM_Timing.AddressSetupTime = 1;
	SRAM_Timing.AddressHoldTime = 1;
	SRAM_Timing.DataSetupTime = 1;
	SRAM_Timing.BusTurnAroundDuration = 1;
	SRAM_Timing.CLKDivision = 1;
	SRAM_Timing.DataLatency = 1;
	SRAM_Timing.AccessMode = FSMC_ACCESS_MODE_A;

	hsram.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;

	/* Initialize the SRAM controller ----------------------------*/
	if(HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK){
		/* Init Error */
		Error_Handler();
	}
}

/**
  * @brief init I2C1
  * @param None
  * @retval None
  */
static void I2C_init(void){
	/*	Configure the I2C peripheral ---------------------------*/
  I2cHandle.Instance             = I2Cx;

  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      = 400000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle.Init.OwnAddress2	 = I2C_ADDRESS;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();    
  }

}

/**
  * @brief init USART
  * @param None
  * @retval None
  */
static void USART_init(void){
	/*	Configure the USART1 peripheral in the Asynchronous mode (UART Mode)------*/
	/* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USARTx;
  
	UartHandle.Init.BaudRate     = 28800;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_8;
    
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief init timer
  * @param None
  * @retval None
  */
static void timer_init(void){
	uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
	
	TimHandle.Instance = TIMx;
	
	TimHandle.Init.Period = 1000 - 1;
	TimHandle.Init.Prescaler = uwPrescalerValue;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
}

/******************************************************************************/

/**
  * @brief  Retargets the C library printf function to the USART (GNU)
  * @param  None
  * @retval None
  */
int __io_putchar(int ch){
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */

	Debug_ITMDebugOutputChar(ch); // SWV

  return(ch);
}

//******************************************************************************

/**
  * @brief  Retargets the C library scanf function to the USART (GNU)
  * @param  None
  * @retval None
  */
int __io_getchar(void){
  /* Place your implementation of fgetc here */
  /* e.g. read a character from the USART */

  return((int)-1);
}

//******************************************************************************

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  while(1); /* Infinite loop */
}
#endif

//******************************************************************************
