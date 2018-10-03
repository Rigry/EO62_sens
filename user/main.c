#include <stdbool.h>
#include "stm32f10x.h"
#include "communic.h"
#include "temp_calc.h"
//////////////////////////
#include "macros.h"		//Modbus
#include "MBSlave.h"	//Modbus

#define WRITE_START_ADDR	0x0801FC00 //адрес страницы для записи параметров
#define ADC1_DR_Address    ((uint32_t)0x4001244C)		//Адрес регистра данных АЦП
//ДК: откуда это адское число?
#define BufferLenght       4							//Число каналов оцифровываемых с помощью ацп
														//ДК: стока АЦП будет оцифровано с помощью ДМА
														
#define START_TIMER		SysTick->VAL = TimerTick; SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;  	//запускает таймер определения конца пакета 
#define STOP_TIMER		SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk 	
#define TimerTick  		4700*72-1	// 4700 мкс для 9600 и 72МГц
														
void init_ports (void);			//ну тут всё ясно
void init_adc_dma_trans(void); 	//это для АЦП1 прямо в память, внутри не разбирался
void UART3_Init(void);			//инициализация функциями, что мне не нравится, хотя в целом ясно
void PACK_SEND(uint8_t *BUF, uint8_t count);	//ДК: отправка пакета. Работает, пока не отправит

//////////////////////////////
void InitTimer(void);		//Modbus
struct UartBufSt UartBuf;	//Modbus
struct MBRegSt MBReg;		//Modbus
enum MBOutRegE {			//Modbus
	uf 		= 0,			//Modbus
	temper 	= 1				//Modbus
};							//Modbus
 
uint8_t  board_addr;
 
__IO uint16_t ADC1ConvertedValue[BufferLenght];	//буфер АЦП
//ДК: __IO это volatile - нужно, чтоб компилятор не оптимизировал его

int main (void)
{
	uint16_t temp ,uroven_uf;	//здесь значение АЦП для датчиков температуры и УФ
	int16_t temperatura;		//тут конвертнутая температура в цельсиях
	uint8_t summ_counter=0;
	uint32_t summa=0;
	uint32_t summat = 0;
	
	__enable_irq();			//ДК: разрешение прерывания судя по всему - указание компилятору

 	init_ports();			//настраиваем порты
	init_adc_dma_trans();	//ДК: магия с дма
	UART3_Init();			//Инициализация RS канала
	USART_MODE_RX();		//RS на прием
							//ДК: управляет RTS - define
	InitTimer();			//для определения конца пакета
	
	board_addr=0x0A; //Адрес платы на шине RS485
	 
	temp=0;
	uroven_uf=0;
//////////////////////////////	
	UartBuf.N = 0;			//Modbus
	UartBuf.NeedSend = 0;	//Modbus
	UartBuf.EndMes = false;	//Modbus 
	
	while(1) //Главный говноцикл
	{
	/////////////////////////////////////////////////////////////////////////////////////////////////
	//обновление измеренных значений, когда DMA закончил
	/////////////////////////////////////////////////////////////////////////////////////////////////		
		if (DMA_GetFlagStatus(DMA1_FLAG_TC1))	{	//ДК: когда магия с дма кончилась
			if (summ_counter<250) {					//всё суммируем до 250 значений
				summa = summa + ADC1ConvertedValue[0];
				summat = summat + ADC1ConvertedValue[1];
				summ_counter++;
			} else {
				uroven_uf=summa/250;			//ДК: находим среднее, чтобы уместится в 2 байта, потому что стока и шлём
			//////////////////////////////////////
				MBReg.RegOut[uf] = uroven_uf;	//Modbus
				
				temp = summat / 250;
				temperatura=(calc_temperature(temp))/10;		//переводим отсчеты АЦП в значение температуры
			//////////////////////////////////////	 
				MBReg.RegOut[temper] = temperatura;
				 
				summ_counter=0;
				summa=0;
				summat = 0;
			}
			DMA_ClearFlag(DMA1_FLAG_TC1);
		}
					  
	/////////////////////////////////////////////////////////////////////////////////////////////////
	//отправка ответа по UART
	/////////////////////////////////////////////////////////////////////////////////////////////////
		if (UartBuf.EndMes) {
			MBSlave(&UartBuf, &MBReg, board_addr);		//EndMes сбрасывается в функции
														//там же формируется пакет и его размер
			if (UartBuf.NeedSend != 0) {
				GPIOB->BSRR = GPIO_Pin_15;	//Включаем Индикатор активности на шине
				PACK_SEND(UartBuf.Buf, UartBuf.NeedSend);
				UartBuf.NeedSend = 0;
				GPIOB->BRR = GPIO_Pin_15;//выключаем Индикатор активности на шине
			}	
		}	 
	}//main loop end
}//main

void USART3_IRQHandler(void)	//ДК: прерывание по приему байта
{
	STOP_TIMER;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		UartBuf.Buf[UartBuf.N] = (USART_ReceiveData(USART3) & 0xFF);
		UartBuf.N++;
		if (UartBuf.N >= UART_BUF_SIZE-1) {
			UartBuf.N = 0;
		}	
		if (UartBuf.Buf[0] == board_addr) {
		//	GPIOB->BSRR = GPIO_Pin_15;	//Включаем Индикатор активности на шине
		}
	}
	START_TIMER;

  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  { }
}

void SysTick_Handler(void)	//Modbus
{
	STOP_TIMER;
	UartBuf.EndMes = true;
//	GPIOB->BRR = GPIO_Pin_15;//выключаем Индикатор активности на шине
}

void PACK_SEND(uint8_t *BUF, uint8_t count)
{
	uint8_t Nsend = 0;
	USART_MODE_TX();
	__NOP();__NOP(); 
	do {
		 while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		 USART_SendData(USART3,BUF[Nsend]);
	} while (Nsend++ < count-1);
	 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	USART_MODE_RX();				
}


void InitTimer(void)
{
	SysTick->LOAD = TimerTick;	// Загрузка значения
	SysTick->VAL = TimerTick;	// Обнуляем таймеры и флаги 
 
	SysTick->CTRL =	SysTick_CTRL_CLKSOURCE_Msk 		//processor clock
					| SysTick_CTRL_TICKINT_Msk;		//разрешение прерывания
}

void init_ports()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;	//наша инит структура
	
/////////////////////////////////////////////////////////////////////////////////////////////////
//  ПОРТЫ Джамперов Адреса платы																														 //
/////////////////////////////////////////////////////////////////////////////////////////////////
	/* GPIOB, GPIOD and AFIO clocks enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		//Копипаст:  Enables or disables the High Speed APB (APB2) peripheral clock.
		//Копипаст: This parameter can be any combination of the following values
		//ДК: RCC_APB2Periph_AFIO для ремапинга как я понял
   GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		//фулл жЫтаг нахуй,шьемся по SWD(освобождаем PA15,PB3,PB4)
																//Копипаст: JTAG-DP Disabled and SW-DP Enabled
   
/////////////////////////////////////////////////////////////////////////////////////////////
// ПОРТ  диода активности
/////////////////////////////////////////////////////////////////////////////////////////////
	/*  PB15-ACT LED*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//Копипаст: GPIO_Mode_Out_PP = 0x10, Выход Пуш-Пул
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/////////////////////////////////////////////////////////////////////////////////////////////
// АНАЛОГОВЫЕ ВХОДЫ ОТ ДАТЧИКОВ
/////////////////////////////////////////////////////////////////////////////////////////////	 
 	//4 АЦП
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//ДК: А где УАРТ?!!, а УАРТ в инициализации УАРТА
}

void init_adc_dma_trans()
{
	DMA_InitTypeDef   DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//подключаем ДМА контроллер
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);// ПОДКЛЮЧАЕМ АЦП
	/* DMA1 channel1 configuration ---------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = BufferLenght;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = BufferLenght;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel11, channel14, channel16 and channel17 configurations */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
  
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
  
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
  
	/* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
  
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
  
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
     
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void UART3_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//ДК: клоки для уарта 3 (аналогично в GPIO, только для APB2)
	//ДК: взводит нужные биты в регистре управления клоками RCC_APB2ENR
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//Ремап пинов на PC10,PC11
	//ДК: меняет нужные биты в AFIO->MAPR2 или в AFIO->MAPR чтобы ремапнуть
	//ДК: для узарта3 это биты 3 и 4 AFIO->MAPR
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3 , ENABLE);
	//Копипаста: #define GPIO_PartialRemap_USART3    ((uint32_t)0x00140010)  /*!< USART3 Partial Alternate Function mapping */
	//Копипаста: Bits 5:4 USART3_REMAP[1:0]: USART3 remapping
	//Копипаста: 01: Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)
	//ДК: а 14 нужна для внутренних нужд функции, сложно понять зачем
	
	/* Configure USART3 Rx as input floating */
	//ДК: конкретно на что настраивать написано в RM0008 таблица 24
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART3 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//конфигурим управляющий пин USART3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//ДК: интересно, почему тут none. если в программе все равно этот вывод используется
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
	/* Configure USART3 */
	//ДК: тут интересно что USART3 это указатель на структуру
	//ДК: хотелось бы, но лучше не пользоваться чужими библиотеками
	USART_Init(USART3, &USART_InitStructure);
	// Подключаем прерывание USART3 по готовности буфера приема
	NVIC_EnableIRQ(USART3_IRQn);
	USART_ITConfig  (USART3,USART_IT_RXNE, ENABLE); 

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);
}







 
