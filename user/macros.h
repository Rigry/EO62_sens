/*
 * macros.h
 *
 * Created: 08.03.2017 7:11:01
 *  Author: dvk
 */ 
#include <stdint.h>
#include <stdbool.h>

#ifndef MACROS_H_
#define MACROS_H_

//#define F_CPU	16000000UL

#define UART_BUF_SIZE 100
#define QTY_IN_REG	1
#define QTY_OUT_REG	2

/*
#define RTS_DDR DDRD
#define RTS_PORT PORTD
#define RTS_PIN PD2

//скопировал откудато
//назначения портов и сигналов
#define PORT_SPI     PORTB
#define DDR_SPI      DDRB
#define PIN_SPI      PINB
#define SDO          PB3
#define CKL          PB5
#define CS_FH        PB0
#define CS_FL        PD7
#define CS_PWM       PB1
*/

#define	SetBit(reg, bit)		reg |= (1<<bit)
#define	ClearBit(reg, bit)		reg &= (~(1<<bit))
#define	InvBit(reg, bit)        reg ^= (1<<bit)
#define	BitIsSet(reg, bit)      ((reg & (1<<bit)) != 0)
#define	BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)

struct UartBufSt {					//структура работы с УАРТ на уровне отделенном от работы с регистрами
	uint8_t Buf[UART_BUF_SIZE];		//буффер куда приходят, откуда уходят данные
	uint8_t	N;						//количество переданных, принятых данных
	uint8_t NeedSend;				//количество байт, которыйе необходимо передать из Buf,
									//число отличное от нуля, переводит в режим передачи функцией UARTStartByRec()
	bool EndMes;					//признак конца пакета для модбаса
};

//проба еепром, пока не закончено
struct EEPROMst{
	uint8_t tmp1;
	uint16_t tmp2;
};

#endif /* MACROS_H_ */
