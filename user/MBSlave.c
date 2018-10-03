/*
 * MBSlave.c
 *
 * Created: 07.02.2017 15:18:22
 *  Author: dvk
 */ 

#include "MBSlave.h"
#include "stm32f10x.h"		//нужен только для отладки


#define MBFUNC03	(Buf->Buf[1] == 3)		//из спецификации на модбас чтение регистров мастером
#define MBFUNC16	(Buf->Buf[1] == 16)		//из спецификации на модбас установка регистров мастером
#define OURADDR		(Buf->Buf[0] == Addr)
#define ALLADDR		(Buf->Buf[0] == 0)		//широковещательный запрос

#define LIGHTON		GPIOB->BSRR = GPIO_Pin_15;	//Включаем Индикатор активности на шине

void MBSlave(struct UartBufSt *Buf,struct MBRegSt *Reg, uint8_t Addr)
{
	uint16_t LowAddr = 0,RegQty = 0,HighAddr = 0;
	uint16_t RegVal[QTY_OUT_REG];
	uint16_t CRCF;
	uint8_t CRC1st;
	uint8_t CRC2nd;
	enum ErrorCode{	//из спецификации на модбас
		NoErr		= 0,
		FuncErr 	= 1,
		RegErr		= 2,
		ValueErr	= 3
	} Err = NoErr;
	enum MBSlaveStep{	//по порядку в switch
		StartCheck,
		MinMesCheck,
		AddrCheck,
		FuncCheck,
		RegCheck,
		ValueCheck,
		CRCCheck,
		AnswerErr,
		AnswerMB03,
		AnswerMB16,
		AnswerCRC,
		FuncDone,
	} Step = StartCheck;	
	
	uint16_t i;
		
	while (Step!=FuncDone) {
		switch (Step) {
		case StartCheck:
			if (Buf->EndMes) {
				Step = MinMesCheck;
			} else {
				Step = FuncDone;
			}
			break;
		case MinMesCheck:
			if (Buf->N < 8) {	//минимальный размер пакета по спецификации ModBus
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone;
			} else {
				Step = AddrCheck;
			}	
			break;		
		case AddrCheck:
			if (OURADDR || ALLADDR) {
				Step = FuncCheck;
			} else {
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone;
			}
			break;
		case FuncCheck:
			if (MBFUNC03 || MBFUNC16) {
				Step=RegCheck;
			} else {
				Err = FuncErr;
				Step = CRCCheck;
			}
			break;	
		case RegCheck:
			//все числа по спецификации Modbus
			LowAddr = (uint16_t)Buf->Buf[2] * 256 + Buf->Buf[3];
			RegQty = (uint16_t)Buf->Buf[4] * 256 + Buf->Buf[5];
			HighAddr = LowAddr + RegQty - 1;		
			if (MBFUNC16) {
				if (Buf->N != RegQty * 2 + 9) {	//длина пакета не соответсвует спецификации ModBus
					Buf->EndMes = false;
					Buf->N = 0;
					Step = FuncDone;
				} else if (HighAddr > (QTY_IN_REG - 1)) {
					Err = RegErr;
					Step = CRCCheck;
				} else {
					Step = ValueCheck;
				} 
			} else if (MBFUNC03) {
				//КОСТЫЛЬ: отчего то на 1 байт больше принимал
				if ( !( (Buf->N == 8) || (Buf->N == 9) ) ) {	//длина пакета не соответсвует спецификации ModBus
					Buf->EndMes = false;
					Buf->N = 0;
					Step = FuncDone;
				} else if (HighAddr > (QTY_OUT_REG - 1)) {
					Err = RegErr;
					Step = CRCCheck;
				} else {
					Step = CRCCheck;					
				}
			}
			break;	
		case ValueCheck:
			if (MBFUNC16) {
				bool AllGood = true;
				for (i = 0; i < RegQty; i++) {
					RegVal[i] = (uint16_t)Buf->Buf[i*2+7] * 256 + Buf->Buf[i*2+8];
					AllGood = AllGood && RegVal[i] >= Reg->RegInMinVal[i+LowAddr]
									  && (RegVal[i] <= Reg->RegInMaxVal[i+LowAddr] 
									  	  || Reg->RegInMaxVal[i+LowAddr] == 0);
				}
				Err = AllGood ? Err : ValueErr;
				Step = CRCCheck;
			} else {
				Step=CRCCheck;
			}
			break;	
		case CRCCheck:
			CRCF = crc16(&Buf->Buf[0], Buf->N - 2);
			CRC1st = CRCF % 256;
			CRC2nd = CRCF / 256;
			if ((CRC1st != Buf->Buf[Buf->N-2]) || (CRC2nd != Buf->Buf[Buf->N-1])) {
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone;
			} else {
				if (Err != NoErr) {
					Step = AnswerErr;
				} else if (MBFUNC03) {
					Step = AnswerMB03;
				} else if (MBFUNC16) {
					Step = AnswerMB16;
				} else {
					Step=FuncDone; 
				}
			}	
			break;	
		case AnswerErr:
			if (ALLADDR) {
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone; 
			} else {
				SetBit(Buf->Buf[1], 7);	//из спецификации MODBUS
				Buf->Buf[2] = Err;
				Buf->NeedSend = 3;		//адрес, функция, код ошибки
				Step = AnswerCRC; 
			}
			break;	
		case AnswerMB03:
			if (ALLADDR) {
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone;
			} else {
				Buf->Buf[2] = RegQty * 2;
				for (i = 0; i < RegQty; i++) {
					Buf->Buf[2*i+3] = Reg->RegOut[LowAddr+i] / 256;
					Buf->Buf[2*i+4] = Reg->RegOut[LowAddr+i] % 256;	
				}
				Buf->NeedSend = 3 + Buf->Buf[2];	//адрес, функция, qty байт данных, данные (в количестве qty	байт)
				Step = AnswerCRC;
			}
			break;
		case AnswerMB16:
			for (i = 0; i < RegQty; i++) {
				Reg->RegIn[LowAddr+i] = RegVal[i];
			}
			if (ALLADDR) {
				Buf->EndMes = false;
				Buf->N = 0;
				Step = FuncDone;
			} else {
				Buf->NeedSend = 6;	//ответ как запрос, только обрезаем данные
				Step = AnswerCRC;
			}
			break;	
		case AnswerCRC:
			CRCF = crc16(&Buf->Buf[0], Buf->NeedSend);
			Buf->Buf[Buf->NeedSend] = CRCF % 256;
			Buf->Buf[Buf->NeedSend+1] = CRCF / 256;
			Buf->NeedSend = Buf->NeedSend + 2;
			Buf->EndMes = false;
			Buf->N = 0;
			Step = FuncDone;
			break;										
		}//switch (Step) 
	}//while (Step != FuncDone) 
	return;
}//void MBSlave()

//это с форума ОВЕНа - для ARM
uint16_t crc16(uint8_t* data, uint8_t length)
{
	int j;
	uint16_t reg_crc = 0xFFFF;
	while(length--)	{
		reg_crc ^= *data++;
		for(j = 0; j < 8; j++) {
			if (reg_crc & 0x01) {
				reg_crc=(reg_crc>>1) ^ 0xA001; // LSB(b0)=1
			} else {
				reg_crc=reg_crc>>1;
			}	
		}
	}
	return reg_crc;
}
//это я нашел на просторах инета, для атмеги
/*
uint16_t crc16(uint8_t *p, uint8_t len) 
{	
	uint16_t crc;
	uint8_t n;
	if (len > 256U) return (0);
	n = (unsigned char)len;
	crc = 0xffff;
	do {
		crc = _crc16_update(crc, *p++);  // uint16_t _crc16_update(uint16_t __crc, uint8_t __data)
	}
	while (--n);                         // pass through message buffer (max 256 items)
	return (crc);
}
*/
