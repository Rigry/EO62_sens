#include "kb.h"

extern uint8_t kb;
void Keyboard(void)
{
	uint8_t keys,temp;
	static uint8_t kb_event;
	static uint16_t kb_time;
	uint16_t i;
	kb_event&=0x7F;
	while(1)
	{
		keys=(GPIOB->IDR & 0x03); //Получаем нажатые кнопки от PB[0,1]
		keys|=((GPIOB->IDR>>6)&0x0C);//Получаем нажатые кнопки от PB[8,9]
		for (i=0;i<0xFFF;i++); //ждем дребезг контактов
		temp=(GPIOB->IDR & 0x03); //Получаем нажатые кнопки от PB[0,1]
		temp|=((GPIOB->IDR>>6)&0x0C);//Получаем нажатые кнопки от PB[8,9]
		if (keys==temp) break;
	}
	
		
	switch (kb_event&0x70)
	{
			case KB_FREE:
				if(keys)
			{
				kb_event&=0x0F;
				kb_event=KB_PRESS;
				kb_event|=0x80;
				kb_event|=keys;
				kb_time=1000;
				
			}
			break;
			
			case KB_PRESS:
			if(!keys)
			{
				kb_event&=0x0F;
				kb_event|=KB_RELEASE;			
				kb_event|=0x80;
			}
			else
			{	
				if(keys==(kb_event&0x0F))
				{
					if(!kb_time)
					{
						kb_event&=0x0F;
						kb_event|=KB_HOLD;
						kb_event|=0x80;
						kb_time=2000 ;
					}
					else kb_time--;
				}
				
				
			}
			break;
			
			case KB_RELEASE:
			if(keys)
			{
				kb_event&=0x0F;
				kb_event=KB_PRESS;
				kb_event|=0x80;
				kb_event|=keys;
			}
			else
			{
				kb_event&=0x0F;
				kb_event=KB_FREE;
			}
			break;
			
			case KB_HOLD:
				if(!keys)
					{
						kb_event&=0x0F;
						kb_event|=KB_RELEASE;
						kb_event|=0x80;
					}
						
				else
				{
					if(keys==(kb_event&0x0F))
						{
							if(!kb_time)
							{
								kb_event&=0x0F;
								kb_event|=KB_HOLD;
								kb_event|=0x80;
								kb_time=2000 ;
							}
							else kb_time--;
						}
						
				}	
			break;
			
	}		
	kb=kb_event;
	
}


