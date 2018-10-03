#include "stm32f10x.h" 

#define KEY1	0x01
#define KEY2	0x02
#define KEY3	0x04
#define KEY4	0x08

#define KB_FREE				0x00
#define KB_PRESS			0x10
#define KB_HOLD				0x20
#define KB_RELEASE		0x30

#define LEFT_PRESS 			(0x12|0x80)   //уф
#define LEFT_HOLD  			(0x22|0x80)
#define LEFT_RELEASE 		(0x32|0x80) 
			
#define UP_PRESS 			(0x14|0x80)   
#define UP_HOLD  			(0x24|0x80)
#define UP_RELEASE 		(0x34|0x80) 

#define RIGHT_PRESS 			(0x11|0x80)   //узг кнопка
#define RIGHT_HOLD  			(0x21|0x80)
#define RIGHT_RELEASE 		(0x31|0x80) 

#define DOWN_PRESS 			(0x18|0x80)   
#define DOWN_HOLD  			(0x28|0x80)
#define DOWN_RELEASE 		(0x38|0x80) 

#define ENTER_PRESS 			(0x1C|0x80)   
#define ENTER_HOLD  			(0x2C|0x80)
#define ENTER_RELEASE 		(0x3C|0x80)

void Keyboard(void);
