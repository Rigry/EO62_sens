#include "display.h"

#define RS					GPIO_Pin_12
#define RW  				GPIO_Pin_2
#define E						GPIO_Pin_3
#define RS_PORT 		GPIOC->ODR
#define RW_PORT			GPIOD->ODR
#define E_PORT			GPIOB->ODR
#define RS_1				RS_PORT|=RS
#define RS_0				RS_PORT&=~RS

#define RW_1				RW_PORT|=RW
#define RW_0				RW_PORT&=~RW

#define E_1					E_PORT|=E
#define E_0					E_PORT&=~E

#define DATA_PORT		GPIOB->ODR

//GPIO_InitTypeDef  GPIO_InitStructure;
//Таблица перекодировки в русские символы.
static const unsigned char  convert_HD44780[64] =
{
	0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,
	0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8,
	0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,
	0xAC,0xE2,0xAD,0xAE,0xAD,0xAF,0xB0,0xB1,
	0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,
	0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE,
	0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,
	0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7
};



void delay_func_init(void)
{
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 8000-1;
    TIM4->CR1 |= TIM_CR1_OPM;

}
void delay_ms (uint16_t value)
{
    TIM4->ARR = value;
    TIM4->CNT = 0;
    TIM4->CR1 = TIM_CR1_CEN;
    while((TIM4->SR & TIM_SR_UIF)==0){} // ждем конца задержки
  TIM4->SR &= ~TIM_SR_UIF;
}

void lcd_write_4bit (uint8_t c)
{
  RW_0;
	DATA_PORT = (DATA_PORT & ~0x00F0) | ((c<<4)&0xF0); 
  E_1;
  delay_ms(5);
  E_0;
  delay_ms(5);
}

void lcd_write_cmd (unsigned char c)
{
  delay_ms(1);

  RS_0;
  lcd_write_4bit (c>>4);
  lcd_write_4bit (c);
}

static void lcd_write_data (unsigned char c)
{
  delay_ms(1);

  RS_1;
  lcd_write_4bit (c>>4);
  lcd_write_4bit (c);
}

void lcd_putchar (char c)
{ 
  c=lcd_rus(c);
	lcd_write_data (c);
}

void lcd_init (void)
{ 
  
	//int i;
  //char const *p;

  
  delay_ms (50);
  RS_0;
  lcd_write_4bit (0x3); /* func set             */
  delay_ms (50);
  lcd_write_4bit (0x3);	//func set
	delay_ms (10);
	lcd_write_4bit (0x3);//funct set
	delay_ms(10);
	lcd_write_4bit (0x2);
	delay_ms(10);
	
  lcd_write_cmd (0x28);                 /* 2 lines, 5x8 character matrix      */
  lcd_write_cmd (0x0C);                 /* Display ctrl:Disp=ON,Curs/Blnk=OFF */
  lcd_write_cmd (0x01);    //clear             
	lcd_write_cmd (0x06);/* Entry mode: Move right, no shift   */
  /* Load user-specific characters into CGRAM                                 */
 // lcd_write_cmd(0x40);                  /* Set CGRAM address counter to 0     */
 // p = &UserFont[0][0];
  //for (i = 0; i < sizeof(UserFont); i++, p++)
  //  lcd_putchar (*p);

  lcd_write_cmd(0x80);                  /* Set DDRAM address counter to 0     */
}

void set_cursor (int column, int line)
{
  unsigned char address;
	
	switch (line)
	{
		case 0:
			{
				address = 0x00 + column;
			}
			break;
		case 1:
			{
				address = 0x40 + column;
			}
			break;
		case 2:
			{
				address = 0x14 + column;
			}
			break;
		case 3:
			{
				address = 0x54 + column;
			}
			break;
		default:
		 {
			 address = 0x00 + column;
		 }
	}
  //address = (line * 40) + column;
  address = 0x80 + (address & 0x7F);
  lcd_write_cmd(address);               /* Set DDRAM address counter to 0     */
}

void lcd_clear (void)
{
  lcd_write_cmd(0x01);                  /* Display clear                      */
  set_cursor (0, 0);
}




void lcd_print (char *string)
{
  while (*string)  {
    lcd_putchar (*string++);
  }
}

static uint8_t lcd_rus(uint8_t c)
{

	if  (c > 191)
	{
		c -=192;
		c=convert_HD44780[c];
	}

	return c;
}



