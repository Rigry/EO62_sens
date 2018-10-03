#include "stm32f10x.h"


void delay_ms (uint16_t value);
void lcd_write_4bit (uint8_t c);
void lcd_write_cmd (unsigned char c);
static void lcd_write_data (unsigned char c);
void lcd_putchar (char c);
void lcd_init (void);
void set_cursor (int column, int line);
void lcd_clear (void);
void lcd_print (char *string);
static uint8_t lcd_rus(uint8_t c);
void delay_func_init(void);



