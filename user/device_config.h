//Количество ламп подключенных к базовой плате
#define	LAMPS_INST	(((GPIOB->IDR)>>12)&0x0F)


//маска для проверки включенных ламп.
static const int  LAMPS_MASK[10] =
{
	0x01, 0x03,0x07,0x0f,0x1f,0x3f,0x7f,0xff,0x1ff,0x3ff
};

