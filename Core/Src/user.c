/*
 * user.c
 *
 *  Created on: Jun 11, 2022
 *      Author: kon
 */

#include <user.h>

int menu( const char* entries[], const unsigned entries_no )
{
	static int choice = 0, flag = 0;

	while( !flag )
	{
		 BSP_LCD_GLASS_ExitableScrollSentence( (uint8_t*) entries[choice], 1, 100);
		 //while( !enc_state );

		 switch( enc_state )
		 {
		 case NOP: break;
		 case BUTTON: flag = 1; break;
		 case LEFT: --choice; break;
		 case RIGHT: ++choice; break;
		 default: break;
		 }
		 choice = choice < 0 ? entries_no-1  : choice;
		 choice %= entries_no;
		 enc_state = NOP;
	}

	flag = 0;
	BSP_LCD_GLASS_Clear();
	return choice;
}

void lcd_put_v( float voltage )
{
	uint8_t text[8];

	sprintf( (char*)text, "%1.3fV", voltage );
	BSP_LCD_GLASS_DisplayString(text);
}

void lcd_put_a( float curr )
{
	uint8_t text[8];

	sprintf( (char*)text, "%1.3fA", curr );
	BSP_LCD_GLASS_DisplayString(text);
}

void lcd_put_nm( float torq )
{
	uint8_t text[8];

	sprintf( (char*)text, "%04.2fNm", torq );
	BSP_LCD_GLASS_DisplayString(text);
}

void lcd_put_speed( float rpm )
{
	uint8_t text[8];

	sprintf( (char*)text, "%06.1f", rpm > MAX_RPM ? 0 : rpm );
	BSP_LCD_GLASS_DisplayString(text);
}

void BSP_LCD_GLASS_ExitableScrollSentence(uint8_t *ptr, uint16_t nScroll, uint16_t ScrollSpeed)
{
  uint8_t repetition = 0, nbrchar = 0, sizestr = 0;
  uint8_t *ptr1;
  uint8_t str[6] = "";

  /* Reset interrupt variable in case key was press before entering function */
  enc_state = NOP;

  if (ptr == 0) return;

  /* To calculate end of string */
  for (ptr1 = ptr, sizestr = 0; *ptr1 != 0; sizestr++, ptr1++);

  ptr1 = ptr;

  BSP_LCD_GLASS_DisplayString(str);
  HAL_Delay(ScrollSpeed);

  /* To shift the string for scrolling display*/
  for (repetition = 0; repetition < nScroll; repetition++)
  {
    for (nbrchar = 0; nbrchar < sizestr; nbrchar++)
    {
      *(str) = * (ptr1 + ((nbrchar + 1) % sizestr));
      *(str + 1) = * (ptr1 + ((nbrchar + 2) % sizestr));
      *(str + 2) = * (ptr1 + ((nbrchar + 3) % sizestr));
      *(str + 3) = * (ptr1 + ((nbrchar + 4) % sizestr));
      *(str + 4) = * (ptr1 + ((nbrchar + 5) % sizestr));
      *(str + 5) = * (ptr1 + ((nbrchar + 6) % sizestr));
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString(str);

      /* user button pressed stop the scrolling sentence */
      if (enc_state) return;
      HAL_Delay(ScrollSpeed);
    }
  }
  while( !enc_state );
}
