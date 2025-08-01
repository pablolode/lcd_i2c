/*
 * xlcd.c
 *
 *  Created on: Dec 10, 2021
 *      Author: pablolode
 */

#include "lcd_i2c.h"
#include <stdbool.h>
#include "at24_hal_i2c.h"
#include "eeprom_datos.h"
/*********inicializar en main********************
 *extern LCD_I2C_t lcd;
MX_I2C1_Init();


HAL_Delay(100);
 	LCD_Init(&lcd, &hi2c1, 0x27, 20, 4);//inicia y limpia display
	LCD_SendString(&lcd,  "     WENTUX-S4    ");
	LCD_SetCursor(&lcd, 0 ,1);//linea 1
	LCD_SendString(&lcd,  "   wentux.com.ar    ");
 	LCD_Backlight(&lcd, 1); //Backlight on

 * *******************************************/

//***TEST
 LCD_I2C_t lcd;

//Prototypes
uint8_t LCD_command(LCD_I2C_t *lcd_i2c, uint8_t command);
uint8_t LCD_send(LCD_I2C_t *lcd_i2c, uint8_t data, uint8_t flags);
uint8_t habilitadisplay4lineas;

void seleccionarDisplay(void)
{
	eeprom_ReadByte(EEP_DISPLAY_20X4, &habilitadisplay4lineas);
}

uint8_t habilitardisplay_20x4 (void)
{
	return habilitadisplay4lineas;
}

void LCD_Init(LCD_I2C_t *lcd_i2c, I2C_HandleTypeDef *handle, uint8_t addr, uint8_t cols, uint8_t rows)
{	 

    lcd_i2c->hi2c1 = handle;

	lcd_i2c->Addr = (addr << 1); //correct address
  	lcd_i2c->cols = cols;
  	lcd_i2c->rows = rows;
	lcd_i2c->numlines = rows;
  	lcd_i2c->backlight = LCD_NOBACKLIGHT;
	lcd_i2c->displaycontrol = LCD_CURSOROFF|LCD_BLINKOFF;
	HAL_Delay(50);  // wait for >40ms
	LCD_command(lcd_i2c, LCD_FUNCTIONSET|LCD_4BITMODE);    
	HAL_Delay(50);
	LCD_command(lcd_i2c, LCD_FUNCTIONSET|LCD_4BITMODE|LCD_5x8DOTS|LCD_2LINE);
	//LCD_command(lcd_i2c, LCD_FUNCTIONSET|LCD_8BITMODE|LCD_5x8DOTS|LCD_2LINE);
	HAL_Delay(50);
	LCD_command(lcd_i2c, LCD_DISPLAYCONTROL|LCD_DISPLAYOFF|LCD_CURSOROFF|LCD_BLINKOFF);
	HAL_Delay(50);
	LCD_command(lcd_i2c, LCD_CLEARDISPLAY);    
	HAL_Delay(2);
	LCD_command(lcd_i2c, LCD_ENTRYMODESET|LCD_ENTRYLEFT|LCD_ENTRYSHIFTDECREMENT);
	HAL_Delay(50);
	LCD_command(lcd_i2c, LCD_DISPLAYCONTROL|LCD_DISPLAYON|lcd_i2c->displaycontrol);

}

void LCD_Init_bug(LCD_I2C_t *lcd_i2c, I2C_HandleTypeDef *handle, uint8_t addr, uint8_t cols, uint8_t rows)
{
   
}

void LCD_Home(LCD_I2C_t *lcd_i2c)
{
	LCD_command(lcd_i2c, LCD_RETURNHOME);   
	//DELAY_MS(2);
	HAL_Delay(10);
}
//LCD_Clear (&lcd);
void LCD_Clear(LCD_I2C_t *lcd_i2c)
{
	LCD_command(lcd_i2c, LCD_CLEARDISPLAY);
	//DELAY_MS(2);
	HAL_Delay(10);
}

void LCD_Backlight(LCD_I2C_t *lcd_i2c, uint8_t backlight)
{
	if (backlight) lcd_i2c->backlight = LCD_BACKLIGHT;
	else lcd_i2c->backlight = 0;
	LCD_command(lcd_i2c, 0);
}

void LCD_Blink(LCD_I2C_t *lcd_i2c, uint8_t on)
{
	if (on) {
		lcd_i2c->displaycontrol |= LCD_BLINKON;
	} else {
		lcd_i2c->displaycontrol &= ~LCD_BLINKON;
	}
	LCD_command(lcd_i2c, LCD_DISPLAYCONTROL|LCD_DISPLAYON|lcd_i2c->displaycontrol);
}

void LCD_Cursor(LCD_I2C_t *lcd_i2c, uint8_t cur)
{
	if (cur) {
		lcd_i2c->displaycontrol |= LCD_CURSORON;
	} else {
		lcd_i2c->displaycontrol &= ~LCD_CURSORON;
	}
	LCD_command(lcd_i2c, LCD_DISPLAYCONTROL|LCD_DISPLAYON|lcd_i2c->displaycontrol);
}

void LCD_SetCursor(LCD_I2C_t *lcd_i2c, uint8_t col, uint8_t row)
{
	if ((row+1)>lcd_i2c->rows) return;

	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > (4-1) ) {
		row = 4 - 1;    // we count rows starting w/0
	}
	LCD_command(lcd_i2c, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_SendString(LCD_I2C_t *lcd_i2c, const char *str)
{
	while(*str && LCD_send(lcd_i2c, (uint8_t)(*str), 1)) {

        str++; 
    }
}

void LCD_SendStringFloat(LCD_I2C_t *lcd_i2c, const char *str)
{
	uint8_t i = 0;
 
	while(*str && LCD_send(lcd_i2c, (uint8_t)(*str), 1)) {
        str++;  
        i++;
       // if (i == 3)//solo 3 caracteres, entero, punto y decimal
          if (i == 4)//solo 3 caracteres,2 enteros, punto y decimal
        	return;
    }
}

/***** Internal communication functions ******/

/**
 * Send command to display
 * @par *lcd_i2c - Pointer to @ref LCD_I2C_t working lcd_i2c struct
 * @par command  - command
 * @retval 1 - success, 0 - error divice not responding
 */
uint8_t LCD_command(LCD_I2C_t *lcd_i2c, uint8_t command)
{
	HAL_Delay(10);
	return LCD_send(lcd_i2c, command, 0);
}

/**
 * Send data to display
 * @par *lcd_i2c - Pointer to @ref LCD_I2C_t working lcd_i2c struct
 * @par data - datas to send
 * @par flags - 1 - send datas or 0 - send command
 * @return 1 - success, 0 - error divice not responding
 */
uint8_t LCD_send(LCD_I2C_t *lcd_i2c, uint8_t data, uint8_t flags)
{
	if (lcd_i2c->hi2c1 == NULL) return 0;

	HAL_StatusTypeDef res;

	//Is the device ready by lcd_i2c->Addr?
	//Trials - 10 times
	res = HAL_I2C_IsDeviceReady(lcd_i2c->hi2c1, lcd_i2c->Addr, 10, HAL_MAX_DELAY);
	//Failed, nobody answered
	if(res != HAL_OK) return 0;

	//Split byte to upper and lower parts
	uint8_t up = data & 0xF0; //upper part
	uint8_t lo = (data << 4) & 0xF0; //lower part

	uint8_t data_arr[4];
	// 4-7 bits contains info, bits 0-3 contains configuration
	data_arr[0] = up|flags|lcd_i2c->backlight|BIT_EN;
	//  
	// send again, this tine EN is zero
	data_arr[1] = up|flags|lcd_i2c->backlight;
	//The same for configuration
	data_arr[2] = lo|flags|lcd_i2c->backlight|BIT_EN;
	data_arr[3] = lo|flags|lcd_i2c->backlight;

	res = HAL_I2C_Master_Transmit(lcd_i2c->hi2c1, lcd_i2c->Addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);

	//DELAY_MS(5);
	HAL_Delay(15);
	if (res == HAL_OK) return 1;
	else return 0;
}

/**************************************************************************/
/********funciones compatibles con compiler microchip c18************/
/**************************************************************************/
/*
 *
 *
 * void LCD_Clear(LCD_I2C_t *lcd_i2c)
{
	// очистка дисплея
	LCD_command(lcd_i2c, LCD_CLEARDISPLAY);
	DELAY_MS(2);
}
*/
void lcd_send_comand( unsigned char cmd)
{

	if (cmd == LINEA1)
		LCD_SetCursor(&lcd, 0,0);//linea 1
	else if  (cmd == LINEA2)
		LCD_SetCursor(&lcd, 0,1);//linea 2
	else if  (cmd == CLEAR)
	{
		 LCD_Clear (&lcd);//
	}
	//WriteCmdXLCD(cmd);
}
/*
void LCD_SendString(LCD_I2C_t *lcd_i2c, const char *str)
{
	// *char по сути является строкой
	while(*str && LCD_send(lcd_i2c, (uint8_t)(*str), 1)) {
        // пока строчка не закончится
		// передача первого символа строки
        str++; // сдвиг строки налево на 1 символ
    }
}
LCD_SendString(&lcd, "    WENTUX");
 * */

void lcd_write(const char *buffer)
{

	LCD_SendString(&lcd, buffer);
   //putrsXLCD(buffer);
}

void lcd_write_char( char *buffer)
{

	LCD_SendString(&lcd, buffer);
   //putsXLCD(buffer);
}
void lcd_write_float( char *buffer)
{
	LCD_SendStringFloat(&lcd, buffer);
   //putsXLCDfloat(buffer);
}

/*
void LCD_Cursor(LCD_I2C_t *lcd_i2c, uint8_t cur)
{
	if (cur) {
		lcd_i2c->displaycontrol |= LCD_CURSORON;
	} else {
		lcd_i2c->displaycontrol &= ~LCD_CURSORON;
	}
	LCD_command(lcd_i2c, LCD_DISPLAYCONTROL|LCD_DISPLAYON|lcd_i2c->displaycontrol);
}

*LCD_SetCursor(&lcd, pocicion,linea);//
*/
void lcd_posicion(unsigned char pocicion, unsigned char line )
{
	line = line -1;
	pocicion = pocicion -1;
	LCD_SetCursor(&lcd, pocicion,line);//
}
/***************************************************************/
/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 * ej:
 * static uint8_t resultHR[10];
 * static uint8_t resultTEMP[10];
 * static int8_t  resultVAR[10];
 * int32_t HR;
 *
 *     itoa( (int) HR, (char*)resultHR, 10 )
 *     lcdWrite( resultTEMP );
 *
 **/
  char* itoa_v2(int value, char* result, int base)
{

    char* ptr = result, *ptr1 = result, tmp_char;
//
//       char*      tmp_char;
//   char* ptr = result;
//    char     *ptr1 = result;
   int tmp_value;
// check that the base if valid
   if ((base < 2) || (base > 36))
   { *result = '\0'; return result; }

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

  /*----------------- FLOAT TO ASCII---------------*/

char* ftoa_v2(float f_number,char *buffer, uint8_t places)
{
    int pos = 0,
    		ix,
			dp,
			num;


    if ( f_number < 0 )
    {
        buffer[ pos++ ]='-';
        f_number = - f_number;
    }
    dp = 0;

    while ( f_number >= 10.0 )
    {
        f_number = f_number / 10.0;
        dp++;
    }
  //for ( ix = 1 ;ix < 8; ix++ )
    for ( ix = 1 ;ix < places + 3; ix++ )
    {
            num = f_number;
            f_number = f_number - num;
            if ( num > 9 )
                buffer[ pos++ ] = '#';
            else
                buffer[ pos++ ] = '0' + num;
            if ( dp == 0 )
            	buffer[ pos++ ] = '.';
            f_number = f_number * 10.0;
            dp--;
    }
    return buffer;
}

//	 	 https://github.com/ProjectsByJRP/stm32_hal_i2c_bus_scan/blob/master/Src/main.c
/*
	 	   * the HAL wants a left aligned i2c address
	 	   * &hi2c1 is the handle
	 	   * (uint16_t)(i<<1) is the i2c address left aligned
	 	   * retries 2
	 	   * timeout 2
	 	   */
/*
printf("Scanning I2C bus:\r\n");
		HAL_StatusTypeDef result;
	for (i=1; i<128; i++)
{
	 	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
	 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	 	  {
	 		  printf("."); // No ACK received at that address
	 	  }
	 	  if (result == HAL_OK)
	 	  {
	 		  printf("0x%X", i); // Received an ACK at that address
	 	  }
	 	}
	 	*/
