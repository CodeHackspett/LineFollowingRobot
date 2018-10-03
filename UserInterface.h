/*
 * UserInterface.c
 *
 * Created: 2014-10-01 10:14:27
 * Author: Md Reaz Ashraful Abedin, Mattia Picchio and Dawit Kahsay 
*/

/************************************* Include Files *************************************/
#include "i2cmaster.h"

/*****************************************************************************************/


/*************************************** Constants ***************************************/
#define LCDdisplay  0xC6
#define LCD_ROW_LENGTH 16
#define LCD_COLUMN_LENGTH 2
#define LCD_COMMAND_REGISTER 0
#define LCD_KP_LOW_REGISTER 1
#define LCD_KP_HIGH_REGISTER 2
#define TURN_ON_BACKLIGHT  19
#define TURN_OFF_BACKLIGHT  20
#define SHOW_UNDERLINE_CURSOR  5
#define SHOW_BLINKING_CURSOR  6
#define SET_CURSOR  2
#define HIDE_CURSOR  4
#define CLEAR_SCREEN  12
#define DOUBLE_KEYPAD_SCAN  28
#define NORMAL_KEYPAD_SCAN  29

/*****************************************************************************************/


/*********************************** Global Variables ************************************/
const int TRUE = 1, FALSE = 0;
const int HIDE = 2, UNDERLINE = 1, BLINKING = 0;

/*****************************************************************************************/




/*************************************** Functions ***************************************/
/*
 * Function Name: LCD_Command
 * Access: public
 * Params: val the value to be written to the LCD
 * Returns: void
 * Description: writes the value to the LCD
 */ 
void LCD_Command(int val)
{
	i2c_start_wait(LCDdisplay+I2C_WRITE);     // set device address and write mode
	i2c_write(LCD_COMMAND_REGISTER);
	i2c_write(val);
	i2c_stop();                             // set stop condition = release bus	
}


/*
 * Function Name: LCD_setBacklight
 * Access: public
 * Params: status On or Off
 * Returns: void
 * Description: turns on or off the back-light
 */ 
void LCD_setBacklight(int status)
{	
	if (status == TRUE)
	{
		LCD_Command(TURN_ON_BACKLIGHT);
	}
	else
		LCD_Command(TURN_OFF_BACKLIGHT);
}


/*
 * Function Name: LCD_setCursor
 * Access: public
 * Params: status Underline, Blinking or Hide
 * Returns: void
 * Description: shows or hides the cursor (underline or blinking)
 */ 
void LCD_setCursor(int status)
{   
	if (status == UNDERLINE)
		LCD_Command(SHOW_UNDERLINE_CURSOR);
	else if(status == BLINKING)
		LCD_Command(SHOW_BLINKING_CURSOR);
	else
		LCD_Command(HIDE_CURSOR);
}


/*
 * Function Name: LCD_goToPosition
 * Access: public
 * Params: position the place where the cursor will move
 * Returns: void
 * Description: moves the cursor to the desired position
 */ 
void LCD_goToPosition(int position)
{
	LCD_Command(SET_CURSOR);
	LCD_Command(position);
}


/*
 * Function Name: LCD_clearScreen
 * Access: public
 * Params: none
 * Returns: void
 * Description: clears the screen
 */ 
void LCD_clearScreen(void)
{
	LCD_Command(CLEAR_SCREEN);
}

/*
 * Function Name: LCD_printChar
 * Access: public
 * Params: c a character to be printed
 * Returns: void
 * Description: prints the character in the LCD
 */ 
void LCD_printChar(char c)
{
	LCD_Command(c);
}


/*
 * Function Name: LCD_printString
 * Access: public
 * Params: str a string of characters to be printed 
 * Returns: void
 * Description: prints the string of characters to the LCD
 */ 
void LCD_printString(unsigned char *str)
{
	unsigned char i = 0;
	while(str[i] != 0x00)
	{
		LCD_printChar(str[i]);
		i++;
	}
}


/*
 * Function Name: LCD_printNumber
 * Access: public
 * Params: num an integral number to be printed 
 * Returns: void
 * Description: prints the integral number to the LCD
 */ 
void LCD_printNumber(long num)
{
char buffer[15];
ltoa(num, buffer, 10);
LCD_printString(buffer);
}


/*
 * Function Name: LCD_getKeypadInput
 * Access: public
 * Params: none
 * Returns: char
 * Description: reads the character corresponding to the user input
 *				from the keypad. 
 */
char LCD_getKeypadInput(void)
{
	i2c_start_wait(LCDdisplay+I2C_WRITE);     // set device address and write mode
	i2c_write(LCD_KP_LOW_REGISTER);
	i2c_rep_start(LCDdisplay+I2C_READ);
	int keypadLow = i2c_readNak();
	i2c_stop();
	
	i2c_start_wait(LCDdisplay+I2C_WRITE);     // set device address and write mode
	i2c_write(LCD_KP_HIGH_REGISTER);
	i2c_rep_start(LCDdisplay+I2C_READ);
	int keypadHigh = i2c_readNak();
	i2c_stop();
			
	for(int i=0; i<4; i++)
	{
		switch(keypadHigh&(1<<i))
		{
			case 0b00000001:
							return('9');
			break;
			case 0b00000010:
							return('*');
			break;
			case 0b00000100:
							return('0');
			break;
			case 0b00001000:
							return('#');
			break;
			default:
					;
		}		
	}
	
	for(int i=0; i<8; i++)
	{
		switch(keypadLow&(1<<i))
		{
			case 0b00000001:
			return('1');
			break;
			case 0b00000010:
			return('2');
			break;
			case 0b00000100:
			return('3');
			break;
			case 0b00001000:
			return('4');
			break;
			case 0b00010000:
			return('5');
			case 0b00100000:
			return('6');
			case 0b01000000:
			return('7');
			break;
			case 0b10000000:
			return('8');
			break;
			default:
			;
		}
	}	
	return('\0');
}

/*****************************************************************************************/