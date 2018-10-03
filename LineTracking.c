/*
 * LineTrackingRobot.c
 *
 * Created: 2014-10-01 11:05:34
 * Author: Md Reaz Ashraful Abedin, Mattia Picchio and Dawit Kahsay 
 */ 

// Micro-controller clock frequency
#define F_CPU 8000000UL

/************************************* Include Files *************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "i2cmaster.h"
#include "UserInterface.h"
/*****************************************************************************************/



/*************************************** Constants ***************************************/

//User interface I2C bus address
#define DISPLAY address

//Motor drive bus/register addresses
#define MOTOR_DRIVE  0xB0	//MD25 motor drive I2C bus address
#define MODE 0x0F			//Operation Mode register
#define COMMAND 0x10		//Command register
#define LEFT_SPEED 0		//Speed register for left motor
#define RIGHT_SPEED 1		//Speed register for right motor
#define ACCELERATION 0x0E	//Acceleration register 

//Speed values
#define SLOW_SPEED 159
#define MED_SPEED 180
#define HIGH_SPEED 200

//Moving direction
#define CW 0
#define CCW 1

//Turning Direction
#define LEFT 1
#define RIGHT 2

//EEPROM address for the PID coefficients, 
//max lap and desired speed
#define EEPROM_ADD_P 0
#define EEPROM_ADD_I 1
#define EEPROM_ADD_D 2
#define EEPROM_ADD_MAX_LAP 3
#define EEPROM_SPEED_ADD 4

//Left motor encoder registers addresses
#define EN1_FIRST_BYTE 2
#define EN1_SECOND_BYTE 3
#define EN1_THIRD_BYTE 4
#define EN1_FOURTH_BYTE 5

//Right motor encoder registers addresses
#define EN2_FIRST_BYTE 6
#define EN2_SECOND_BYTE 7
#define EN2_THIRD_BYTE 8
#define EN2_FOURTH_BYTE 9
/*****************************************************************************************/


/*********************************** Global Variables ************************************/

volatile int max_laps = 5; //maximum lap count
volatile int lap_count = 0; //current lap count

//default desired/reference speed and the associated PID values
volatile int ref_speed = 180; 
volatile float Kp = 8; 
volatile float Ki = 0; 
volatile float Kd = 1.5;

volatile int direction = CCW;
volatile int last_error = 0; // previous offset of the IR sensors from the line
volatile int slow_flag = 0;

volatile int robot_distance = 0;
int32_t prev_en1_val = 0;
int32_t prev_en2_val = 0;

int lap_count_timer_val = 0;
int start_lap_count_flag = 1;
int time_count = 0;

/*****************************************************************************************/


/************************************ Main Function **************************************/
/* After making all the necessary initial configurations by calling the initialize 
 * function, the program displays the Main UI to the user.
*/
int main(void)
{
	initialize();
	display_main_menu();
	
}

/*****************************************************************************************/


/******************************* Initialization Functions ********************************/


/*
 * Function Name: initialize
 * Access: public
 * Params: none
 * Returns: void
 * Description: It initializes all the processors so that the core functions will work. 
 *				This function in turn calls four other functions which initialize the 
 *				the i2c communication, UI, IR and motor modules respectively. 
 */
void initialize()
{
	i2c_init(); 
	_delay_ms(500);
	initialize_ui();
	initialize_ir();
	initialize_motor_drive();	
}

/*
 * Function Name: initialize_lcd
 * Access: public
 * Params: none
 * Returns: void
 * Description: initializes the LCD, displays the "Line Follower Robot
 *				Group-5" for two seconds and configures Timer 1 as the display 
 *				refresh timer
 */ 
void initialize_ui()
{
	LCD_clearScreen();
	LCD_setBacklight(TRUE);
	
	LCD_goToPosition(1);
	LCD_printString(" Line  Follower ");
	LCD_goToPosition(17);
	LCD_printString(" Robot  Group-5 ");
	LCD_setCursor(HIDE);
	_delay_ms(2000);
	
	//configures Timer 1 as the display refresh timer
	TCCR1B |= (1 << CS12);		//1024
	TCNT1 = 0;					//initialize counter
	TIMSK1 |= (1 << TOIE1);		//enable overflow interrupt
}


/*
 * Function Name: initialize_ir
 * Access: public
 * Params: none
 * Returns: void
 * Description: configures Port D pins of the main processor as inputs
 *				which are in turn connected to the sensor outputs
 */ 
void initialize_ir()
{
	DDRD = 0b00000000; 
}


/*
 * Function Name: initialize_motor_drive
 * Access: public
 * Params: none
 * Returns: void
 * Description: initializes the motor drive processor.
 */ 
void initialize_motor_drive()
{
	write_this_to(0, MODE); //set the operation mode register to Mode 0 (0x00)
	
	write_this_to(48, COMMAND); //command register, disable automatic speed regulation (48=0x30)
	
	write_this_to(50, COMMAND); //command register, disable 2s timeout of motors  (50=0x32)
	
	write_this_to(128, LEFT_SPEED); //stop the left motor (128 = 0x80)
	
	write_this_to(128, RIGHT_SPEED); //stop the right motor (128 = 0x80)
	
	write_this_to(5, ACCELERATION); //set the acceleration register to 5 (0x05)
	
}

/*****************************************************************************************/


/******************************* User Interface Functions ********************************/

/*
 * Function Name: display_main_menu
 * Access: public
 * Params: none
 * Returns: void
 * Description: displays the Main UI and listens for user choice
 */ 
void display_main_menu()
{
	LCD_clearScreen();
	LCD_setCursor(HIDE);
	LCD_goToPosition(1);
	LCD_printString("  1 -> Start  ");
	LCD_goToPosition(17);
	LCD_printString(" 2 -> Settings ");
	
	input_complete();
	main_menu_handler();
}


/*
 * Function Name: main_menu_handler
 * Access: public
 * Params: none
 * Returns: void
 * Description: reads which menu from the Main UI the user selects
 *				and displays the corresponding window by calling the 
 *				associated function.
 */ 
void main_menu_handler()
{
	char input = LCD_getKeypadInput();
	while(!(input == '1' || input == '2'))
	{
		input = LCD_getKeypadInput();
	}
	
	input_complete();
	if(input == '1')
	{
		move_robot();
	}
	else if(input == '2')
	{
		display_settings();
	}
	
}


/*
 * Function Name: display_settings
 * Access: public
 * Params: none
 * Returns: void
 * Description: displays the Settings UI and listens for user choice
 */ 
void display_settings()
{
	LCD_clearScreen();
	LCD_setCursor(HIDE);
	LCD_goToPosition(1);
	LCD_printString("1->Speed  2->PID");
	LCD_goToPosition(17);
	LCD_printString("3->Laps  #->Back");
	
	input_complete();
	settings_handler();
}


/*
 * Function Name: settings_handler
 * Access: public
 * Params: none
 * Returns: void
 * Description: reads which menu from the Settings UI the user selects
 *				and displays the corresponding window by calling the 
 *				associated function.
 */ 
void settings_handler()
{
	char input = LCD_getKeypadInput();
	while(!((input == '1') || (input == '2') || (input == '3') || (input == '#')))
	{
		input = LCD_getKeypadInput();
	}
	
	input_complete();
	if(input == '1')
	{
		speed_settings();
	}
	else if(input == '2')
	{
		pid_settings();
	}
	else if(input == '3')
	{
		lap_settings();
	}
	else if(input == '#')
	{
		display_main_menu();
		return;
	}
	
}



/*
 * Function Name: speed_settings
 * Access: public
 * Params: none
 * Returns: void
 * Description: sets the value of the desired robot speed according to  
 *				what the user chooses and writes the values to the EEPROM 
 *				of the main processor. The user can choose three speed modes
 *				Slow, Medium or Fast. The best PID coefficients for these values 
 *				is automatically written to the EEPROM of the main processor for
 *				optimal performance. The user is of course free to change anyone
 *				of these values although good performance is not guaranteed.				
 */ 
void speed_settings()
{
	LCD_clearScreen();
	LCD_goToPosition(17);
	LCD_printString("*->Ok  #->Back");
	LCD_goToPosition(1);
	LCD_printString("1->Slow ");
	LCD_printString("2->Medium");
	LCD_goToPosition(17);
	LCD_printString("3->Fast  ");
	LCD_printString("#->Back");
	
	char input = LCD_getKeypadInput();
	while(!((input == '1') || (input == '2') || (input == '3') || (input == '#')))
	{
		input = LCD_getKeypadInput();
	}
	
	input_complete();
	if(input == '1')
	{
		ref_speed = SLOW_SPEED;
		Kp = 10;
		Ki = 0;
		Kd = 0;		
		slow_flag = 1; // To activate lap counting and displaying as well as displaying the current direction
	}
	else if(input == '2')
	{
		ref_speed = MED_SPEED;
		Kp = 8;
		Ki = 0;
		Kd = 2;
		slow_flag = 0;
	}
	else if(input == '3') //race speed
	{
		ref_speed = HIGH_SPEED;
		Kp = 8;
		Ki = 0;
		Kd = 3;
		slow_flag = 0;
	}
	
	if(input != '#')
	{
		print_success();
		eeprom_write_byte((uint8_t*)EEPROM_SPEED_ADD, ref_speed);
		eeprom_write_byte((uint8_t*)EEPROM_ADD_P, Kp);
		eeprom_write_byte((uint8_t*)EEPROM_ADD_I, Ki);
		eeprom_write_byte((uint8_t*)EEPROM_ADD_D, Kd);
	}
	display_settings();
	return;
}



/*
 * Function Name: pid_settings
 * Access: public
 * Params: none
 * Returns: void
 * Description: sets the value of the PID coefficients according to the 
 *				new values from a user and writes the values to the EEPROM 
 *				of the main processor 				
 */ 
void pid_settings()
{
	LCD_clearScreen();
	LCD_goToPosition(17);
	LCD_printString("*->Ok  #->Back");
	LCD_goToPosition(1);
	LCD_printString("P=");
	LCD_setCursor(BLINKING);
	
	input_complete();
	int value = new_value();
	if(value == -1)
	{
		display_settings();
		return;
	}
	Kp = value;
	eeprom_write_byte((uint8_t*)EEPROM_ADD_P, Kp);
	
	LCD_goToPosition(6);
	LCD_printString("I=");
	LCD_setCursor(BLINKING);
	value = new_value();
	if(value == -1)
	{
		display_settings();
		return ;
	}
	Ki = value;
	eeprom_write_byte((uint8_t*)EEPROM_ADD_I, Ki);
	
	LCD_goToPosition(11);
	LCD_printString("D=");
	LCD_setCursor(BLINKING);
	value = new_value();
	if(value == -1)
	{
		display_settings();
		return;
	}
	Kd = value;
	eeprom_write_byte((uint8_t*)EEPROM_ADD_D, Kd);
	print_success();
	display_settings();
	return;
}



/*
 * Function Name: lap_settings
 * Access: public
 * Params: none
 * Returns: void
 * Description: sets the value of the maximum laps to go in one
 *				direction according to the new values from a user
 *				and writes the value to the EEPROM of the main processor
 */ 
void lap_settings()
{
	LCD_clearScreen();
	LCD_goToPosition(17);
	LCD_printString("*->Ok  #->Back");
	LCD_goToPosition(1);
	LCD_printString("Max Laps=");
	LCD_setCursor(BLINKING);
	
	input_complete();
	int value = new_value();
	if(value != -1)
	{
		max_laps = value;
		print_success();
		eeprom_write_byte((uint8_t*)EEPROM_ADD_MAX_LAP, max_laps);
	}
	display_settings();
	return;
}


/*
 * Function Name: new_value
 * Access: public
 * Params: none
 * Returns: int the value read or -1 (back key)
 * Description: reads one or two digit value from the keypad and returns the 
 *				value after the ok key (*) is pressed. It returns -1 if the 
 *				back key (#) is pressed at any stage to inform that the user
 *				didn't change any value.
 */ 
int new_value()
{
	int new_value = 0;
	char first = LCD_getKeypadInput();
	while(first == '\0')
	{
		first = LCD_getKeypadInput();
	}
	input_complete();
	
	if((first == '#') || (first == '*')) //Back or OK key
	{
		return -1;
	}
	
	LCD_printChar(first);
	new_value = new_value + (first - 48); //convert from ASCII to decimal
	char second = LCD_getKeypadInput();
	while(second == '\0')
	{
		second = LCD_getKeypadInput();
	}
	
	input_complete();
	if(second == '#') //Back
	{
		return -1;
	}
	else if (second == '*') //Ok
	{
		return new_value;
	}
	new_value = new_value * 10;
	LCD_printChar(second);
	LCD_setCursor(HIDE);
	new_value = new_value + (second - 48);
	char confirm = LCD_getKeypadInput();
	while(!((confirm == '#') || (confirm == '*')))
	{
		confirm = LCD_getKeypadInput();
	}
	
	input_complete();
	if(confirm == '#') //Back
	{
		return -1;
	}
	else if (confirm == '*') //Ok
	{
		return new_value;
	}
}


/*
 * Function Name: display_static_data
 * Access: public
 * Params: none
 * Returns: void
 * Description: displays the static part of the robot
 *				parameter values
 */ 
void display_static_data()
{
	LCD_clearScreen();
	LCD_goToPosition(1);
	
	LCD_printString("L=  /"); // L -> Laps
	//reads the max lap from the EEPROM and displays it
	max_laps = eeprom_read_byte(EEPROM_ADD_MAX_LAP);
	LCD_printNumber(max_laps);
	
	LCD_goToPosition(9); //13
	LCD_printString("D="); // D -> Distance
	LCD_goToPosition(17);
	
	//reads the PID coefficients from the EEPROM and displays them
	LCD_printString("P=");
	Kp = eeprom_read_byte(EEPROM_ADD_P);
	LCD_printNumber(Kp);
	LCD_printString(" I=");
	Ki = eeprom_read_byte(EEPROM_ADD_I);
	LCD_printNumber(Ki);
	LCD_printString(" D=");
	Kd = eeprom_read_byte(EEPROM_ADD_D);
	LCD_printNumber(Kd);
}


/*
 * Function Name: display_dynamic_data
 * Access: public
 * Params: none
 * Returns: void
 * Description: refreshes the dynamic (non-static) part of the robot
 *				parameter values
 */ 
void display_dynamic_data()
{
	LCD_goToPosition(3);
	LCD_printNumber(lap_count);
	LCD_goToPosition(31);
	if(direction == CW)
	LCD_printString("-");
	else
	LCD_printString("+");
	LCD_goToPosition(11);
	LCD_printNumber(robot_distance);	
}

/*
 * Function Name: print_success
 * Access: public
 * Params: none
 * Returns: void
 * Description: prints "Successful!" after changing some Setting parameters 
 */ 
void print_success()
{
	LCD_clearScreen();
	LCD_setCursor(HIDE);
	LCD_goToPosition(1);
	LCD_printString("  Successful!  ");
	_delay_ms(1500);
}


/*
 * Function Name: input_complete
 * Access: public
 * Params: none
 * Returns: void
 * Description: waits until a user releases a pressed key
 */ 
void input_complete()
{
	char input = LCD_getKeypadInput();
	while(!(input == '\0'))
	{
		input = LCD_getKeypadInput();
	}
}

/*****************************************************************************************/


/************************************ Core Functions *************************************/

/*
 * Function Name: move_robot
 * Access: public
 * Params: none
 * Returns: void
 * Description: This function implements the line following algorithm. It first reads
 *				the current configuration of the robot parameters like from the EEPROM
 *				of the main processor, displays the static data and configuration parameter 
 *				values of the robot in the UI, calculates the offset of the robot heading from 
 *				the path line using an array of 8 IR transceivers, and moves the robot forward 
 *				if the error is zero or turns the robot by an angle proportional to the error using 
 *				control values calculated using the current PID coefficients so as to make the robot 
 *				heading aligned to the line while moving at the desired speed. It then refreshes the 
 *				dynamic data of the robot before recalculating the error and repeating the remaining 
 *				steps again.
 */ 
void move_robot()
{
	read_params();
	display_static_data();
	sei(); // enable global interrupts
	
	int prev_error = 0;
	int sum_error = 0;
	while(1)
	{
		int error = calculate_error();
		if(error == 0)
		{
			move_forward(ref_speed);
		}
		else
		{
			sum_error = sum_error + error;
			int P = Kp * error;
			int I = sum_error * Ki;
			int D = (error - prev_error) * Kd;
			prev_error = error;
			int control = P + I + D;
			
			move_left_motor(control);
			_delay_ms(10);
			move_right_motor(control);
			_delay_ms(10);
		}
		
		if(time_count >= 2)
		{
			int left_motor_dist = get_motor_distance(LEFT);
			int right_motor_dist = get_motor_distance(RIGHT);
			robot_distance = robot_distance + ((left_motor_dist + right_motor_dist) / (2)); //check the formula
			
			display_dynamic_data();
			
			time_count=0;
		}
	}
}

/*
 * Function Name: ISR
 * Access: public
 * Params: none
 * Returns: void
 * Description: interrupt service routine when Timer 1 overflows
 */ 
ISR(TIMER1_OVF_vect)
{	
	time_count++;
	
	if (start_lap_count_flag==0)
	{
		if(lap_count_timer_val < 2)
		{
			lap_count_timer_val++;
		}
		if (lap_count_timer_val == 2) //after 2 * 2sec
		{
			start_lap_count_flag = 1;
			lap_count_timer_val = 0;
		}
	}	
}
/*
 * Function Name: read_params
 * Access: public
 * Params: none
 * Returns: void
 * Description: reads the maximum number of laps, desired speed and PID coefficients from the
 *				EEPROM of the main processor.
 */ 
void read_params()
{
	max_laps = eeprom_read_byte(EEPROM_ADD_MAX_LAP);
	ref_speed = eeprom_read_byte(EEPROM_SPEED_ADD);
	if(ref_speed == SLOW_SPEED)
	{
		slow_flag = 1;
	}
	else
	{
		slow_flag = 0;
	}
	Kp = eeprom_read_byte(EEPROM_ADD_P);
	Ki = eeprom_read_byte(EEPROM_ADD_I);
	Kd = eeprom_read_byte(EEPROM_ADD_D);
	if(Kd == 2)
		Kd = 1.5;
}

/*
 * Function Name: get_motor_distance
 * Access: public
 * Params: this_motor LEFT or RIGHT
 * Returns: int
 * Description: returns the distance traveled by the left or right motor.
 */ 
int get_motor_distance(int this_motor)
{
	
	char firstByte;
	char secondByte;
	char thirdByte;
	char fourthByte;
	int32_t current_encoder_val = 0;
	int distance = 0; // distance in cm
	
	i2c_start(MOTOR_DRIVE+I2C_WRITE);
	if(this_motor == LEFT)
	{
		i2c_write(EN1_FIRST_BYTE);	//write to operation mode register
	}
	else
	{
		i2c_write(EN2_FIRST_BYTE);	//write to operation mode register
	}
	
	i2c_rep_start(MOTOR_DRIVE+I2C_READ);	//mode 0
	firstByte = i2c_readNak();
	i2c_stop();			//stop and release bus
		
	_delay_ms(6);
	i2c_start(MOTOR_DRIVE+I2C_WRITE);
	if(this_motor == LEFT)
	{
		i2c_write(EN1_SECOND_BYTE);	//write to operation mode register
	}
	else
	{
		i2c_write(EN2_SECOND_BYTE);	//write to operation mode register
	}
	i2c_rep_start(MOTOR_DRIVE+I2C_READ);	//mode 0
	secondByte = i2c_readNak();
	i2c_stop();			//stop and release bus
	
	_delay_ms(6);
	i2c_start(MOTOR_DRIVE+I2C_WRITE);
	if(this_motor == LEFT)
	{
		i2c_write(EN1_THIRD_BYTE);	//write to operation mode register
	}
	else
	{
		i2c_write(EN2_THIRD_BYTE);	//write to operation mode register
	}
	i2c_rep_start(MOTOR_DRIVE+I2C_READ);	//mode 0
	thirdByte = i2c_readNak();
	i2c_stop();			//stop and release bus	
	
	_delay_ms(6);
	i2c_start(MOTOR_DRIVE+I2C_WRITE);
	if(this_motor == LEFT)
	{
		i2c_write(EN1_FOURTH_BYTE);	//write to operation mode register
	}
	else
	{
		i2c_write(EN2_FOURTH_BYTE);	//write to operation mode register
	}
	i2c_rep_start(MOTOR_DRIVE+I2C_READ);	//mode 0
	fourthByte = i2c_readNak();
	i2c_stop();			//stop and release bus
	
	current_encoder_val = (firstByte<<24) | (secondByte<<16) | (thirdByte<<8) | (fourthByte<<0);	
	if(this_motor == LEFT)
	{		
		distance = ((current_encoder_val - prev_en1_val)* 31.46) / (360); //t=2sec, 1rev = 360 degrees, C = 31cm	
		prev_en1_val = current_encoder_val;
	}
	else
	{	
		distance = ((current_encoder_val - prev_en2_val)* 31.46) / (360);  //t=2sec, 1rev = 360 degrees, C = 31cm
		prev_en2_val = current_encoder_val;		
	}
	return distance;
}


/*
 * Function Name: move_left_motor
 * Access: public
 * Params: control PID speed control input
 * Returns: void
 * Description: slows down or speeds up the left motor according to the 
 *				contorl input
 */ 
void move_left_motor(int control)
{
	
	if(control > ref_speed)
	{
		control = ref_speed; //to ensure underflow
	}
	write_this_to((ref_speed - control), LEFT_SPEED);
}


/*
 * Function Name: move_right_motor
 * Access: public
 * Params: control PID speed control input
 * Returns: void
 * Description: slows down or speeds up the right motor according to the 
 *				contorl input
 */ 
void move_right_motor(int control)
{
	if(control < -ref_speed)
	{
		control = -ref_speed; //to ensure underflow
	}
	write_this_to((ref_speed + control), RIGHT_SPEED);
}


/*
 * Function Name: move_forward
 * Access: public
 * Params: speed desired speed
 * Returns: void
 * Description: moves the robot with the desired speed
 */ 
void move_forward(int speed)
{
	write_this_to(speed, LEFT_SPEED);
	write_this_to(speed, RIGHT_SPEED);
}


/*
 * Function Name: calculate_error
 * Access: public
 * Params: none
 * Returns: int
 * Description: returns negative error if only the right most IR sensors detect the
 *				black line, positive if only the left most IR sensors detect the 
 *				line and zero otherwise. The amount returned depends on which sensors 
 *				make the detections with the ones on the edges returning the highest value.
 */ 
int calculate_error()
{
	if ((!bit_is_clear(PIND,PD4)) && (!bit_is_clear(PIND,PD3))) //what about &
	{
		last_error = 0;
		return 0;
	}	
	
	
	if ((!bit_is_clear(PIND,PD3)) && (!bit_is_clear(PIND,PD2)))
	{
		last_error = -1;
		return -1;
	}	
	if ((!bit_is_clear(PIND,PD4)) && (!bit_is_clear(PIND,PD5)))
	{
		last_error = 1;
		return 1;
	}
	
		
	if ((!bit_is_clear(PIND,PD2)) && (!bit_is_clear(PIND,PD1)))
	{
		last_error = -1;
		return -1;
	}	
	if ((!bit_is_clear(PIND,PD5)) && (!bit_is_clear(PIND,PD6)))
	{
		last_error = 1;
		return 1;
	}
	
	
	if ((!bit_is_clear(PIND,PD1)) && (!bit_is_clear(PIND,PD0)))
	{
		last_error = -3;
		return -3;
	}		
	if ((!bit_is_clear(PIND,PD6)) && (!bit_is_clear(PIND,PD7)))
	{
		last_error = 3;
		return 3;
	}
	
	
	if (!bit_is_clear(PIND,PD0))
	{
		last_error = -4;
		return -4;
	}	
	if (!bit_is_clear(PIND,PD7))
	{
		last_error = 4;
		return 4;
	}
	
	if ((bit_is_clear(PIND,PD0)) && (bit_is_clear(PIND,PD1)) && (bit_is_clear(PIND,PD2)) && (bit_is_clear(PIND,PD3)) && (bit_is_clear(PIND,PD4)) && (bit_is_clear(PIND,PD5)) && (bit_is_clear(PIND,PD6)) && (bit_is_clear(PIND,PD7)))
	{		
		if(slow_flag)
		{
			if (start_lap_count_flag == 1)
			{
				++lap_count;
				start_lap_count_flag = 0;
			}		
			
			if(lap_count == max_laps)
			{
				LCD_goToPosition(3);
				LCD_printNumber(lap_count); //display max lap count while turning
				lap_count = 0;
				change_direction();	
				return 0;
			}
		}
		return last_error;
	}
	
	
	else
	{
		return last_error;
	}
}



/*
 * Function Name: change_direction
 * Access: public
 * Params: none
 * Returns: void
 * Description: changes the direction of the robot motion when lap count 
 *				reaches the max level
 */ 
void change_direction()
{
	//First stop for half a second
	move_forward(128);
	_delay_ms(500);
	
	//Change the direction and turn left or right
	if(direction == CCW)
	{
		direction = CW;
		//turn left
		move_left_motor(ref_speed - 111);
		move_right_motor(143 - ref_speed);
	}
	else
	{
		direction = CCW;
		//turn right
		move_left_motor(ref_speed - 143);
		move_right_motor(111 - ref_speed);
	}
	
	_delay_ms(1500); //keep turning for 1.5 seconds
	
	//wait until any of the four central IR sensors are detected
	while ((bit_is_clear(PIND,PD2)) && (bit_is_clear(PIND,PD3)) && (bit_is_clear(PIND,PD4)) && (bit_is_clear(PIND,PD5)))
	{
		;
	}
}


/*
 * Function Name: write_this_to
 * Access: public
 * Params: data & address the data to be written and the address to write to
 * Returns: void
 * Description: writes the data to the specified address
 */ 
void write_this_to(unsigned char data, unsigned char address)
{
	i2c_start(MOTOR_DRIVE+I2C_WRITE);
	i2c_write(address);	//write to command register
	i2c_write(data);
	i2c_stop();			//stop and release bus
	_delay_ms(10);
}


