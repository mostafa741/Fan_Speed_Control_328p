/*
* Tachometer_and_motor_controller_FreeRTOS_328p.c
*
* Created: 26-Apr-17 7:12:27 AM
* Author : Mostafa
*/
/*vTaskDelay in ticks not in ms*/

#define F_CPU 16000000UL
#define TIMER2_FREQUENCY (F_CPU/_BV(5))
#define MAX_RPM 3000.0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdfix.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "uart.h"
#include "hd44780.h"

static void KEYBOARD_TASK(void* pvParameters);
static void PID_control_TASK(void* pvParameters);
static void LCD_TASK(void* pvParameters);
static void vApplicationIdleHook( void );

static volatile uint32_t TIMER2_OVF_offset=0;
static volatile uint32_t period=0;

static volatile float measured_rpm=0;
static volatile uint16_t target_rpm=1000;

static volatile float Kp=10,Ki=2,Kd=5.2;
//volatile float Kp=1,Ki=1,Kd=0.1;

static volatile float error;
static volatile float output;
static volatile float sum_error=0,last_error=0;

static const float read_sample_delay=10;



int main(void)
{
	//UART_BAUD_SELECT(9600,F_CPU);
	uart_init(UART_BAUD_SELECT(9600,F_CPU));
	xTaskCreate(KEYBOARD_TASK,( const char * const )"KEYBOARD TASK",150,NULL,1,NULL);
	xTaskCreate(PID_control_TASK,( const char * const )"PID control TASK",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(LCD_TASK,( const char * const )"LCD TASK",150,NULL,1,NULL);

	vTaskStartScheduler();

	while (1)
	{

	}
}

void KEYBOARD_TASK(void* pvParameters)
{

	while(1)
	{
		char recived_buffer[10]={0},temp;
		uint8_t i=0;
		while (uart_available())
		{
			temp= uart_getc();

			if (temp=='\r'||temp=='\n')
			{
				break;
			}

			else if (temp>=0x2E||temp<=0x7A)
			{
				recived_buffer[i]=temp;
				i++;
			}
			vTaskDelay(5);
		}
		vTaskDelay(5);

		temp= recived_buffer[0];
		memmove(&recived_buffer[0], &recived_buffer[0 + 1], strlen(recived_buffer) - 0);
		switch (temp)
		{
			case 's':
			target_rpm=atof(recived_buffer);
			break;
			case 'p':
			Kp=atof(recived_buffer);
			break;
			case 'i':
			Ki=atof(recived_buffer);
			break;
			case 'd':
			Kd=atof(recived_buffer);
			break;
			default:
			break;
		}
		uart_puts("Measured RPM=");
		dtostrf(measured_rpm,0,0,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("\n\r");

		uart_puts("Target RPM=");
		dtostrf(target_rpm,0,0,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("\n\r");

		uart_puts("kp=");
		dtostrf(Kp,0,1,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("\n\r");

		uart_puts("ki=");
		dtostrf(Ki,0,1,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("\n\r");

		uart_puts("kd=");
		dtostrf(Kd,0,1,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("\n\r");

		uart_puts("Duty cycle=");
		dtostrf((OCR0A/256.0)*100,0,1,recived_buffer);
		uart_puts(recived_buffer);
		uart_puts("%");
		uart_puts("\n\r");

		vTaskDelay(1000);
	}
}

void PID_control_TASK(void* pvParameters)
{
	/*SETUP TIMER 2*/
	TIMSK2=_BV(TOIE2);
	sei();
	TCCR2B=_BV(CS20)|_BV(CS21)/*|_BV(CS22)*/;

	/*SETUP EXTERNAL INTERRUPT*/
	//PORTD|=_BV(PIND2);
	EICRA=_BV(ISC01)/*|_BV(ISC00)*/;
	EIMSK=_BV(INT0);


	/*SETUP PWM TIMER 0*/
	DDRD|=_BV(PIND6)|_BV(PIND5);
	TCCR0A=_BV(COM0A1)|_BV(COM0B1)|_BV(WGM00)|_BV(WGM01);
	TCCR0B=_BV(CS02)|/*_BV(CS01)|*/_BV(CS00);
	OCR0B=90;

	float error_i=0;
	while(1)
	{
		measured_rpm=((TIMER2_FREQUENCY/(float)period)*60)/2;
		if (measured_rpm>MAX_RPM*(1+0.1)||measured_rpm<0)
		{
			OCR0A=0xff;
			measured_rpm=0;
			continue;
		}
		error=(target_rpm-measured_rpm);
		error_i+=error_i*read_sample_delay;
		output=Kp*error+Ki*error_i+Kd*((error-last_error)/read_sample_delay);
		if (output>MAX_RPM)
		{
			output=MAX_RPM;
		}
		else if(output<0)
		{
			output=0;
		}
		OCR0A=(output/MAX_RPM)*0xff;
		vTaskDelay(read_sample_delay);
	}
}

void LCD_TASK(void* pvParameters)
{
	lcd_init();
	while(1)
	{
		lcd_clrscr();
		char buffer[15];
		dtostrf(measured_rpm,0,0,buffer);
		lcd_puts("RPM=");
		lcd_puts(buffer);
		sprintf(buffer,"T_RPM=%d",target_rpm);
		//dtostrf(Kp,0,0,buffer);
		lcd_goto(0x40);
		lcd_puts(buffer);
		vTaskDelay(300);
	}
}

ISR(TIMER2_OVF_vect)
{
	TIMER2_OVF_offset+=_BV(8);
}

ISR(INT0_vect)
{
	period=TCNT2+TIMER2_OVF_offset;
	TCNT2=0;
	TIMER2_OVF_offset=0;
}

