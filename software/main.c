/*
* This file is§ part of LP-blink
*
* Copyright (C) 2013 Mirco Gysin <miagox@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include <msp430.h>
#include <msp430g2553.h>
#include <clock.h>
#include <timerA.h>
#include <launchpad.h>

#define interrupt(x) void __attribute__((interrupt (x)))

#define LED_1           BIT0 
#define LED_2           BIT1 
#define LED_3           BIT2 
#define LED_4           BIT3 
#define LED_5           BIT4 
#define LED_6           BIT5 

#define BLINKING_MODE_1_WAIT 20
#define BLINKING_MODE_1_CYCLES 10
#define BLINKING_MODE_2_WAIT 9
#define BLINKING_MODE_2_CYCLES 300

#define LED_OUT         P1OUT
#define LED_DIR         P1DIR

#define BUTTON 		BIT7
#define CYCLES_PER_MINUTE 975
#define TIMEOUT_MODE1 46

void wait_ms(int time_ms);
void set_leds(int blinking_mode, int value);

void set_timer(int value);
void set_led_on(int led);
void set_led_off(int led);

void init_leds();

int timerCount;

volatile int timeout_counter = 0;

int mode = 0;
int current_mode1_time = 0;

int main( void ){

	disableWDT();// Stop watchdog timer
	setDCOCLK( DCO_1M );
	setSMCLK( SMCLK_DCO, CLK_DIV_1 );

	P1DIR &= ~BUTTON; //Button input
 	P1IE |= BUTTON; //interrupt for button enabled
 	P1IFG &= ~BUTTON; //Interrupt flag enabled
	P1IES |= BUTTON; //Hi/lo edge


	LED_DIR |= (LED_1+LED_2+LED_3+LED_4+LED_5+LED_6);


	init_leds();

	enableTimerA0CCInterrupt();
	setTimerA0Mode( TAMODE_CONT );
	setTimerA0ClockSource( TA_SMCLK );
	setTimerA0Divider( TA_DIV_1 );

	__enable_interrupt();
	
	__bis_SR_register( 0x18 );

	//__bis_SR_register( 0x18 ); // LPM0 with interrupts enabled

	while( 1 ){};

	return 0;
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void timerA0ISR( void )
{
	if(timeout_counter != 0){
		timeout_counter = timeout_counter - 1;
	}
 //	LED_OUT = LED_OUT ^ LED_1;
 	if(mode == 1)
	{
		if(timeout_counter == 0)
		{
			mode = 2; // running mode
			set_leds(1, current_mode1_time);
			timeout_counter = current_mode1_time * CYCLES_PER_MINUTE;
		}
	}
	if(mode == 2) 
	{
		if(timeout_counter == 0)
		{
			mode = 0; // IDLE mode
			current_mode1_time = 0;
        		set_leds(2, 0);
		}
	}
}

 #pragma vector=PORT1_VECTOR
__interrupt void Port_1( void )
{
	//P1IES ^= BUTTON;
	P1IFG &= ~BUTTON; // P1.3 IFG cleared
	//LED_OUT = LED_OUT ^ BIT5;
	//set_led_on(LED_2);
	//wait_ms(40);
	//set_led_off(LED_2);
	if(mode == 0){
		mode = 1;
		timeout_counter = 46;
	}
	if(mode == 1)
	{
		current_mode1_time = current_mode1_time + 5;
		set_timer(TIMEOUT_MODE1);	
		if(current_mode1_time > 30)
		{
			current_mode1_time = 5;
		}
        	set_leds(1, current_mode1_time);
	}
	if(mode == 2)
	{
		if(timeout_counter < (5 * CYCLES_PER_MINUTE))
		{
			set_leds(1, 5);
		} else if(timeout_counter < (10 * CYCLES_PER_MINUTE))
		{
			set_leds(1, 10);
		} else if(timeout_counter < (15 * CYCLES_PER_MINUTE))
		{
			set_leds(1, 15);
		} else if(timeout_counter < (20 * CYCLES_PER_MINUTE))
		{
			set_leds(1, 20);
		} else if(timeout_counter < (25 * CYCLES_PER_MINUTE))
		{
			set_leds(1, 25);
		} else 
		{
			set_leds(1, 30);
		}
	}
}

void set_leds(int blinking_mode, int value)
{
	int cycle_counter = 0;
	if(blinking_mode == 1)
	{
		if(value == 5)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_1);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_1);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
		if(value == 10)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_2);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_2);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
		if(value == 15)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_3);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_3);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
		if(value == 20)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_4);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_4);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
		if(value == 25)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_5);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_5);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
		if(value == 30)
		{
			for(cycle_counter = 1; cycle_counter <= BLINKING_MODE_1_CYCLES; cycle_counter++)
			{
				set_led_on(LED_6);
				wait_ms(BLINKING_MODE_1_WAIT);
				set_led_off(LED_6);
				if(cycle_counter != BLINKING_MODE_1_CYCLES)
				{
					wait_ms(BLINKING_MODE_1_WAIT);
				}
			}
		}
	}
	if(blinking_mode == 2)
	{
		// timer timeout, x minutes have passed

		for(cycle_counter = 0; cycle_counter <= BLINKING_MODE_2_CYCLES; cycle_counter++)
		{
			set_led_on(LED_1);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_1);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_on(LED_2);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_2);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_on(LED_3);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_3);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_on(LED_4);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_4);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_on(LED_5);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_5);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_on(LED_6);
			wait_ms(BLINKING_MODE_2_WAIT);
			set_led_off(LED_6);
			wait_ms(BLINKING_MODE_2_WAIT);
		}
		
	}
}

void wait_ms(int time_ms)
{
	// @1M, div_1: 1000 = 1 ms
	int counter = 0;
	for(counter = 0; counter < time_ms; counter++){
		__delay_cycles(1000);
	}
}

void set_led_on(int led)
{
	LED_OUT &= ~led;
}

void set_led_off(int led)
{
	LED_OUT |= led;
}

void init_leds()
{
	LED_OUT |= (LED_1 + LED_2 + LED_3 + LED_4 + LED_5 + LED_6);
}

void set_timer(int value)
{
	timeout_counter = value;
}
