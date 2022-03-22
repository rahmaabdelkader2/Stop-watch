/*
 * mini_project2_r.c
 *
 *  Created on.;7: Sep 21, 2021
 *      Author: Rahma Abdelkader
 */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//global variables

unsigned char second = 0;
unsigned char minute = 0;
unsigned char hour = 0;

void TIMER1_Init(void )
{

	TCCR1A = (1<<FOC1A);
	TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS11);
	TCNT1 = 0;
	OCR1A = 15625;
	TIMSK|=(1<<OCIE1A); // compare match
}
     //counting
ISR(TIMER1_COMPA_vect)
{
	second ++;
	if (second == 60){
		second = 0;
		minute ++;
	}
	if (minute == 60){
		second = 0;
		minute = 0;
		hour ++;
	}
	if (hour == 24){
		second = 0;
		minute = 0;
		hour = 0 ;
	}

}
void INT0_Init_Reset(void)
{
	DDRD &=~(1<<PD2);
	PORTD|=(1<<PD2);
	MCUCR|=(1<<ISC01);
	GICR |=(1<<INT0);

}
ISR(INT0_vect)
{
	second = 0;
	minute = 0;
	hour = 0 ;
}
void INT1_Init_Pause(void)
{
	DDRD &= ~(1<<PD3);

	MCUCR|=(1<<ISC10)|(1<<ISC11);
	GICR |=(1<<INT1);

}
ISR(INT1_vect)
{
     //disable interrupts by setting three pins to zero
	TCCR1B &= ~(1<<CS10)&~(1<<CS11)&~(1<<CS12);
}
void INT2_Init_Resume(void)
{
	DDRB &=~(1<<PB2);
	PORTB |= (1<<PB2);
	MCUCR&=~(1<<ISC2);
	GICR |=(1<<INT2);

}
ISR(INT2_vect)
{
	TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS11);
}
int main()
{
	DDRA = 0xff;	//configure the 7seg data pins to input
	DDRC = 0x0f;	//configure the 7seg data pins to output
	PORTC = 0x00;   // at beginning all LEDS off
    SREG|=(1<<7);   //enable global interrupt enable bit
    TIMER1_Init();

     //three interrupt function

	INT0_Init_Reset();
	INT1_Init_Pause();
	INT2_Init_Resume();
	while(1)
	{
		//7 segment implementation

		PORTA = (1<<5);
		PORTC = (PORTC &0XF0) | (second % 10);
		_delay_ms(5);
		PORTA = (1<<4);
		PORTC = (PORTC &0XF0) | (second / 10);
		_delay_ms(5);
		PORTA = (1<<3);
		PORTC = (PORTC &0XF0) | (second % 10);
		_delay_ms(5);
		PORTA = (1<<2);
		PORTC = (PORTC &0XF0) | (second / 10);
		_delay_ms(5);
		PORTA = (1<<1);
		PORTC = (PORTC &0XF0) | (second % 10);
		_delay_ms(5);
		PORTA = (1<<0);
		PORTC = (PORTC &0XF0) | (second / 10);
		_delay_ms(5);
	}

}
