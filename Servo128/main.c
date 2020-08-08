#define F_CPU 8000000UL			
#include <avr/io.h>			
#include <avr/interrupt.h>
#include <stdio.h>			
#include <util/delay.h>		
#include "myLCD.h"

// Define frequency
#define PWM_MAX_DUTY_CYCLE 0x3FF
#define  Sampling_Time 25
#define  inv_Sampling_Time 40

volatile uint8_t Direction = 0, RPM = 0;
volatile long Pulse=0;
volatile float pPart = 0, iPart = 0, dPart = 0, Output = 0;
int  pre_Err = 0, Err;
unsigned long Acc;

void ADC_vInit(void)
{
	DDRF  = 0x00; /* Make ADC port as input */
	PORTF = 0xFF;
    /* 
       Select AVCC as reference with external capacitor at AREF pin
       ADC1 as the single-ended input channel with 1x gain
    */
    ADMUX =   (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX4)|
			  (0<<MUX3) |(0<<MUX2) |(0<<MUX1) |(0<<MUX0);

    /*
       Enable ADC; Select Prescaler of 32 (clock frequency of 57.6 kHz)
    */
    ADCSRA =  (1<<ADEN) |(0<<ADSC) |(0<<ADFR) |(0<<ADIF)|
			  (0<<ADIE) |(1<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);
}

uint16_t ADC_Read(unsigned int adc_channel)
{
	ADMUX |= adc_channel;
	ADCSRA|= (1<<ADSC);
	while(bit_is_clear(ADCSRA,ADIF))
	{
		;
	}
	return ADCW;
}

void PWM_vInit(void)
{
	DDRE |= (1<<PE4);		/* Make OCR3B pin as Output */
    /* 
       Start Timer 3 with clock prescaler CLK/8 and phase correct 
       10-bit PWM mode. Output on PE4 (OC3B). Resolution is 1.09 us.
       Frequency is 450 Hz.
    */
    TCCR3A =  (0<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|
			  (0<<COM3C1)|(0<<COM3C0)|(1<<WGM31) |(1<<WGM30); 
			  
	TCCR3B =  (0<<ICNC3) |(0<<ICES3) |(0<<WGM33) |(0<<WGM32)|
			  (0<<CS32)  |(1<<CS31)  |(0<<CS30);
	
	TCCR3C =  (0<<FOC3A) |(0<<FOC3B) |(0<<FOC3C);
	
    // Set duty cycle to 0%
    OCR3B  = 0;
}

void PWM_vSetDutyCycle(uint16_t u16DutyCycle)
{
    // Clip parameter to maximum value
    if (u16DutyCycle > PWM_MAX_DUTY_CYCLE)
    {
        u16DutyCycle = PWM_MAX_DUTY_CYCLE;
    }

    OCR3B = u16DutyCycle;
}

void TMR0_vInit(void)
{
	/* Start timer 0 with clock prescaler CLK/1024 */
	/* Resolution is 139 us */
	/* Maximum time is 9.1 s */
	TCCR0 =  (0<<FOC0)  |(0<<WGM00)|(0<<COM01) |(0<<COM00)|
			 (0<<WGM01) |(1<<CS02) |(1<<CS01)  |(1<<CS00);
	TCNT0 = 60;
	TIMSK =  (0<<OCIE2) |(0<<TOIE2)|(0<<TICIE1)|(0<<OCIE1A)|
			 (0<<OCIE1B)|(0<<TOIE1)|(0<<OCIE0) |(1<<TOIE0);
}


void ISR_vInit(void)
{
	DDRD &= ~((1<<PD0)|(1<<PD1));		/* Make INT0 pin as Input */
	PORTD = (1<<PD0)|(1<<PD1);
	EIMSK = (0<<INT7)|(0<<INT6)|(0<<INT5)|(0<<INT4)|
			(0<<INT3)|(0<<INT2)|(1<<INT1)|(1<<INT0);		/* Enable INT0/INT1*/
	EICRA = (0<<ISC31)|(0<<ISC30)|(0<<ISC21)|(0<<ISC20)|
			(1<<ISC10)|(1<<ISC11)|(1<<ISC01)|(0<<ISC00);/* Trigger INT0 on Rising Edge triggered */
}

void Port_vInit(void)
{
	DDRC  = 0xFF;			/* Make PORTC as output Port */
	DDRE  &=~ (1<<PE6);
	PORTE |= (1<<PE6);
}

void put_int(long int z)
{
	putChar_LCD((z/100)+48);
	putChar_LCD((z%100/10)+48);
	putChar_LCD((z%10)+48);
}

void LCD_display(unsigned int PWM)
{
	// Print Duty Cycle
	uint16_t b = ((uint32_t)PWM*100)/PWM_MAX_DUTY_CYCLE;
	//PWM_vSetDutyCycle(PWM);
	move_LCD(1,1);
	print_LCD("Duty Cycle:");
	move_LCD(1,13);
	put_int(b);
	move_LCD(1,16);
	print_LCD("%");
}


void Speed_Print()
{
	char dis[20];
	move_LCD(2,1);
	sprintf(dis,"RPM: %d   ",RPM);
	print_LCD(dis);
}

void Speed_PID(uint8_t Kp, uint8_t Ki, float Kd){
	Err = (Acc*360)/1024 - RPM;
	
	pPart = Kp* Err;
	iPart += Ki*Sampling_Time*Err/1000;
	dPart = Kd*(Err-pre_Err)*inv_Sampling_Time;
	Output += pPart + iPart + dPart;
	if(Output >1023) Output =1023;
	if(Output <1) Output =1;
	PWM_vSetDutyCycle(Output);
}
int main(void)
{
	Port_vInit();
	ADC_vInit();			
	PWM_vInit();
	LCD_vInit();
	clr_LCD();
	ISR_vInit();
	TMR0_vInit();
	sei();				/* Enable Global Interrupt */
	while(1)
	{
		Acc = ADC_Read(0);
		//setpoint = m*250/1024;
		if (Direction !=0)	/* Rotate DC motor Clockwise */
			PORTC = 1;
		else			/* Else rotate DC motor Anticlockwise */
			PORTC = 2;
// 		PWM_vSetDutyCycle(Acc);
		LCD_display(Acc);
		Speed_Print();
	}
}

ISR(INT0_vect)
{
	Direction = ~Direction;		/* Toggle Direction */
	_delay_ms(50);			/* Software de-bouncing control delay */
}

ISR(INT1_vect)
{
	Pulse++;
}

ISR(TIMER0_OVF_vect)
{   
	TCNT0=60; 
	RPM = (Pulse*40*60)/360;
	Pulse = 0;
	Speed_PID(3, 1, 1);
}