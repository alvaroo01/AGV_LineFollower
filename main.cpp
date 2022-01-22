#include <avr/io.h>
#include <stdio.h>

#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/interrupt.h>

#define motor1_velocity PD3
#define motor1_pinA PD5
#define motor1_pinB PD4
#define motor2_velocity PB3
#define motor2_pinA PD6
#define motor2_pinB PD1
#define infra_1 PC0
#define infra_2 PC1
#define infra_3 PC2
#define infra_4 PC3
#define infra_5 PC4
#define led PB2
#define delay_ 5000
#define led_1hZ PB6 
#define test_led PC0
#define Sonar_Led1 PD7
#define Sonar_Trig PB7


typedef struct
{
	unsigned char flag_ir1;
	unsigned char flag_ir2;
	unsigned char flag_ir3;
	unsigned char flag_ir4;
	unsigned char flag_ir5;
	unsigned char start_flag;
	unsigned char flag_sonar;

	
} Flags_st;

volatile Flags_st flags={0,0,0,0,0,0};
	
typedef struct
{
	unsigned char infra1;
	unsigned char infra2;
	unsigned char infra3;
	unsigned char infra4;
	unsigned char infra5;
	
} Infra_st;

volatile Infra_st infra={0,0,0,0,0};

int cnt = 100;
int distancia;
int pulse;
int i;

void init(void){
	DDRC = 0b00000001;                    // Sets PC0, PC1, PC2, PC3, PC4 as inputs for infrareds.
	DDRD = 0b01111010;		// All the outputs from the motors and led1 and led2
	DDRB = 0b00001101;		// Output from motor2_velocity and output from 1Hz blinking led
	
	PORTD = 0b00000000;		// Put to 0 all the pins on PORTD
	PORTB = 0b00000101;
	PORTC = 0b00000000;                   // Sets the led 1 to be on by default.
	
	//Making the 1Hz led
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000100;
	TIMSK0 = 0b00000010;
	OCR0A  = 19;
	
	DDRB |= (1<<Sonar_Trig);
	DDRD |=(1<<Sonar_Led1);//|Sonar_Led2;
	TCCR1A = 0x00;
	EICRA|=(1<<ISC00);
	EIMSK|=(1<<INT0);
	PORTD &= ~(1<<Sonar_Led1);
	
	TCCR1B = 0b00001011;				  // Sets CTC mode and prescaler to 64.	-> (1<<WGM12) | (1<<CS11) | (1<<CS10)
	//TIMSK1 = 0b00000010;				  // Chooses compare match A which will enable the interruption every time the ocr is reached. -> (1<<OCIE1A)
	OCR1A = 391;						  // The value of ocr required to have a 1s period (1Hz).
	
	TCCR2A = 0b10100011;    // bit7 a 1 -> Clear OC2B and OC2A on compare match | bit0 e bit1 a 1-> Define operation mode as Fast PWM
	TCCR2B |= (1<<CS20) | (1<<CS22);	// 128 prescaler
	OCR2B=0;				// left motor
	OCR2A=0;				// right motor
	
	sei();								  // Activates all interruptions.
}


void sonar(void)
{
	PORTB|=(1<<Sonar_Trig);        //
	_delay_us(10);
	PORTB &=~(1<<Sonar_Trig);    // entra no pulso baixo

	distancia = (pulse/58);

	if(distancia<=25)
	{
		flags.flag_sonar = 1;
		PORTD |= (1<<Sonar_Led1);
		}else{
		PORTD &= ~(1<<Sonar_Led1);
		flags.flag_sonar = 0;

	}
}




void forward (int vel){
	//M1
	OCR2B = vel;
	OCR2A = vel;
	PORTD |= (1<<motor1_pinA);
	PORTD &= ~(1<<motor1_pinB);
	//M2
	PORTD |= (1<<motor2_pinA);
	PORTD &= ~(1<<motor2_pinB);
	
}

void backwards (int vel){
	//M1
	OCR2B = vel;
	OCR2A = vel;
	PORTD &= ~(1<<motor1_pinA);
	PORTD |= (1<<motor1_pinB);
	//M2
	PORTD &= ~(1<<motor2_pinA);
	PORTD |= (1<<motor2_pinB);
	
}

void stop (){
	//M1
	OCR2B = 0;
	OCR2A = 0;
	PORTD &= ~(1<<motor1_pinA);
	PORTD &= ~(1<<motor1_pinB);
	//M2
	PORTD &= ~(1<<motor2_pinA);
	PORTD &= ~(1<<motor2_pinB);
}

void left (int vel1, int vel2){
	//left motor is A
	//right motor is B
	OCR2B = vel1;
	OCR2A = vel2;
	PORTD |= (1<<motor1_pinA);
	PORTD &= ~(1<<motor1_pinB);
}

void right (int vel1, int vel2){
	//left motor is A
	//right motor is B
	OCR2B = vel2;
	OCR2A = vel1;
	PORTD |= (1<<motor1_pinA);
	PORTD &= ~(1<<motor1_pinB);
}
	
void start (int start_vel){
	if ((PINC | 0b11000001) == 0b11000001)
		{
		forward(start_vel);
		flags.start_flag = 1;
		}
		}

void ir_sensors_test2(	int start_vel, int low_speed){
	sonar();
	if ((PINC | 0b11111101) == 0b11111101){
		while(!((PINC | 0b11110111) == 0b11110111)){
			right(start_vel,0);
		}		
		
		} else if ((PINC | 0b11011111) == 0b11011111){
		while(!((PINC | 0b11110111) == 0b11110111)){
			left(start_vel,0);
		}
		
		} else if ((PINC | 0b11101111) == 0b11101111){
			left(start_vel,0);
		
		} else if ((PINC | 0b11111011) == 0b11111011){
			right(start_vel,0);
		} else if (flags.flag_sonar==1){
		while(!(flags.flag_sonar==0)) {
			stop();
			sonar();
		}
		forward(80);
		
		} else forward(start_vel);
	
}


int main(void)
{
	init();
	int velocity =	35;
	while (1)
	{
		flags.start_flag = 0;
		sonar();
		if(flags.flag_sonar == 0) {
		start(velocity);	
		} else stop();		
		while(flags.start_flag == 1){
			ir_sensors_test2(velocity, 25);
			}
		}
	}





ISR(TIMER0_COMPA_vect)                    // Interruption of TC1.
{
	
	
	cnt--;
	if (cnt==0){
		PORTB ^= (1<<led_1hZ);					  // Swap the state of the bit (0 to 1 or 1 to 0).
		cnt = 100;
	}
}

ISR(INT0_vect)
{
	if(i == 0)
	{

		TCCR1B |= 1<<CS10;
		TCNT1 = 0;
		i = 1;
	}
	else
	{

		pulse = TCNT1;
		TCCR1B = 0;
		i = 0;
	}
}

