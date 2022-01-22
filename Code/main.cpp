/*
AGV_Linefollower 
Created by:
	Álvaro Lopes-Cardoso
	Fabien Lefebvre
Date: 
	22/01/2022
*/

#include <avr/io.h>									// All the Includes necessary.
#include <stdio.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/interrupt.h>

#define motor1_velocity PD3							// Several Defines.
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


typedef struct										// Flag struct.
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

int cnt = 100;											// Counter for TC0 Interrupt.
int cm;													// Stores the distance measured by the sonar. 
int wave_time;											// Stores the time that the sonar wave took to go from the sonar to the object and back to the sonar
int sonf;												// Sonar flag for interruption. 

void init(void){
														// 0 is input and 1 is output.
														// 1 turns on Led and 0 turns off.
	DDRC = 0b00000000;									// Sets PC1, PC2, PC3, PC4, PC5 as inputs for infrareds. 
	DDRD = 0b11111010;									// Motors outputs (IN's and EN1) and Echo pin for Sonar as input and PD7 as output for Sonar Led.
	DDRB = 0b10001000;									// Motor output (EN2) output for 1Hz blinking led.
	
	PORTD = 0b00000000;									// Clear all the pins on PORTD.
	PORTB = 0b00000101;									
	PORTC = 0b00000000;									// Clears Sonar led to be off by default.
	
	//Making the 1Hz led
	
	TCCR0A = 0b00000010;								// (1<<WGM01) for CTC mode.						
	TCCR0B = 0b00000100;								// (1<<CS02) Prescaler of 256 .
	TIMSK0 = 0b00000010;								// (1<<OCIE0A) Enables Compare match A .
	OCR0A  = 19;										
	
	TCCR1A = 0b00000000;								
	EICRA = 0b00000001;									// (1<<ISC00).
	EIMSK = 0b00000001;								    // (1<<INT0).
	
	TCCR2A = 0b10100011;							    // Clear OC2B and OC2A on compare match and define operation mode as Fast PWM.
	TCCR2B = 0b00000101;								// (1<<CS22) (1<<CS20) Prescaler of 128.
	OCR2B=0;											// Left motor pwm.
	OCR2A=0;											// Right motor pwm.
	
	sei();												// Activates all interruptions.
}


void sonar(void)
{
	PORTB|=(1<<Sonar_Trig);								// Trig pin in set on HIGH.
	_delay_us(10);										// Waits for 10us. 
	PORTB &=~(1<<Sonar_Trig);							// Trig pin in set on LOW.
	cm = (wave_time/58);								// The distance is calculated.		

	if(cm<=25)											// If the distance is less than 25cm then...
	{
		flags.flag_sonar = 1;							// Sonar flag goes to 1.
		PORTD |= (1<<Sonar_Led1);						// Sonar led turns on
		}else{
		PORTD &= ~(1<<Sonar_Led1);						// Sonar led turns off
		flags.flag_sonar = 0;							// Sonar flag goes to 0.
	}
}




void forward (int vel){									// Car goes forward.
	
	OCR2B = vel;										// Sets the speed of both motors
	OCR2A = vel;
	//M1
	PORTD |= (1<<motor1_pinA);							// Current flows in a certain direction allowing both motors to be in the forward direction
	PORTD &= ~(1<<motor1_pinB);
	//M2
	PORTD |= (1<<motor2_pinA);	
	PORTD &= ~(1<<motor2_pinB);
	
}

void backwards (int vel){								// Car goes backwards.
	//M1
	OCR2B = vel;										// Sets the speed of both motors.
	OCR2A = vel;
	PORTD &= ~(1<<motor1_pinA);							// Current flows in a certain direction allowing both motors to be in the backwards direction.
	PORTD |= (1<<motor1_pinB);							
	//M2
	PORTD &= ~(1<<motor2_pinA);
	PORTD |= (1<<motor2_pinB);
	
}

void stop (){											// Car stops.
	//M1
	OCR2B = 0;											// Sets the speed of both motors at 0.
	OCR2A = 0;
	PORTD &= ~(1<<motor1_pinA);							// No current goes to the motors.
	PORTD &= ~(1<<motor1_pinB);
	//M2
	PORTD &= ~(1<<motor2_pinA);
	PORTD &= ~(1<<motor2_pinB);
}

void left (int vel1, int vel2){							// Car goes right (The name of the function relates to the motor thats rotating, so if the left motor is rotating and the right one isn't the car goes right).
	//right motor is A	
	//left motor is B
	OCR2B = vel1;										// Sets the speed of both motors.
	OCR2A = vel2;
	PORTD |= (1<<motor1_pinA);							// Current flows in a certain direction allowing both motors to be in the forward direction.
	PORTD &= ~(1<<motor1_pinB);
}

void right (int vel1, int vel2){						// Car goes left (The name of the function relates to the motor thats rotating, so if the right motor is rotating and the left one isn't the car goes left).
	//right motor is A
	//left motor is B
	OCR2B = vel2;										// Sets the speed of both motors.
	OCR2A = vel1;
	PORTD |= (1<<motor1_pinA);							// Current flows in a certain direction allowing both motors to be in the forward direction.
	PORTD &= ~(1<<motor1_pinB);	
}
	
void start (int start_vel){								// Function that determines when the car starts.
	if ((PINC | 0b11000001) == 0b11000001)				// If every IR Sensor is detecting black then the car starts.	
		{
		forward(start_vel);								// The car starts going forward.
		flags.start_flag = 1;							// Changes the start_flag to 1.
		}
		}

void ir_sensors(	int start_vel, int low_speed){			// Function that determines when the car changes direction or stops when it has already started.
	sonar();												// Runs the sonar() funciton.
	if ((PINC | 0b11111101) == 0b11111101){					// If IR1 is detecting black...
		while(!((PINC | 0b11110111) == 0b11110111)){		// While IR3 isn't detecting black...
			right(start_vel,0);								// Car goes left.
		}		
		
		} else if ((PINC | 0b11011111) == 0b11011111){		// If IR5 is detecting black...
		while(!((PINC | 0b11110111) == 0b11110111)){		// While IR3 isn't detecting black...
			left(start_vel,0);								// Car goes right.
		}
		
		} else if ((PINC | 0b11101111) == 0b11101111){		// If IR4 is detecting black...
			left(start_vel,0);								// Car goes right.
		
		} else if ((PINC | 0b11111011) == 0b11111011){		// If IR2 is detecting black...
			right(start_vel,0);								// Car goes left.
			
		} else if (flags.flag_sonar==1){					// If Sonar detects an object...
			while(!(flags.flag_sonar==0)) {					// While it doesn't stop detecting....
				stop();										// Car stops.
				sonar();									// Reads the sonar again.
			}
			forward(80);									// Car goes forward with higher speed to be able to have enought startign torque.
		
		} else forward(start_vel);							// Car goes forward normally.
	
}


int main(void)												// Main function
{	
	init();													// Runs the init() function.
	int velocity =	35;										// Sets main velocity of the car. 	
	while (1)												// While loop.
	{
		flags.start_flag = 0;								// Sets start flag to 0. Just a precaution. 
		sonar();											// Runs the sonar() function.
		if(flags.flag_sonar == 0) {							// If sonar isn't detecting anything...
		start(velocity);									// Starts the car.
		} else stop();										// If the above conditions weren't met then the car stops.
		while(flags.start_flag == 1){						// If the car has started...
			ir_sensors(velocity, 25);						// Run the function ir_sensors on a loop. 
			}
		}
	}


ISR(TIMER0_COMPA_vect)										// Interruption generated by Timer0.
{
	cnt--;													// Decrements the counter.
	if (cnt==0){											// If the counter has reached 0 then...
		PORTB ^= (1<<led_1hZ);								// Swap the state of the bit (0 to 1 or 1 to 0).
		cnt = 100;											// Return counter to 100. 
	}
}

ISR(INT0_vect)												// Interruption for the sonar.	
{
	if(sonf == 0)											// The sonf is a flag that makes sure that the Low-High state change initiates the timer and the High-Low state change stops the timer.
	{
		TCCR1B |= 1<<CS10;									// By giving it a prescaler we initiate the timer.			
		TCNT1 = 0;											// We set TCNT1 to zero in order to clear it to start the count.
		sonf = 1;											
	}
	else
	{
		wave_time = TCNT1;									// Retrieving how much time passed.
		TCCR1B = 0;											// Turning off the Timer.
		sonf = 0;
	}
}

