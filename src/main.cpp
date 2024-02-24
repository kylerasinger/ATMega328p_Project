#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "UART.h"
#include "UsSensor.h"

//define pins for the US sensor
#define US_TRIG PB4
#define US_ECHO PB0

//define pins for the IR
#define IR_IN PC0

//define LED pin
#define LED_PIN PB3
#define Y_LED_PIN PB5

//function prototypes
uint16_t get_us_dist();
double map(double x, double in_min, double in_max, double out_min, double out_max);
uint16_t get_ir_dist();
int get_us_pulse();

//Initialize objects
UART uart;
UsSensor usSensor(US_ECHO, US_TRIG, &uart);

// uint16_t get_us_dist(){
//   int maxDistCM = 400;
//   int minDistCM = 2;
//   int temp = 22.8;

//   float speedOfSoundCMperMicroSec = 0.03313 + (0.0000606 * temp);  //speed of sound depending on temperature

//   //pulse the US sensors input, triggering the US's cycle
//   PORTB &= ~(1 << US_TRIG);
//   _delay_us(2);
//   PORTB |= (1 << US_TRIG);
//   _delay_us(10);
//   PORTB &= ~(1 << US_TRIG);

//   //calculate ultrasound pulse
//   int pulseUS = get_us_pulse();

//   char buffer2[32];
//   sprintf(buffer2, "PulseUS: %u us", pulseUS);
//   uart.println(buffer2);

//   float calibrationFloat = 1.30434;
//   uint16_t distCM = ((pulseUS * calibrationFloat) * speedOfSoundCMperMicroSec)/2; //pulseUS is the pulse length in microseconds
  
//   //out of range error catch
//   if(distCM < minDistCM || distCM > maxDistCM){
//     return 0;
//   }

//   return distCM;
// }

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t get_ir_dist(){
  float calibrationFloat = 1;
  double ir_val2;
  float distance;

  // ADCSRA &= ~(1 << ADPS0); // Clear ADPS0 to ensure ADC prescaler is not at minimum value.

  ADCSRA |= (1 << ADSC); // Start ADC conversion by setting ADSC bit.

  while (!(ADCSRA & (1 << ADIF))); // Wait for ADC conversion to complete (ADIF bit set to 1).

  ADCSRA |= (1 << ADIF); // Clear ADIF bit by writing 1 to it, resetting ADC interrupt flag.

  ir_val2 = ADC; // Read ADC conversion result from ADC Data Register.

  char buffer3[32];
  sprintf(buffer3, "ADC Value: %u on 1023", ADC);
  uart.println(buffer3);

  float voltageIR = map(ir_val2, 0, 1023, 0, 3200)/1000; //all in milliVolts 

  distance = 20 * pow(voltageIR, -0.9); //all in milliVolts
  distance *= calibrationFloat;

  return (int)distance;
}

// int get_us_pulse(){
//   //this function returns the length of teh ultrasound pulse but in microseconds.
//   uint16_t pulseDuration = 0;
  
//   TCCR2B = 0; //stop timer 2
//   TCNT2 = 0;  //reset timer 2

//   //while echo pin is high
//   while (!(PINB & (1 << US_ECHO))); 

//   //start timer (no prescaler because of the reset done 6 lines up)
//   TCCR2B |= (1 << CS20);

//   //while echo pin is low
//   while (PINB & (1 << US_ECHO)) {
//     if (TCNT2 == 255) {  //counts and checks for overflow TRY REMOVING THIS, I SHOULDNT HAVE ANY OVERFLOW REMOVE
//       pulseDuration += 255;  //on each overflow
//       TCNT2 = 0;  // Reset Timer2 to count from 0 again
//     }
//   }
//   pulseDuration += TCNT2;

//   //stop the timer
//   TCCR2B &= ~(1 << CS20);

//   pulseDuration = pulseDuration/4;
//   return pulseDuration;
// }

void set_brightness(int brightness){
  OCR2A = brightness;
}

int main(void)
{
  //true for US
  //false for IR
  const bool MODE = true;
  

  /* --===-- Setup US Sensor --===-- */
  //sets the US_TRIG as an output (1)
  // DDRB |= (1 << US_TRIG);
  //sets the US_ECHO as an input (0)
  // DDRB &= ~(1 << US_ECHO);
  
  /* --===-- Setup IR Sensor --===-- */
  ADCSRA |= (1 << ADEN) | (1 << ADPS2);
  ADMUX |= IR_IN | (1 << REFS0);


  /* --===-- Setup LED --===-- */
  //sets the LED pin as an output
  DDRB |= (1 << LED_PIN);
  PORTB |= (1 << LED_PIN);    

  /* --===-- Setup Serial Coms --===-- */
  //enable transmitter and receiver for USART
  UCSR0B = (1 << RXEN0)|(1 << TXEN0);
  //Sets the frame format as 8 bits for serial comms
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  //we are using a baud rate of 9600 bps at 16mhz because it is standard
  UBRR0H = 0;
  UBRR0L = 103; 
  
  /* --===-- Setup Timers --===-- */
  //this is used in ADC aswell as the US
  //using 8 bit timer with fast PWM, we can use it for US without worrying about
    //an overflow because the max distance that can be read from the timer is approx 550cm
    //and our sensor can only go to 400cm
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << CS22); //prescaler at 64, 128 is "slowmode" MIGHT BE ABLE TO REMOVE IF IR AND US BOTH STILL WORK

  //we use uint16_t because uint8_t wont fit the entire range of sensors.
  uint16_t distance = 0;

  while (1)
  {
    distance = (MODE == true) ? usSensor.getDistance() : get_ir_dist();

    if(distance < 15){
      //set led brightness to 100
      set_brightness(255);
      //turn on yellow led
      PORTB |= (1 << Y_LED_PIN);
    }else if(distance >= 15 && distance < 40){
      //set led brightness to a linear percent
      set_brightness(map(distance, 15, 45, 255, 0));
      PORTB &= ~(1 << Y_LED_PIN);
    }else if(distance >= 40){
      set_brightness(0);
      PORTB |= (1 << Y_LED_PIN);
    }

    char buffer[32];
    sprintf(buffer, "Distance: %u cm", distance);
    uart.println(buffer);

    _delay_ms(250);
  }
}
