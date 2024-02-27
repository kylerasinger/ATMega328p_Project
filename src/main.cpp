#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR

//personally made classes and helper functions
#include "Communication/UART.h"
#include "Communication/LED.h"
#include "Sensors/UsSensor.h"
#include "Sensors/IrSensor.h"
#include "Sensors/MPU6050.h"
#include "Servo/Servo.h"
#include "Utilities/Utilities.h"
#include "Utilities/I2C.h"
#include <stdlib.h>


/* ---===       PIN DECLARATION      ===--- */

  //define pins for the US sensor
  #define US_TRIG PB4
  #define US_ECHO PB0

  //define pins for the IR
  #define IR_IN PC0

  //define LED pin
  #define LED_PIN PB3
  #define Y_LED_PIN PB5

  //define IMU 
  #define I2C_DATA PC4
  #define I2C_CLOCK PC5
  #define MPU6050_ADDRESS 0x68

  //define Servo pins
  #define SERVO_PIN PB1

/* ---===   END OF PIN DECLARATION   ===--- */


// void servoInit(){
//   //set servo pin as output
//   DDRB |= (1 << SERVO_PIN);

//   // Set Fast PWM mode with non-inverted output
//     //COM1A1 clears OC1A on compare match (set output to low level)
//     //WGM11 sets waveform generation mode to PWM phase correct 9 bit 
//       //with a top at 0x01FF
//       //update of OCR1A at bottom
//   TCCR1A |= (1 << COM1A1) | (1 << WGM11);
//     //creates a FAST PWM with a top of ICR1 and updates OCR1X at bottom, TOV1 flag set to 1 at top
//   TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // CS11 creates prescaler of 8

//   //set TOP value for a 20ms period
//   ICR1 = 39999;
// }

// void setServoAngle(uint8_t angle) {
//   // Ensure the angle is within the 0 to 180 range
//   if (angle > 180) angle = 180;

//   // float pulseWidth = ((angle / 180.0) * (2.1 - 0.9)) + 0.9; // Pulse width in ms

//   //1050 to 4850 gives the best results for the servo at 90 degrees
//   uint16_t ocrValue = map(angle, 0, 180, 1050, 4850); 

//   // uint16_t ocrValue = (uint16_t)((pulseWidth / 20.0) * 39999);

//   OCR1A = ocrValue; // Set the pulse width for the servo
// }

int main(void) {
  setUp8BitTimer();

  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  MPU6050 mpu(MPU6050_ADDRESS, 0b00000000, 0b00000000, &largeLED);
  Servo servo(SERVO_PIN);

  largeLED.setBrightness(0);


  while(true){
    _delay_ms(1000);
    mpu.readSensor(&largeLED, &uart);

    _delay_ms(1000);
    servo.setServoAngle(0);
    _delay_ms(1000);
    servo.setServoAngle(90);
    _delay_ms(1000);
    servo.setServoAngle(180);

  }
}

