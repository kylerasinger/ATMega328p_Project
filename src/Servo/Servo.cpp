#include "Servo.h"
#include "./Utilities/Utilities.h"

Servo::Servo(uint8_t servoPin){
	//set servo pin as output
  DDRB |= (1 << servoPin);

  // Set Fast PWM mode with non-inverted output
    //COM1A1 clears OC1A on compare match (set output to low level)
    //WGM11 sets waveform generation mode to PWM phase correct 9 bit 
      //with a top at 0x01FF
      //update of OCR1A at bottom
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
	//creates a FAST PWM with a top of ICR1 and updates OCR1X at bottom, TOV1 flag set to 1 at top
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // CS11 creates prescaler of 8

  //set TOP value for a 20ms period
  ICR1 = 39999;
}

void Servo::setServoAngle(uint8_t angle){
	// Ensure the angle is within the 0 to 180 range
  if (angle > 180) angle = 180;

  // float pulseWidth = ((angle / 180.0) * (2.1 - 0.9)) + 0.9; // Pulse width in ms

  //1050 to 4850 gives the best results for the servo at 90 degrees
  uint16_t ocrValue = map(angle, 0, 180, 1050, 4850); 

  // uint16_t ocrValue = (uint16_t)((pulseWidth / 20.0) * 39999);

  OCR1A = ocrValue; // Set the pulse width for the servo
}