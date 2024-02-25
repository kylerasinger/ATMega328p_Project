#include "IrSensor.h"
#include "../Utilities/Utilities.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

//helper function
// double map(double x, double in_min, double in_max, double out_min, double out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }


IrSensor::IrSensor(int irInPin, UART* uart) : irInPin(irInPin){
    ADCSRA |= (1 << ADEN) | (1 << ADPS2); //Enable ADC and set the prescaler division factor to 16
    ADMUX |= irInPin | (1 << REFS0); // sets AVcc as the voltage reference
}

uint16_t IrSensor::getDistance(){
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
    uart->println(buffer3);

    float voltageIR = map(ir_val2, 0, 1023, 0, 3200)/1000; //all in milliVolts 

    distance = 20 * pow(voltageIR, -0.9); //all in milliVolts
    distance *= calibrationFloat;

    return (int)distance;
}
