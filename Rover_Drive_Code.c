/*
 * File:   MT_Rover_Code.c
 * Author: cmiles10
 *
 * Created on November 5, 2025, 2:04 PM
 */


#include "xc.h"

// select oscillator
#pragma config FNOSC = LPRC // 31 khz

int main(void){/
    
    // configure pins
    ANSB = 0;
    ANSA = 0;
    
    // configure output or input
    _TRISA0 = 0; // pin 2 (direction pin)
    _TRISA1 = 0; // pin 3 (direction pin)
                 // pin 4 (left motor PWM pin OC2)
    _TRISB1 = 1; // pin 5 (a/d pin AN3)
    _TRISB2 = 1; // pin 6 (a/d pin AN4)
    _TRISA2 = 1; // pin 7 (a/d pin AN13)
                 // pin 14 (right motor PWM pin OC1)
    
    // set initial state of output pins
    _LATA0 = 0;
    _LATA1 = 1;
    
    // configure PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    
    while(1){
        
        // line detection
        if (ADC1BUF4 >= 1000){
            
            // left QRD activated
            if (ADC1BUF3 >= 1000){
                
                // right motor normal
                OC1RS = 78;
                OC1R = 39;
                
                // left motor slow
                OC2RS = 39;
                OC2R = 20;
                
            }
            
            // right QRD activated
            else if (ADC1BUF13 >= 1000){
                
                // left motor normal
                OC2RS = 78;
                OC2R = 39;
                
                // right motor slow
                OC1RS = 39;
                OC1R = 20;
                
            }
            
            else {
                
                // set PWM equal
                OC1RS = 78;
                OC1R = 39;
                OC2RS = 78;
                OC2R = 39;
            
            }
               
        }
        
    }
    
return 0;

}
