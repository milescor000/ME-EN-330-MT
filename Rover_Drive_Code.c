/*
 * File:   Rover_Drive_Code.c
 * Author: cmiles10
 *
 * Created on October 29, 2025, 11:26 AM
 */


#include "xc.h"

// select oscillator
#pragma config FNOSC = LPRC // 31 khz

// global variables
int forward_steps = 0;
int turning_steps = 0;
int s = 0;


// OC1 Interrupt Service Routine
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void)
{
    // When the OC1 Interrupt is activated, the code will jump up here
    // each time your PIC generates a PWM step
    // Add code to clear the flag and increment the step count each time
    _OC1IF = 0;
    forward_steps++;
}

// OC2 Interrupt Service Routine
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    // When the OC1 Interrupt is activated, the code will jump up here
    // each time your PIC generates a PWM step
    // Add code to clear the flag and increment the step count each time
    _OC2IF = 0;
    turning_steps++;
}

int main(void) {
    
    // states
    enum { forward, full_turn, half_turn } state;
    
    // configure pins
    ANSB = 0;
    ANSA = 0;
    
    // configure output or input
    _TRISA0 = 0;
    _TRISA1 = 0;
    
    // set initial state of output pins
    _LATA0 = 0;
    _LATA1 = 1;
    
    // configure PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    
    // PWM period
    OC1RS = 78;
        
    // duty cycle
    OC1R = 39;
    
    // initialize OC1
    _OC1IP = 4;
    _OC1IE = 1;
    _OC1IF = 0;
    
    // initialize state
    state = forward;
    
    // initialize while loop
    while(1){
        
        // state machine
        // forward state
        if (state == forward){
            
            // s = 0 for full_turn
            if (s == 0){
                
                // transition to full_turn
                if (forward_steps >= 1000){
                    
                // disable OC1
                _OC1IE = 0;
                
                // enable OC2
                _OC2IP = 4;
                _OC2IE = 1;
                _OC2IF = 0;
                
                // change directions
                _LATA0 = 0;
                _LATA1 = 0;
                
                // set turning_steps
                turning_steps = 0;
                
                // change state
                state = full_turn;
                
                }
                
            }
            
            // s = 1 for half-turn
            else{
                
                // transition to full_turn
                if (forward_steps >= 1000){
                    
                // disable OC1
                _OC1IE = 0;
                
                // enable OC2
                _OC2IP = 4;
                _OC2IE = 1;
                _OC2IF = 0;
                
                // change directions
                _LATA0 = 0;
                _LATA1 = 0;
                
                // set turning_steps
                turning_steps = 0;
                
                // change state
                state = half_turn;
                
                }
                
            }
            
        }
        
        // full_turn (180 degree) state
        if (state == full_turn){
                
            if (turning_steps >= 200){
                
                // disable OC2
                _OC2IE = 0;
                
                // enable OC1
                _OC1IP = 4;
                _OC1IE = 1;
                _OC1IF = 0;
                
                // change directions
                _LATA0 = 0;
                _LATA1 = 1;
                
                // change s
                s = 1;
                
                // reset forward_steps
                forward_steps = 0;
                
                // change state
                state = forward;
                
            }
        }
            
        // half_turn (90 degree) state
        if (state == half_turn){
            
            if (turning_steps >= 100){
                
                // disable OC2
                _OC2IE = 0;
                
                // enable OC1
                _OC1IP = 4;
                _OC1IE = 1;
                _OC1IF = 0;
                
                // change directions
                _LATA0 = 0;
                _LATA1 = 1;
                
                // change s
                s = 0;
                
                // reset forward_steps
                forward_steps = 0;
                
                // change state
                state = forward;
                
            }
        
        }
        
    }
            
return 0;

}
    
    
