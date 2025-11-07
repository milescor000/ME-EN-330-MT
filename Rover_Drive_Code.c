/*
 * File:   MT_Rover_Code_2.c
 * Author: cmiles10
 *
 * Created on November 6, 2025, 3:38 PM
 */


#include "xc.h"

// select oscillator
#pragma config FNOSC = LPFRC // 31 khz

// turn of pin 8 clock
#pragma config OSCIOFNC = OFF

// turn of secondary oscillator
#pragma config SOSCSRC = DIG

void line_straight(void){
    
    // go straight
    _LATA0 = 0;
    _LATA1 = 1;
        
    // both motors equal
    OC1RS = 500;
    OC1R = 250;
    OC2RS = 500;
    OC2R = 250;
    
}

void line_left(void){
        
    // 
    OC1RS = 500;
    OC1R = 250;
    OC2RS = 1000;
    OC2R = 500;
    
}

void line_right(void){
        
    // both motors equal
    OC1RS = 1000;
    OC1R = 500;
    OC2RS = 500;
    OC2R = 250;
    
}

void config_ad(void){
    
    _ADON = 0;          // AD1CON1<15> -- Turn off A/D during config
    
    // Clear all A/D registers
    AD1CON1 = 0; 
    AD1CON2 = 0; 
    AD1CON3 = 0; 
    AD1CON5 = 0; 
    AD1CSSL = 0; 
    AD1CSSH = 0; 
    
    // AD1CON1 register
    _ADSIDL = 0;    // AD1CON1<13> -- A/D continues while in idle mode
    _MODE12 = 1;    // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;      // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;      // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;      // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;     // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;     // AD1CON2<13> -- Use VSS as negative ref voltage
    _BUFREGEN = 1;  // AD1CON2<11> -- Result appears in buffer
                    // location corresponding to channel, e.g., AN12
                    // results appear in ADC1BUF12
    _CSCNA = 1;     // AD1CON2<10> -- Scans inputs specified in AD1CSSx
                    // registers
    _SMPI = 2;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
                    // For example, if you are sampling 4 channels, you
                    // should have _SMPI = 3;
    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register -- Change _SAMC and _ADCS based on your
    // selection of oscillator and postscaling
    _ADRC = 0;      // AD1CON3<15> -- Use system clock
    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
    
    // AD1CHS register
    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS
    _CSS4 = 1;
    _CSS13 = 1;
    _CSS15 = 1;
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
    
}

int main(void){
    
    // states
    enum { stop, linestraight, lineleft, lineright } state;
    
    // configure peripherals
    config_ad();
    _ANSA0 = 0;
    _ANSA1 = 0;
    
    // configure output or input
    _TRISA0 = 0; // pin 2 (right motor direction pin)
    _TRISA1 = 0; // pin 3 (left motor direction pin)
                 // pin 4 (left motor OC2)
    _TRISB2 = 1; // pin 6 (right QRD a/d pin AN4)
    _TRISA2 = 1; // pin 7 (middle QRD a/d pin AN13)
    _TRISB4 = 1; // pin 9 (left QRD a/d pin AN15)
                 // pin 14 (right motor OC1)
    
    // configure PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    
    // set state
    state = stop;
    
    // while loop
    while(1){
        
        if (state == stop){
            
            // initial PWM
            OC1RS = 0;
            OC1R = 0;
            OC2RS = 0;
            OC2R = 0;
            
            if (ADC1BUF13 < 500){
                
                state = linestraight;
                
            }
            
        }
        
        if (state == linestraight){
            
            line_straight();
            
            if (ADC1BUF15 < 500){
                
                state = lineleft;
                
            }
            
            if (ADC1BUF4 < 500){
                
                state = lineright;
                
            }
            
        }
        
        if (state == lineleft){
            
            line_left();
            
            if (ADC1BUF15 > 500){
              
                state = linestraight;
                
            }
            
        }
        
        if (state == lineright){
            
            line_right();
            
            if (ADC1BUF4 > 500){
                
                state = linestraight;
                
            }
            
        }
        
    }
    
return 0;

}
