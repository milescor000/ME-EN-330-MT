/*
 * File:   MT_Rover_Code_2.c
 * Author: cmiles10
 *
 * Created on November 6, 2025, 3:38 PM
 */

//---Set-Up---------------------------------------------------------------------
#include "xc.h"
#include <stdbool.h>

// select oscillator
#pragma config FNOSC = LPFRC // 31 khz

// turn of pin 8 clock
#pragma config OSCIOFNC = OFF

// turn of secondary oscillator
#pragma config SOSCSRC = DIG
//------------------------------------------------------------------------------

//---global variables-----------------------------------------------------------
int norm_speed = 150;
int slow_line = 2000;
int medium_line = 100;
int fast_line = 30;
int turn_speed = 78;
int qrd_thresh = 2000;
int steps = 0;
int turn90 = 640;
int reverse90 = 590;
int ball_back = 200;
int ball_forward = 700;
int ball_rforward = 600;
int ballwait_time = 3000;
bool ball_wait = true;
int ball_exit = 800;
int canyon_back = 250;
int canyon_turn90 = 600;
//------------------------------------------------------------------------------

//---OC1 interrupt--------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
 
    _OC1IF = 0;
    steps++;

}
//------------------------------------------------------------------------------

//---TMR1 INTERRUPt
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    
    _T1IF = 0; // clear interrupt flag
    ball_wait = false; // set ball wait to false
    
}
//------------------------------------------------------------------------------

//---line_straight function-----------------------------------------------------
void drive_straight(void){
    
    // go straight
    _LATA0 = 0;
    _LATA1 = 0;
        
    // both motors equal
    OC1RS = norm_speed;
    OC1R = OC1RS/2;
    OC2RS = norm_speed;
    OC2R = OC2RS/2;
    
}
//------------------------------------------------------------------------------

//---line_left function---------------------------------------------------------
void line_left(void){

    // check line middle
    if (ADC1BUF13 < qrd_thresh){
        OC1RS = medium_line;
        OC1R = OC1RS/2;
       OC2RS = slow_line;
        OC2R = OC2RS/2;
    }
    
    // no line middle
    else{
        OC1RS = fast_line;
        OC1R = OC1RS/2;
        OC2RS = slow_line;
        OC2R = OC2RS/2; 
    }
    
}
//------------------------------------------------------------------------------

//---line_right function--------------------------------------------------------
void line_right(void){
    
    // check line middle
    if (ADC1BUF13 < qrd_thresh){
        OC1RS = slow_line;
        OC1R = OC1RS/2;
        OC2RS = medium_line;
        OC2R = OC2RS/2;
    }
    
    // no line middle
    else{
        OC1RS = slow_line;
        OC1R = OC1RS/2;
        OC2RS = fast_line;
        OC2R = OC2RS/2;
    }
    
}
//------------------------------------------------------------------------------

//---backwards direction function-----------------------------------------------
void drive_back(void) {
    
_LATA0 = 1; // right dir
_LATA1 = 1; // left dir
OC1RS = norm_speed; OC1R = OC1RS / 2;
OC2RS = norm_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---turn left function---------------------------------------------------------
void turn_left(void) {
    
_LATA0 = 0; // right motor backward
_LATA1 = 1; // left motor forward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---turn right function--------------------------------------------------------
void turn_right(void) {
    
_LATA0 = 1; // right motor forward
_LATA1 = 0; // left motor backward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---stop function--------------------------------------------------------------
void stop_func(void) {
    
_LATA0 = 0; // right dir
_LATA1 = 0; // left dir
OC1RS = 0; OC1R = OC1RS / 2;
OC2RS = 0; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---configure ad conversion----------------------------------------------------
void config_ad(void){
    
    // AD1CON1<15> -- Turn off A/D during config
    _ADON = 0;
    
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
    _SMPI = 3;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
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
    _CSS12 = 1;
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
    
}
//------------------------------------------------------------------------------

//---main robot behavior--------------------------------------------------------
int main(void){
    
    // states
    enum { linestraight, lineleft, lineright, ballback, 
    ballright, ballforward, ballwait, rballforward, rballright,
    ballexit, canyonstraight, canyonback, canyonright, canyonleft, stop } state;
    
    // configure peripherals
    config_ad();
    _ANSA0 = 0;
    _ANSA1 = 0;
    _ANSB13 = 0;
    _ANSB15 = 0;
    _ANSB14 = 0;
    
    // configure output or input
    _TRISA0 = 0; // pin 2 (right motor direction pin)
    _TRISA1 = 0; // pin 3 (left motor direction pin)
                 // pin 4 (left motor PWM OC2)
                 // pin 5 (servo PWM OC3)
    _TRISB2 = 1; // pin 6 (right QRD a/d pin AN4)
    _TRISA2 = 1; // pin 7 (middle QRD a/d pin AN13)
    _TRISB4 = 1; // pin 9 (left QRD a/d pin AN15)
                 // pin 14 (right motor OC1)
    _TRISB12 = 1; // pin 15 (ball color QRD AN12)
    _TRISB13 = 1; // pin 16 (right IR)
    _TRISB14 = 1; // pin 17 (front IR)
    _TRISB15 = 1; // pin 18 (left IR)
   
    // configure PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC3CON1 = 0x1C06;
    OC3CON2 = 0x001F;
    
    // initiate OC1 interrupt
    _OC1IE = 1;
     
    // configure TMR1
    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
    
    // initiate TMR1 interrupt
    _T1IP = 4; // select interrupt priority
    _T1IF = 0; // clear interrupt flag
    _T1IE = 1; // enable interrupt
    PR1 = ballwait_time; // TMR1 period
    
    // set initial state
    state = linestraight;
    
    // while loop
    while(1){
        
        switch (state){
        
            //---linestraight state---------------------------------------------
            case linestraight:

                // execute drive_straight function
                drive_straight();

                // check line left
                if (ADC1BUF15 < qrd_thresh){

                    // change state to lineleft
                    state = lineleft;

                }

                // check line right
                if (ADC1BUF4 < qrd_thresh){

                    // change state to lineright
                    state = lineright;

                }
                
                // check right IR
                if (_RB13 == 0 && _RB15 == 1 && ADC1BUF13 < qrd_thresh){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballback
                    state = ballback;
                     
                }
                
                // check left IR
                if (_RB15 == 0 && _RB13 == 0 && ADC1BUF13 < qrd_thresh){
                    
                    // change state to stop
                    state = stop;
                    
                }
                
                // check for canyon
                if (_RB15 == 0 && _RB13 == 0 &&  ADC1BUF13 > qrd_thresh){
                    
                    // change state to canyonstraight
                    state = canyonstraight;
                    
                }

                break;
            //------------------------------------------------------------------

            //---lineleft state-------------------------------------------------
            case lineleft:

                // execute line_left function
                line_left();

                // check no line left
                if (ADC1BUF15 > qrd_thresh){

                    // change state to linestraight
                    state = linestraight;

                }

                break;
            //------------------------------------------------------------------

            //---lineright state------------------------------------------------
            case lineright:

                // execute line_right function
                line_right();

                // check no line right
                if (ADC1BUF4 > qrd_thresh){

                    // change state to linestraight
                    state = linestraight;

                }

                break;
            //------------------------------------------------------------------
                
            //---ballback state-------------------------------------------------
            case ballback:
                
                // execute drive_back function
                drive_back();
                
                // check step count
                if (steps > ball_back){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballright
                    state = ballright;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballright state-------------------------------------------------
            case ballright:
                
                // execute turn_right function
                turn_right();
                
                // check step count
                if (steps > turn90){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballforward
                    state = ballforward;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballforward state----------------------------------------------
            case ballforward:
                
                // execute drive_straight function
                drive_straight();
                
                // check step count
                if (steps > ball_forward){
                    
                    // reset timer
                    TMR1 = 0;
                    
                    // reset ball_wait
                    ball_wait = true;
                    
                    // change state to ballwait
                    state = ballwait;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballwait state-------------------------------------------------
            case ballwait:
                
                // execute stop_func function
                stop_func();
                
                // check timer
                if (ball_wait == false){
                    
                    // reset step count
                    steps = 0;
                    
                    // change state to rballforward
                    state = rballforward;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---rballforward state---------------------------------------------
            case rballforward:
                
                // execute drive_back function
                drive_back();
                
                //check step count
                if (steps > ball_rforward){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to rballright
                    state = rballright;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---rballright state------------------------------------------------
            case rballright:
                
                // execute turn_left function
                turn_left();
                
                // check step count
                if (steps > reverse90){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballexit
                    state = ballexit;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballexit state-------------------------------------------------
            case ballexit:
                
                // execute drive_straight function
                drive_straight();
                
                // check step count
                if (steps > ball_exit){
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonstraight state-------------------------------------------
            case canyonstraight:
                
                // execute drive_straight function
                drive_straight();
                
                // check wall front
                if (_RB14 == 0){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to canyonback
                    state = canyonback;
                    
                }
                break;
            //------------------------------------------------------------------
                
            //---canyonback state-----------------------------------------------
            case canyonback:
                
                // execute drive_back function
                drive_back();
                
                // check step count
                if (steps > canyon_back){
                    
                    // check wall right
                    if (_RB13 == 0){
                        
                        // reset steps
                        steps = 0;
                        
                        // change state to canyonleft
                        state = canyonleft;
                    }
                    
                    // check wall left
                    else{
                        
                        // reset steps
                        steps = 0;
                        
                        // change state to canyonright
                        state = canyonright;
                
                    }
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonright state----------------------------------------------
            case canyonright:
                
                // execute turn right function
                turn_right();
                
                // check step count
                if (steps > canyon_turn90) {
                    
                    // reset steps
                    steps=0;
                    
                    // change state to canyon straight
                    state = canyonstraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonleft state-----------------------------------------------
                case canyonleft:
                
                // execute turn right function
                turn_left();
                
                // check step count
                if (steps > canyon_turn90) {
                    
                    // reset steps
                    steps=0;
                    
                    // change state to canyon straight
                    state = canyonstraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---stop state-----------------------------------------------------
            case stop:
                
                // execute stop_func function
                stop_func();
                
                break;
            //------------------------------------------------------------------
            
        }
        
    }
    
return 0;

}
//------------------------------------------------------------------------------
