/*
 * File:   MT_Rover_Code_2.c
 * Author: cmiles10
 *
 * Created on November 6, 2025, 3:38 PM
 */

//---to-do----------------------------------------------------------------------
// canyon navigation not well tuned
// turning and backing up not consistent enough
// too slow
// satellite task
// reenter lander
//------------------------------------------------------------------------------

//---set-up---------------------------------------------------------------------
#include "xc.h"
#include <stdbool.h>

// select oscillator
#pragma config FNOSC = LPFRC // 500 khz

// turn of pin 8 clock
#pragma config OSCIOFNC = OFF

// turn of secondary oscillator
#pragma config SOSCSRC = DIG
//------------------------------------------------------------------------------

//---global variables-----------------------------------------------------------

// speeds
int norm_speed = 100;
int slow_line = 2000;
int medium_line = 50;
int fast_line = 25;
int canyon_speed = 150;
int deposit_speed = 1500;
int turn_speed = 78;

// sensing
int qrd_thresh = 2000;
int diode_thresh = 500;

// counters
int steps = 0;
int servo_increment = 7;
int final_increment = 15;

// turns
int turn90 = 615;
int reverse90 = 590;
int canyonright_90 = 633;
int canyonleft_90 = 625;
int lander_turn90 = 500;

// backwards
int ball_reverse = 800;
int lander_reverse = 1600;

// adjusts
int pickup_adjust = 380;
int canyon_adjust = 385;
int deposit_adjust = 150;
int lander_adjust = 250;

// forwards
int ball_forward = 900;

// exits
int ball_exit = 200;

// timer periods
int wait_time = 3000;
int short_time = 1500;
int medium_time = 2000;

// booleans
bool wait = true;
bool canyon_complete = false;
bool ballpickup_complete = false;
bool balldeposit_complete = false;

// servos
int servo_left = 63;
int servo_middle = 188;
int servo_right = 313;
int servo_start = 150;
int servo_pwm = 2499;
//------------------------------------------------------------------------------

//---OC1 interrupt--------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
 
    _OC1IF = 0;
    steps++;

}
//------------------------------------------------------------------------------

//---TMR1 interrupt-------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    
    _T1IF = 0; // clear interrupt flag
    wait = false; // set ball wait to false
    
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
    
_LATA0 = 1;
_LATA1 = 1;
OC1RS = norm_speed; OC1R = OC1RS / 2;
OC2RS = norm_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---turn_left function---------------------------------------------------------
void turn_left(void) {
    
_LATA0 = 0; // right motor backward
_LATA1 = 1; // left motor forward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---turn_right function--------------------------------------------------------
void turn_right(void) {
    
_LATA0 = 1; // right motor forward
_LATA1 = 0; // left motor backward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---canyon_forward function----------------------------------------------------
void canyon_forward(void){
    
    // go straight
    _LATA0 = 0;
    _LATA1 = 0;
        
    // both motors equal
    OC1RS = canyon_speed;
    OC1R = OC1RS/2;
    OC2RS = canyon_speed;
    OC2R = OC2RS/2;
    
}
//------------------------------------------------------------------------------

//---canyon_back function-------------------------------------------------------
void canyon_back(void){
    
_LATA0 = 1;
_LATA1 = 1;
OC1RS = canyon_speed; OC1R = OC1RS / 2;
OC2RS = canyon_speed; OC2R = OC2RS / 2;

}
//------------------------------------------------------------------------------

//---stop_func function---------------------------------------------------------
void stop_func(void){
    
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
    _SMPI = 4;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
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
    _CSS11 = 1;
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
    
}
//------------------------------------------------------------------------------

//---main robot behavior--------------------------------------------------------
int main(void){
    
    // states
    enum { startstraight, startleft, linestraight, lineleft, lineright, balladjust, 
        ballpause, ballright, ballforward, ballwait, ballreverse, ballleft,
        ballexit, canyonstraight, canyonadjust, canyonright, canyonleft,
        canyonexitright, canyonexitleft, depositadjust, balldeposit, 
        depositleft, depositright, depositexit, landeradjust, 
        landerright, landerreverse, roveroff, laserincrement, laserdelay, 
        firelaser, missioncomplete } state;
    
    // configure peripherals
    config_ad();
    _ANSA0 = 0;
    _ANSA1 = 0;
    _ANSB15 = 0;
    _ANSB14 = 0;
    
    // configure output or input
    _TRISA0 = 0; // pin 2 (right motor direction pin A0)
    _TRISA1 = 0; // pin 3 (left motor direction pin A1)
                 // pin 4 (left motor PWM OC2)
                 // pin 5 (servo PWM OC3)
    _TRISB2 = 1; // pin 6 (right QRD a/d pin AN4)
    _TRISA2 = 1; // pin 7 (middle QRD a/d pin AN13)
    _TRISB4 = 1; // pin 9 (left QRD a/d pin AN15)
    _TRISA4 = 1; // pin 10 (right IR A4)
    _TRISB7 = 0; // pin 11 (laser power B7)
    _TRISB8 = 1; // pin 13 (underside IR B8)
                 // pin 14 (right motor OC1)
    _TRISB12 = 1; // pin 15 (ball color QRD AN12)
    _TRISB13 = 1; // pin 16 (diode AN11)
    _TRISB14 = 1; // pin 17 (front IR B14)
    _TRISB15 = 1; // pin 18 (left IR B15)
   
    // configure PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC3CON1 = 0x1C06;
    OC3CON2 = 0x001F;
 
    // initiate OC1 interrupt
    _OC1IE = 1;
    
    // initialize servo position (middle)
    OC3RS = servo_pwm;
    OC3R = servo_middle;
    
    // initiate TMR1
    PR1 = wait_time; // TMR1 period
    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
    
    // initiate TMR1 interrupt
    _T1IP = 4; // select interrupt priority
    _T1IF = 0; // clear interrupt flag
    _T1IE = 1; // enable interrupt
    
    // set initial state
    state = startstraight;
    
    // while loop
    while(1){
        
        switch (state){

            //---startstraight state--------------------------------------------
            case startstraight:
                
                // execute drive_straight function
                drive_straight();
                
                // check for path
                if (ADC1BUF13 < qrd_thresh && ADC1BUF15 < qrd_thresh 
                        && ADC1BUF4 < qrd_thresh){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to startleft
                    state = startleft;
                    
                }
                
                break;
            //------------------------------------------------------------------

            //---startleft state------------------------------------------------
            case startleft:
                
                // execute turn_left function
                turn_left();
                
                // check step count
                if (steps > turn90){
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
        
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
                
                // check right IR for ball pickup
                if (_RA4 == 0 && _RB15 == 1 && ADC1BUF13 < qrd_thresh && 
                        ballpickup_complete == false){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to balladjust
                    state = balladjust;
                     
                }
                
                // check left IR for ball deposit
                if (_RB15 == 0 && _RA4 == 1 && ADC1BUF13 < qrd_thresh 
                    && ballpickup_complete == true && balldeposit_complete == false){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to stop
                    state = depositadjust;
                    
                }
                
                // check for canyon
                if (_RB14 == 0 && ADC1BUF13 > qrd_thresh){
                    
                    // change state to canyonstraight
                    state = canyonstraight;
                    
                }
                
                // check for lander
                if (_RB8 == 0 && canyon_complete == true 
                    && ballpickup_complete == true 
                    && balldeposit_complete == true){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to landeradjust
                    state = landeradjust;
                    
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
                
            //---balladjust state-------------------------------------------------
            case balladjust:
                
                // execute drive_back function
                drive_back();
                
                // check step count
                if (steps > pickup_adjust){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballright
                    state = ballright;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballright state------------------------------------------------
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
                    
                    // reset wait
                    wait = true;
                    
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
                if (wait == false){
                    
                    // reset step count
                    steps = 0;
                    
                    // change state to ballreverse
                    state = ballreverse;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballreverse state---------------------------------------------
            case ballreverse:
                
                // execute drive_back function
                drive_back();
                
                //check step count
                if (steps > ball_reverse){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to ballleft
                    state = ballleft;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---ballleft state------------------------------------------------
            case ballleft:
                
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

                    // set ballpickup_complete as true
                    ballpickup_complete = true;
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------

            //---canyonstraight state-------------------------------------------
            case canyonstraight:
                
                // execute canyon_forward function
                canyon_forward();
                
                // check wall front
                if (_RB14 == 0){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to canyonadjust
                    state = canyonadjust;
                    
                }
                
                // check exit conditions
                if (ADC1BUF13 < qrd_thresh){
                    
                    // reset steps
                    steps = 0;
                    
                    if (_RB15 == 0){
                        
                        // change state to canyonexitright
                        state = canyonexitright;
                        
                    }
                    
                    else{
                        
                        // change state to canyonexitleft
                        state = canyonexitleft;
                        
                    }
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonadjust state-----------------------------------------------
            case canyonadjust:
                
                // execute canyon_back function
                canyon_back();
                
                // check step count
                if (steps > canyon_adjust){
                    
                    // check wall left
                    if (_RB15 == 0){
                        
                        // reset steps
                        steps = 0;
                        
                        // change state to canyonright
                        state = canyonright;
                    }
                    
                    // check wall left
                    else{
                        
                        // reset steps
                        steps = 0;
                        
                        // change state to canyonleft
                        state = canyonleft;
                
                    }
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonright state----------------------------------------------
            case canyonright:
                
                // execute turn right function
                turn_right();
                
                // check step count
                if (steps > canyonright_90) {
                    
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
                if (steps > canyonleft_90) {
                    
                    // reset steps
                    steps=0;
                    
                    // change state to canyon straight
                    state = canyonstraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonexitright state------------------------------------------
            case canyonexitright:
                
                // execute turn_right function
                turn_right();
                
                // check step count
                if (steps > turn90){
                    
                    // set canyon_complete to true
                    canyon_complete = true;
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---canyonexitleft state------------------------------------------
            case canyonexitleft:
                
                // execute turn_right function
                turn_left();
                
                // check step count
                if (steps > turn90){
                    
                    // set canyon_complete to true
                    canyon_complete = true;
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---depositadjust state----------------------------------------------
            case depositadjust:
                
                // execute drive_back function
                _LATA0 = 1;
                _LATA1 = 1;
                OC1RS = deposit_speed; OC1R = OC1RS / 2;
                OC2RS = deposit_speed; OC2R = OC2RS / 2;
                
                if (steps > deposit_adjust){
                    
                    // change state to balldeposit
                    state = balldeposit;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---balldeposit state----------------------------------------------
            case balldeposit:
                
                // execute stop_func function
                stop_func();
                
                // check black ball
                if (ADC1BUF12 > qrd_thresh){
                    
                    // reset wait
                    wait = true;
                    
                    // reset timer
                    TMR1 = 0;
                    
                    // change state to depositright
                    state = depositright;
                    
                }
                
                // check ball white
                if (ADC1BUF12 < qrd_thresh){
                    
                    // reset wait
                    wait = true;
                    
                    // reset timer
                    TMR1 = 0;
                    
                    // change state to depositleft
                    state = depositleft;
                    
                }
                
                break;
            //------------------------------------------------------------------

            //---depositleft state----------------------------------------------
            case depositleft:
                
                // turn servo right for black
                OC3RS = servo_pwm;
                OC3R = servo_left;
                
                // check timer count
                if (wait == false){
                    
                    // reset steps
                    steps = 0;
                    
                    // reset servo
                    OC3RS = servo_pwm;
                    OC3R = servo_middle;
                    
                    // change state to linestraight
                    state = depositexit;
                    
                }
                
                break;
            //------------------------------------------------------------------
            
            //---depositright state---------------------------------------------
            case depositright:
                
                // turn servo right for black
                OC3RS = servo_pwm;
                OC3R = servo_right;
                
                // check timer count
                if (wait == false){
                    
                    // reset steps
                    steps = 0;
                    
                    // reset servo
                    OC3RS = servo_pwm;
                    OC3R = servo_middle;
                    
                    // change state to linestraight
                    state = depositexit;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---depositexit state----------------------------------------------
            case depositexit:
                
                // execute drive_straight function
                drive_straight();
                
                // check step count
                if (steps > ball_exit){

                    // set balldeposit_complete to true
                    balldeposit_complete = true;
                    
                    // change state to linestraight
                    state = linestraight;
                    
                }
                
                break;
            //------------------------------------------------------------------

            //---landeradjust state---------------------------------------------
            case landeradjust:
                
                // execute drive_back function
                drive_back();
                
                // check step count
                if (steps > lander_adjust){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to landerright
                    state = landerright;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---landerright state----------------------------------------------
            case landerright:
                
                // execute turn_right function
                turn_right();
                
                // check step count
                if (steps > lander_turn90){
                    
                    // reset steps
                    steps = 0;
                    
                    // change state to landerreverse
                    state = landerreverse;
                    
                }
                
                break;
            //------------------------------------------------------------------

            //---landerreverse state--------------------------------------------
            case landerreverse:
                
                // execute drive_back function
                drive_back();

                // check step count
                if (steps > lander_reverse){

                    // change state to roveroof
                    state = roveroff;

                }

                break;
            //------------------------------------------------------------------
            
            //---roveroff state-------------------------------------------------
            case roveroff:
                
                // execute stop_func function
                stop_func();
                    
                // set state to firelaser
                state = laserincrement;
                
                // change TMR1 period
                PR1 = short_time;
                
                break;
            //------------------------------------------------------------------
                
            //---laserincrement state-------------------------------------------
            case laserincrement:
                
                // increase servo angle
                servo_start = servo_start - servo_increment;
                
                // change OC3R duty cycle
                OC3R = servo_start;
                
                // change wait
                wait = true;
                
                // reset TMR1
                TMR1 = 0;
                
                // change state to laserdelay
                state = laserdelay;
                
                break;
            //------------------------------------------------------------------
                
            //---laserdelay-----------------------------------------------------
            case laserdelay:
                
                // check TMR1
                if (wait == false){
                    
                    // change state to firelaser
                    state = firelaser;
                    
                }
                break;
            //------------------------------------------------------------------
                
            //---firelaser state------------------------------------------------
            case firelaser:
                
                if (ADC1BUF11 > diode_thresh){
                    
                    servo_start = servo_start = final_increment;
                    
                    OC3R = servo_start;
                    
                    // turn laser on
                    _LATB7 = 1;
                    
                    // change state to missioncomplete
                    state = missioncomplete;
                    
                }
                
                else{
                    
                    // change state to laser increment
                    state = laserincrement;
                    
                }
                
                break;
            //------------------------------------------------------------------
                
            //---missioncomplete state------------------------------------------
            case missioncomplete:
                
                // maintain laser position
                OC3R = servo_start;
                
                break;
            //------------------------------------------------------------------
            
        }
        
    }
    
return 0;

}
//------------------------------------------------------------------------------
