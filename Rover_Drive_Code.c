#include "xc.h"

// CONFIGURATION
#pragma config FNOSC = LPFRC
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

// ====== GLOBALS ======
int norm_speed = 750;
int turn_speed = 750;
int steps = 0;
int step_thresh = 145;
int back_step_thresh = 150;

// ====== INTERRUPT ======
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void)
{
_OC1IF = 0;
steps++;
}

// ====== MOVEMENT FUNCTIONS ======
void stop(void) {
_LATA0 = 0; // right dir
_LATA1 = 0; // left dir
OC1RS = 0; OC1R = OC1RS / 2;
OC2RS = 0; OC2R = OC2RS / 2;
}
void drive_straight(void) {
_LATA0 = 0; // right dir
_LATA1 = 0; // left dir
OC1RS = norm_speed; OC1R = OC1RS / 2;
OC2RS = norm_speed; OC2R = OC2RS / 2;
}
void drive_back(void) {
_LATA0 = 1; // right dir
_LATA1 = 1; // left dir
OC1RS = norm_speed; OC1R = OC1RS / 2;
OC2RS = norm_speed; OC2R = OC2RS / 2;
}
void turn_left(void) {
_LATA0 = 0; // right motor backward
_LATA1 = 1; // left motor forward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;
}

void turn_right(void) {
_LATA0 = 1; // right motor forward
_LATA1 = 0; // left motor backward
OC1RS = turn_speed; OC1R = OC1RS / 2;
OC2RS = turn_speed; OC2R = OC2RS / 2;
}

// ====== CONFIGURATION ======
void config_pins(void) {
_ANSA0 = 0; // digital
_ANSA1 = 0;
_ANSA2 = 0;
_ANSB15 = 0;
_ANSB14 = 0;
_ANSB13 = 0;
_TRISA0 = 0; // right dir
_TRISA1 = 0; // left dir
_TRISB13 = 1; // right IR
_TRISB14 = 1; // front IR
_TRISB15 = 1; // left IR

//_ANSB8 = 0;
_TRISB8 = 0;
}

void config_pwm(void) {
OC1CON1 = 0x1C06;
OC1CON2 = 0x001F;
OC2CON1 = 0x1C06;
OC2CON2 = 0x001F;
}

// ====== MAIN ======
int main(void) {
enum { STOP, STRAIGHT, TURN_LEFT, TURN_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT } state;
config_pins();
config_pwm();
state = STOP;

while (1) {
    int front = _RB14; //if it doesn't see a wall it will be high
    int left = _RB15;
    int right = _RB13;

    switch (state){
        case STOP:
            stop();
            if (left == 0) {
            steps = 0;
            _OC1IE = 1;
            state = STRAIGHT;
            _LATB8 = 0;

            }else {
                _LATB8 =1;
            }
            break;

        case STRAIGHT:
           
            drive_straight();
            if (!front && !right) {
            steps = 0;
            state = DRIVE_BACK_LEFT;
            } else if (!front && !left) {
            steps = 0;
            state = DRIVE_BACK_RIGHT;
            }
            break;

        case TURN_LEFT:
           
            turn_left();
            if (steps > step_thresh) {
            steps = 0;
            state = STRAIGHT;
           
            }
            break;

        case TURN_RIGHT:
            turn_right();
            if (steps > step_thresh) {
            steps = 0;
            state = STRAIGHT;
              _LATB8 = 0;

            }else {
                _LATB8 =1;
           
            }
            break;
        case DRIVE_BACK_RIGHT:
            drive_back();
            if (steps > back_step_thresh) {
            steps = 0;
            state = TURN_RIGHT;
                    }
            break;
        case DRIVE_BACK_LEFT:
            drive_back();
            if (steps > back_step_thresh) {
            steps = 0;
            state = TURN_LEFT;
                    }
            break;

    }  
}
return 0;

}
