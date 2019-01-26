/* ==========================================================================
**  TwoPropMixTiny V1.0 
**
**  Core: ATtiny841 / Dr.Azzy (http://drazzy.com/e/tiny841.shtml9)
**
** Setup:
** ------
** 1) Motor stick in max up-position and switch on -> wait for long LED blink
** 2) Motor stick in center-position -> wait for LED blink
** 3) Motor stick in up-position for RC Switch input or down-position for Switch input via Multiswitch -> wait for double LED blink
** 4) Motor stick in center-position -> wait for LED blink
** 5) Motor stick in up-position for Enhanced mode on or down-position for Enhanced mode off -> wait for tripple LED blink
** 6) Motor stick in center-position -> wait for LED blink
** 7) Motor stick in up-position for Reverse mode or down-position for Normal mode -> wait for 4 times LED blink
** 8) Motor stick in center-position -> wait for long LED blink
** Done !
**
** Calibration: 
** ------------
** 1) Motor stick in max down-position and switch on -> wait for LED permanent blinking (1min)
** 2) Put all sticks to their maximum and minimum positions one after another until LED blinking stops
** Done !
**
**  D3 : QSR Output
**  D4 : Motor right Output
**  D6 : Rudder Output
**  D7 : Motor left Output
**  D5 : LED Output
**
**  D0 : RC Input for Switch
**  D8 : RC Input for QSR
**  D9 : RC Input for Motor
**  D10: RC Input for Rudder
**
**  A10: Poti Delay
**  A08: Poti Mix
**
**  EEPROM address:
**  00 - reserved for calibrating value OSCAL
**  01 - 1 for RC-Switch / 0 for digital Switch Input
**  02 - 1 for enhanced Mode on / 0 for off
**  03 - 1 for Reverse / 0 Normal
**  04 - 1 for calibration done / 0 for uncalibrated
**  10 - max value for RCrudder
**  12 - min value for RCrudder
**  14 - max value for RCmotor
**  16 - min value for RCmotor
**  18 - max value for RCqsr
**  20 - min value for RCqsr
**
** Author: Marco Stoffers
** Year: 2016
** License: CC (BY|NC|SA)
** -------------------------------------------------------------------------*/
#include <EEPROM.h>
// -----------------------------
// defines
// -----------------------------
#define RUDDER_PIN 6 
#define MOTORL_PIN 7
#define MOTORR_PIN 4
#define QSR_PIN 3

#define RC_RUDDER_PIN 10
#define RC_MOTOR_PIN 9
#define RC_QSR_PIN 8
#define RC_SWITCH_PIN 0

#define LED_PIN 5

#define POTI1_PIN A10
#define POTI2_PIN A8

#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
// -----------------------------
// variables
// -----------------------------
// the helper variables
uint8_t i = 0;
uint8_t j = 0;
// First the Array of all pins
volatile uint8_t pin_array[11] { RUDDER_PIN, MOTORL_PIN, MOTORR_PIN, QSR_PIN, LED_PIN, RC_RUDDER_PIN, RC_MOTOR_PIN, RC_QSR_PIN, RC_SWITCH_PIN, POTI1_PIN, POTI2_PIN };
// 2 arrays with times for the RC Measurement and one for Measurement done or not
volatile unsigned long start_micros[4]; // Rudder, Motor, QSR, Switch
volatile unsigned long stop_micros[4];
volatile boolean measurement[4] { false, false, false, false };
// temporary stored rc values for building the average over 3 times
volatile uint16_t rc_temp[4];    // Rudder, Motor, QSR, Switch
volatile uint16_t rc_value[4] {0, 0, 0, 0};
volatile uint8_t rc_avg_count[4];
// array for storing the rc signal quality (ok/bad)
volatile boolean rc_ok[4] {false, false, false, false};    // Rudder, Motor, QSR, Switch
volatile boolean rc_switch_enable;
// the middle, max and min of the rc signals as an array (will be overwritten by reading the eeprom)
uint16_t rc_middle[3] {1500, 1500, 1500};
uint16_t rc_max[3] { 1500, 1500, 1500 };    // Rudder, Motor, QSR
uint16_t rc_min[3] { 1500, 1500, 1500 };    // Rudder, Motor, QSR
// set the output array so there are all ESC on neutral
uint16_t output_value[3] {1500, 1500, 1500};
// different variables for calculating the mix level
boolean mix = true;
uint16_t mix_value = 0;
uint16_t delay_value = 0;
uint16_t helper = 0;
boolean switch_value = false;
boolean enhanced_mode = false;
uint16_t rc_enhanced_value;
uint8_t loop_count = 0;
boolean turn_table = true;
boolean reverse = false;
uint16_t calib_temp;
uint16_t qsr_temp = 0;

// -----------------------------
// Setup
// -----------------------------
void setup() {
    for(i = 0; i < 5; i++) {                    // set the first 4 Pins in the array
        pinMode(pin_array[i], OUTPUT);          // to outputs
        digitalWrite(pin_array[i], LOW);        // and set them low / off (Servo Pins)
    }
    for(i = 5; i < 11; i++) {                   // set the next 6 Pins in the array
        pinMode(pin_array[i], INPUT);           // to inputs
    }

    start_rc_input();                           // start RC input via PinChange Interrupt
    delay(500);
    while(!rc_ok[0] && !rc_ok[1] && !rc_ok[2]) {  }            // ... wait until RC-signals are stable
    delay(500);
    if(rc_value[1] > 1700) do_setup();          // if Motor stick is in in position 'forward' go to setup
    if(rc_value[1] < 1300) do_calibrate();      // if Motor stick in in in position 'backward' go to calibration
    
    if(EEPROM.read(3) > 0) {                    // if module is already calibrated
        for(i=0; i<3; i++) {
            rc_max[i]=eeprom_read(10+(i*4));    // load values from eeprom
            rc_min[i]=eeprom_read(12+(i*4));
        }
    }
    else {                                      // if not ...
        for(i=0; i<3; i++) {
            rc_max[i] = 1900;                   // store default values
            rc_min[i] = 1100;
        }
    }
    
    if(EEPROM.read(1) > 0) {                    // read EEPROM and check if RC switch is stored as enabled
      rc_switch_enable = true;
      start_rc_switch_input();                  // and switch on Interrupt for Switch RC
    }
    else {                                      // otherwise set PIN to Input with Pullup Resistor enabled
      rc_switch_enable = false;
      pinMode(RC_SWITCH_PIN, INPUT_PULLUP); 
    }

    if(EEPROM.read(2) > 0) enhanced_mode = true;  // read EEPROM and check if Enhanced Mode is stored as enabled
    else enhanced_mode = false;                 // otherwise disable Enhanced Mode

    learn_rc_middle();                          // now learn middle position of Rudder & Motor stick
        
    start_servos();                             // ... start servos
    output();                                   // ... give first incomming positions to servos

    if(EEPROM.read(4) == 1) reverse = true;     // if module is setup for reverse set variable
    else reverse = false;

    rc_enhanced_value = (rc_middle[1] - rc_min[1]) / 2;   // 50% reverse at enhanced mode

    led_on();                                   // ... and switch LED on to show valid RC signals and Servo output    
}

// -----------------------------
// Main loop
// -----------------------------
void loop() {

    mix_value = map(analogRead(POTI1_PIN),0,1024,0,(rc_max[1] - rc_middle[1]));                 // get values from the potis and map it into rc range
    delay_value = map(analogRead(POTI2_PIN),0,1024,0,(rc_max[0] - rc_middle[0]));
    if(delay_value > (rc_max[0] - rc_middle[0] - 20)) mix = false;                              // if delay poti is set to max, disable two prop mixing during drive
    else mix = true;
    if(!rc_switch_enable) switch_value = !digitalRead(RC_SWITCH_PIN);                           // if switch input is set to rc read it otherwise read digital input
    else {
      if(rc_value[3] > 1700) switch_value = true;                                               
      else switch_value = false;
    }
    
    //
    // Calculate Mixing Level
    //
    //     forward
    //    ---------
    if(rc_ok[0] && rc_ok[1] && (rc_value[1] > (rc_middle[1]+20))) {
        //
        // Rudder mix only if given value (poti "delay") is below max
        //    left
        //   -------
        if(rc_value[0] > (rc_middle[0]+delay_value)) {
            
            if(mix && !enhanced_mode) helper = map(rc_value[0],(rc_middle[0]+delay_value),rc_max[0],0,(rc_max[1] - rc_middle[1]));
            else if(mix && enhanced_mode) helper = map(rc_value[0],(rc_middle[0]+delay_value),rc_max[0],0,(rc_max[1] - rc_middle[1] + rc_enhanced_value));
            else helper = rc_value[1];
            if(reverse) {
                output_value[0] = rc_value[1];
                output_value[1] = rc_value[1] - helper;
                if(!enhanced_mode && (output_value[1] < rc_middle[1])) output_value[1] = rc_middle[1];
                if(enhanced_mode && (output_value[1] < (rc_middle[1] - rc_enhanced_value))) output_value[1] = rc_middle[1] - rc_enhanced_value;
            }
            else {
                output_value[1] = rc_value[1];
                output_value[0] = rc_value[1] - helper;
                if(!enhanced_mode && (output_value[0] < rc_middle[1])) output_value[0] = rc_middle[1];
                if(enhanced_mode && (output_value[0] < (rc_middle[1] - rc_enhanced_value))) output_value[0] = rc_middle[1] - rc_enhanced_value;
            }
            turn_table = false;
        }
        else {
          if(reverse) output_value[1] = rc_value[1];
          else output_value[0] = rc_value[1];
        }
        //
        //    right
        //   ------
        if(rc_value[0] < (rc_middle[0]-delay_value)) {
            if(mix && !enhanced_mode) helper = map(rc_value[0],(rc_middle[0]-delay_value),rc_min[0],0,(rc_max[1] - rc_middle[1]));
            else if(mix && enhanced_mode) helper = map(rc_value[0],(rc_middle[0]-delay_value),rc_min[0],0,(rc_max[1] - rc_middle[1] + rc_enhanced_value));
            else helper = rc_value[1];
            if(reverse) {
                output_value[0] = rc_value[1] - helper;
                output_value[1] = rc_value[1];
                if(!enhanced_mode && (output_value[0] < rc_middle[1])) output_value[0] = rc_middle[1];
                if(enhanced_mode && (output_value[0] < (rc_middle[1] - rc_enhanced_value))) output_value[0] = rc_middle[1] - rc_enhanced_value;
            }
            else {
                output_value[1] = rc_value[1] - helper;
                output_value[0] = rc_value[1];
                if(!enhanced_mode && (output_value[1] < rc_middle[1])) output_value[1] = rc_middle[1];
                if(enhanced_mode && (output_value[1] < (rc_middle[1] - rc_enhanced_value))) output_value[1] = rc_middle[1] - rc_enhanced_value;
            }
            turn_table = false;
        }
        else {
          if(reverse) output_value[0] = rc_value[1];
          else output_value[1] = rc_value[1];
        }
        //
        // QSR mix only when switch is on and speed is below given value (poti "mix")
        //
        if(switch_value && rc_ok[2] && (rc_value[1] < (rc_middle[1] + mix_value))) {
            //
            //     right
            //    -------
            if(rc_value[0] > (rc_middle[0]+20)) {
                if(reverse) qsr_temp = rc_middle[2] - (rc_value[0] - rc_middle[0]);
                else qsr_temp = rc_middle[2] + (rc_value[0] - rc_middle[0]);

                if(output_value[2] > qsr_temp) output_value[2]--;
                if(output_value[2] < qsr_temp) output_value[2]++;
            }
            //
            //     left
            //    ------
            else if(rc_value[0] < (rc_middle[0]-20)) {
                if(reverse) qsr_temp = rc_middle[2] + (rc_middle[0] - rc_value[0]);
                else qsr_temp = rc_middle[2] - (rc_middle[0] - rc_value[0]);

                if(output_value[2] > qsr_temp) output_value[2]--;
                if(output_value[2] < qsr_temp) output_value[2]++;
            }
            else {
              if(output_value[2] > rc_middle[2]) output_value[2]--;
              if(output_value[2] < rc_middle[2]) output_value[2]++;
            }
        }
        else {
          if(output_value[2] > rc_middle[2]) output_value[2]--;
          if(output_value[2] < rc_middle[2]) output_value[2]++;
        }
    }
    //
    //     reverse
    //    ---------
    else if(rc_ok[1] && (rc_value[1] < (rc_middle[1]-20))) {
        output_value[1] = rc_value[1];
        output_value[0] = rc_value[1];
        //
        // QSR mix only when switch is on and speed is below given value (poti "mix")
        //
        if(switch_value && rc_ok[2] && ((rc_middle[1] - mix_value) < rc_value[1])) {
            //
            //     right (turn QSR left for reverse)
            //    -----------------------------------
            if(rc_value[0] > (rc_middle[0]+20)) {
                if(reverse) qsr_temp = rc_middle[2] - (rc_middle[0] - rc_value[0]);
                else qsr_temp = rc_middle[2] + (rc_middle[0] - rc_value[0]);

                if(output_value[2] > qsr_temp) output_value[2]--;
                if(output_value[2] < qsr_temp) output_value[2]++;
            }
            //
            //     left (turn QSR right for reverse)
            //    -----------------------------------
            else if(rc_value[0] < (rc_middle[0]-20)) {
                if(reverse) qsr_temp = rc_middle[2] + (rc_value[0] - rc_middle[0]);
                else qsr_temp = rc_middle[2] - (rc_value[0] - rc_middle[0]);

                if(output_value[2] > qsr_temp) output_value[2]--;
                if(output_value[2] < qsr_temp) output_value[2]++;
            }
            else {
              if(output_value[2] > rc_middle[2]) output_value[2]--;
              if(output_value[2] < rc_middle[2]) output_value[2]++;
            }
        }
        else {
          if(output_value[2] > rc_middle[2]) output_value[2]--;
          if(output_value[2] < rc_middle[2]) output_value[2]++;
        }
    }
    else {
        //
        // "Turn on table" if boat is standing still
        //
        //    right
        //   -------
        if(turn_table && (rc_value[0] > (rc_middle[0] + 20)) && between(rc_value[1], rc_middle[1] - 20, rc_middle[1] + 20)) {
            if(reverse) {
                output_value[0] = rc_middle[1] + (rc_value[0] - rc_middle[0]);
                output_value[1] = rc_middle[1] - (rc_value[0] - rc_middle[0]);
                if(switch_value) output_value[2] = rc_middle[2] - (rc_value[0] - rc_middle[0]);
            }
            else {
                output_value[0] = rc_middle[1] - (rc_value[0] - rc_middle[0]);
                output_value[1] = rc_middle[1] + (rc_value[0] - rc_middle[0]);
                if(switch_value) output_value[2] = rc_middle[2] + (rc_value[0] - rc_middle[0]);
            }
            
        }

        //
        //    left
        //   ------
        else if(turn_table && (rc_value[0] < (rc_middle[0] - 20)) && between(rc_value[1], rc_middle[1] - 20, rc_middle[1] + 20)) {
            if(reverse) {
                output_value[0] = rc_middle[1] - (rc_middle[0] - rc_value[0]);
                output_value[1] = rc_middle[1] + (rc_middle[0] - rc_value[0]);
                if(switch_value) output_value[2] = rc_middle[2] + (rc_middle[0] - rc_value[0]);
            }
            else {
                output_value[0] = rc_middle[1] + (rc_middle[0] - rc_value[0]);
                output_value[1] = rc_middle[1] - (rc_middle[0] - rc_value[0]);
                if(switch_value) output_value[2] = rc_middle[2] - (rc_middle[0] - rc_value[0]);
            }
            
        }

        else {
            output_value[0] = rc_middle[1];
            output_value[1] = rc_middle[1];  
            output_value[2] = rc_middle[2]; 
            if(between(rc_value[0],rc_middle[0] - 20, rc_middle[0] +20)) turn_table = true; 
        }
    }
    //
    // QSR normal (will follow RC input QSR) when switch is off
    //
    if(!switch_value) output_value[2] = rc_value[2];
        
    //
    // and give everything to the ouput pins
    //
    output();
}

// -----------------------------
// Switch LED on
// -----------------------------
void led_on() {
    digitalWrite(LED_PIN, HIGH);
}

// -----------------------------
// Switch LED off
// -----------------------------
void led_off() {
    digitalWrite(LED_PIN, LOW);
}

// -----------------------------
// Blink LED short
// -----------------------------
void blink_led_short() {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
}

// -----------------------------
// Blink LED long
// -----------------------------
void blink_led_long() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}

// -----------------------------
// Learn Middle for Rudder and Motor
// -----------------------------
void learn_rc_middle() {
    rc_middle[0] = rc_value[0];
    rc_middle[1] = rc_value[1];     
    rc_middle[2] = rc_value[2];
}

// -----------------------------
// Setup Routine
// -----------------------------
void do_setup() {
    delay(500);                         // wait 0,5s to avoid interferences from the FS
    blink_led_long();
    while(!(rc_value[1] < 1700)) { }    // wait until the RC signal gets shorter than 1700ms (stick goes to middle)
    delay(500);                         // wait so that the stick is in middle postion
    blink_led_short();
    delay(500);
    while(!((rc_value[1] < 1300)||(rc_value[1] > 1700))) { }  // wait until the stick will be moved up or down
    delay(300);
    if(rc_value[1] < 1300) {            // if down set Switch Port to normal (no RC signal)
      rc_switch_enable = false;
      EEPROM.write(1, 0);               // and store setting in eeprom
    }
    else { 
      rc_switch_enable = true;          // if up set Switch Port to RC input 
      EEPROM.write(1, 1);               // and store setting in eeprom
    }
    delay(300);
    blink_led_short();                  // two blinks as an "ok"
    blink_led_short();
    delay(300);
    while(!between(rc_value[1],1300,1700)) { }    // again wait for the stick to become middle
    blink_led_short();
    delay(500);
    while(!((rc_value[1] < 1300)||(rc_value[1] > 1700))) { }  // and wait for up or down movement of the stick
    delay(300);
    if(rc_value[1] < 1300) {            // if down disable Enhanced mode
      enhanced_mode = false;
      EEPROM.write(2, 0);               // write setting to eeprom
    }
    else { 
      enhanced_mode = true;             // if up enable Enhanced mode
      EEPROM.write(2, 1);               // write setting to eeprom
    }
    delay(300);
    blink_led_short();                  // three blinks for an "ok"
    blink_led_short();
    blink_led_short();
    while(!between(rc_value[1],1300,1700)) { }  // and as always: wait for stick to become middle
    blink_led_short();
    delay(500);
    while(!((rc_value[1] < 1300)||(rc_value[1] > 1700))) { }    // and wait for up or down movement
    delay(300);
    if(rc_value[1] < 1300) {            // down sets mixing to "normal"
      reverse = false;
      EEPROM.write(4, 0);               // write setting to eeprom
    }
    else { 
      reverse = true;                   // up sets mixing to "reverse"
      EEPROM.write(4, 1);               // write setting to eeprom
    }
    delay(300);
    blink_led_short();                  // four blinks for an "ok" here
    blink_led_short();
    blink_led_short();
    blink_led_short();
    while(!between(rc_value[1],1300,1700)) { }    // and last: wait for stick to set to middle
    blink_led_long();                             // blink once and end setup
    delay(300);
}

// -----------------------------
// Calibration Routine
// -----------------------------
void do_calibrate() {
    delay(500);                                                 // calibration is an timed setup (nearly 10s)
    loop_count = 0;                                             
    while(loop_count < 35) {                                    // count the loops to time the calibration
        blink_led_short();                                      // blink led short on any loop to show calibration is still progress
        for(i=0; i<3; i++) {                                    // go on each rc signal
          calib_temp = rc_value[i];                             // store the actual rc value temporaly
          if(calib_temp > rc_max[i]) rc_max[i] = calib_temp;    // and check if the given rc value (from the receiver) is more or less of the already stored one ...
          if(calib_temp < rc_min[i]) rc_min[i] = calib_temp;    // ... if so store the new one in the array
        }
        loop_count++;
    }
    for(i=0; i<3; i++) {                                        // store the new rc values for max and min in the eeprom cells
        eeprom_write((10+(i*4)),rc_max[i]);
        eeprom_write((12+(i*4)),rc_min[i]);
    }
    EEPROM.write(3, 1);                                         // and write a "one" to the eeprom to show "yes, calibrated"
}

// -----------------------------
// Subroutine: Read EEPROM 16bit
// -----------------------------
uint16_t eeprom_read(uint8_t addr) {                            // simple routine to read 2 8bit eeprom cells into the needed 16bit value
    uint16_t var = ((EEPROM.read(addr+1) << 8 ) | EEPROM.read(addr));
    return var;
}

// -----------------------------
// Subroutine: Write EEPROM 16bit
// -----------------------------
void eeprom_write(uint8_t addr, uint16_t var) {                 // simple routine to store a 16bit value in 2 8bit eeprom cells
    uint8_t help = var;
    EEPROM.write(addr, help);
    help = (var >> 8);
    EEPROM.write(addr+1, help);
}

// -----------------------------
// Output
// -----------------------------
void output() {
    if(rc_ok[0]) OCR1A = rc_value[0];                           // load the OC register (output compare register) with the latest rc signals for output ...
    else OCR1A = rc_middle[0];                                  // ... only if the incomming rc signals are still ok
    if(rc_ok[1]) {
        OCR1B = output_value[0];
        OCR2A = output_value[1];
    }
    if(rc_ok[2]) OCR2B = output_value[2];
}

// -----------------------------
// Subroutine: Start Servo Output
// -----------------------------
void start_servos() {
    //Timer 1
    ICR1 = 20000;                       // max count of timer1 will be 20000 eq. 20ms (main loop frame for servos)
    TCCR1A = 0;                         // clear timer1 control register 1 because of Arduino IDE preset
    bitSet(TCCR1A,COM1A1);              // set the output compare mode to high at timer1=0 and low on compare match for channel A
    bitSet(TCCR1A,COM1B1);              // and also for channel B
    bitSet(TCCR1A,WGM11);               // set PWM mode to 14 (FastPWM with Top ICR1)
    TCCR1B = 0;                         // clear timer1 control register 2 because of Arduino IDE preset
    bitSet(TCCR1B,WGM12);               // set PWM mode to 14 (FastPWM with Top ICR1)
    bitSet(TCCR1B,WGM13);               // set PWM mode to 14 (FastPWM with Top ICR1)
    bitSet(TCCR1B,CS11);                // set Prescaler of timer1 to 8 so timer1 runs with 1MHz / 0.001ms
    TOCPMSA0 = 0;                       // clear timer Output Mixer Register 1 because of Arduino IDE preset
    TOCPMSA1 = 0;                       // clear timer Output Mixer Register 2 because of Arduino IDE preset
    TOCPMCOE = 0;                       // clear timer Output enable Register because of Arduino IDE preset
    bitSet(TOCPMSA0, TOCC3S0);          // route OCR1A Output to TOCC3
    bitSet(TOCPMCOE, TOCC3OE);          // enable TOCC3
    bitSet(TOCPMSA0, TOCC2S0);          // route OCR1B Output to TOCC4
    bitSet(TOCPMCOE, TOCC2OE);          // enable OCC4
    TCNT1 = 0;                          // clear timer1
    OCR1A = 1500;                       // set Compare match A to 1500 eq 1,5ms (middle position of servos)
    OCR1B = 1500;                       // set Compare match A to 1500 eq 1,5ms (middle position of servos)

    //Timer 2
    ICR2 = 20000;                       // max count of timer2 will be 20000 eq. 20ms (main loop frame for servos)
    TCCR2A = 0;                         // clear timer2 control register 1 because of Arduino IDE preset
    bitSet(TCCR2A,COM2A1);              // set the output compare mode to high at timer2=0 and low on compare match for channel A
    bitSet(TCCR2A,COM2B1);              // and also for channel B
    bitSet(TCCR2A,WGM11);               // set PWM mode to 14 (FastPWM with Top ICR2)
    TCCR2B = 0;                         // clear timer2 control register 2 because of Arduino IDE preset
    bitSet(TCCR2B,WGM12);               // set PWM mode to 14 (FastPWM with Top ICR2)
    bitSet(TCCR2B,WGM13);               // set PWM mode to 14 (FastPWM with Top ICR2)
    bitSet(TCCR2B,CS11);                // set Prescaler of timer2 to 8 so timer2 runs with 1MHz / 0.001ms
    bitSet(TOCPMSA1, TOCC5S1);          // route OCR2A Output to TOCC5
    bitSet(TOCPMCOE, TOCC5OE);          // enable TOCC5
    bitSet(TOCPMSA1, TOCC6S1);          // route OCR2B Output to TOCC6
    bitSet(TOCPMCOE, TOCC6OE);          // enable TOCC6
    TCNT2 = 0;                          // clear timer2
    OCR2A = 1500;                       // set Compare match A to 1500 eq 1,5ms (middle position of servos)
    OCR2B = 1500;                       // set Compare match B to 1500 eq 1,5ms (middle position of servos)
}

// -----------------------------
// Subroutine: Start RC Input
// -----------------------------
void start_rc_input() {
    bitSet(GIMSK, PCIE0);               // enable PinChange Interrupt on PCINT0 - 7
    bitSet(PCMSK0, PCINT0);             // allow PCINT0
    bitSet(PCMSK0, PCINT1);             // allow PCINT1
    bitSet(PCMSK0, PCINT2);             // allow PCINT2
}

// -----------------------------
// Subroutine: Start RC Switch Input
// -----------------------------
void start_rc_switch_input() {
    bitSet(GIMSK, PCIE1);               // enable PinChange Interrupt on PCINT8 - 11
    bitSet(PCMSK1, PCINT8);             // allow PCINT8
}

// -----------------------------
// Interrupt0 for RC Input
// -----------------------------
ISR (PCINT0_vect) {
    for(uint8_t u = 0; u < 3; u++) {
        if(!digitalRead(pin_array[u+5]) && !measurement[u]) {         // if the PinChange Interrupt was triggered by corresponding pin and the old values where already calculated
          start_micros[u] = micros();                                 // remember the actual micros
          measurement[u] = true;                                      // and mark the measurement as active
        }
        if(digitalRead(pin_array[u+5]) && measurement[u]) {           // if the PinChange Interrupt was again triggered by corresponding pin and the measurement is active
          stop_micros[u] = micros();                                  // remeber the new micros position
          measurement[u] = false;                                     // set the measurement to inactive
          
          if(start_micros[u] < stop_micros[u]) {                      // only calculate the rc value if the beginning time is smaller then the ended time of measurement
            rc_temp[u] += (stop_micros[u] - start_micros[u]);         // store all calculated values in the rc_temp array
            rc_avg_count[u]++;                                        // and increase the average counter
          }
          if(rc_avg_count[u] >2) {                                    // if the average counter is 3
            rc_value[u] = rc_temp[u] / 3;                             // build the average of the rc value and store it in the rc_value array
            rc_avg_count[u] = 0;                                      // don't forget to clear the average counter
            rc_temp[u] = 0;                                           // uups, and also the temp value ...
          }
          // check if value is correct
          if(between(rc_value[u], 900, 2100)) rc_ok[u] = true;        // if the new calculated value is between 900ms and 2100ms
          else rc_ok[u] = false;                                      // set rc_ok to "one"
        } 
    }
}

// -----------------------------
// Interrupt1 for RC Input (rc_switch)
// -----------------------------
ISR (PCINT1_vect) {
  
    // Switch RC Input
    if(!digitalRead(RC_SWITCH_PIN) && !measurement[3]) {              // if the PinChange Interrupt was triggered by corresponding pin and the old values where already calculated
        start_micros[3] = micros();                                   // remember the actual micros
        measurement[3] = true;                                        // and mark the measurement as active
    }
    if(digitalRead(RC_SWITCH_PIN) && measurement[3]) {                // if the PinChange Interrupt was again triggered by corresponding pin and the measurement is active
        stop_micros[3] = micros();                                    // remeber the new micros position
        measurement[3] = false;                                       // set the measurement to inactive
        
        if(start_micros[3] < stop_micros[3]) {                        // only calculate the rc value if the beginning time is smaller then the ended time of measurement
            rc_temp[3] += (stop_micros[3] - start_micros[3]);         // store all calculated values in the rc_temp array
            rc_avg_count[3]++;                                        // and increase the average counter
          }
          if(rc_avg_count[3] >2) {                                    // if the average counter is 3
            rc_value[3] = rc_temp[3] / 3;                             // build the average of the rc value and store it in the rc_value array
            rc_avg_count[3] = 0;                                      // don't forget to clear the average counter
            rc_temp[3] = 0;                                           // and also the temp value ...
          }
          // check if value is correct
          if(between(rc_value[3], 900, 2100)) rc_ok[3] = true;        // if the new calculated value is between 900ms and 2100ms
          else rc_ok[3] = false;                                      // set rc_ok to "one"
    }
}
