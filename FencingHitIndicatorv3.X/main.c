
/*
 * Fencing Hit Indicator v3
 * Copyright (c) 2024 David Rice
 * 
 * github.com/davidsmakerworks
 * 
 * This is a simple fencing hit indicator that implements the hit time and
 * lockout time as specified by current foil fencing rules. Score display is also
 * implemented.
 * 
 * Designed for PIC 16F18855 and STP16CPC26 LED drivers
 * 
 * INPUTS:
 * RA0 - Red fencer B-line
 * RA3 - Green fencer B-line
 * 
 * RA1 - Red fencer score increment
 * RA2 - Green fencer score increment 
 * 
 * OUTPUTS:
 * RB0 - PWM buzzer
 * 
 * RC2 - Red LED display driver latch enable
 * RC3 - Green LED display driver latch enable
 * RC1 - LED display driver data (SSP1DAT)
 * RC0 - LED display driver clock (SSP1CLK)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18855 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Pins where the fencers' foils are connected
#define RED_FOIL PORTAbits.RA0
#define GREEN_FOIL PORTAbits.RA3

// Pins where LED display driver latch enables are connected
#define RED_SCORE_LATCH LATCbits.LATC2
#define GREEN_SCORE_LATCH LATCbits.LATC3

// Pins where score increment signals are connected
#define RED_SCORE_INCREMENT !PORTAbits.RA1
#define GREEN_SCORE_INCREMENT !PORTAbits.RA2

// State values for finite state machine:
//
// STATE_READY: Monitoring for hits by one or both fencers
// STATE_LOCKOUT: Hit has been recorded and lockout time expired
// STATE_RESET: Resetting to prepare for next hit
#define STATE_READY 0
#define STATE_LOCKOUT 1
#define STATE_RESET 2

// Per USA Fencing 2023 rules, the tip of the foil must be depressed for 14 ms
// to register a touch. 300 ms after the first touch is registered, the other
// fencer is no longer able to register a touch.
#define MIN_HIT_TIME 14 
#define LOCKOUT_TIME 300

// Amount of time in msec that buzzer should sound when hit is registered
#define BUZZER_TIME 1500

// Amount of additional time in msec that LEDs should stay on after buzzer has stopped sounding
#define ADDL_LIGHT_TIME 1500

// If a hit is detected within this many ms after the last hit, count it as possibly being due to a disconnected foil
#define DISCONNECT_DETECT_TIME 500

// If this many "hits" are detected in rapid succession, assume that one or both foils are disconnected
// Minimum is 2
#define DISCONNECT_DETECT_COUNT 3

// Amount of time in ms that a score increment signal must be active to increment the score
#define SCORE_INCREMENT_TIME 30

// Time to sound buzzer when score is incremented
#define SCORE_INC_BUZZER_TIME 100

// Minimum time in msec between score increments
#define SCORE_COOLDOWN_TIME 2000

// Score limit when score will reset
#define SCORE_LIMIT 5

// Delay before resetting score in msec after score limit is reached
#define SCORE_RESET_TIME 1000

// Length of score reset signal beeps in msec
#define SCORE_RESET_SIGNAL_TIME 100

// Bit pattern to illuminate hit indicator segments on score display
#define HIT_INDICATOR_ON 0b00111111
#define HIT_INDICATOR_OFF 0b00000000

// Macros for which display is active
#define RED_DISPLAY 0
#define GREEN_DISPLAY 1

// Standard period for hit indication tone (about 2000 Hz)
#define BUZZER_PERIOD 16

// The time stamps (in ms) at which the fencer's hit was first detected
// This condition must persist for MIN_HIT_TIME ms to be considered a valid hit
uint16_t red_start_timestamp;
uint16_t green_start_timestamp;

// The time stamps (in ms) when a score increment signal was detected
uint16_t red_inc_start_timestamp;
uint16_t green_inc_start_timestamp;

// The time stamp (in ms) when the first hit was registered
// No hits can be registered LOCKOUT_TIME ms after the first hit
uint16_t lockout_start_timestamp;

// The time stamp (in ms) when the state was last reset. This is used
// to determine if the buzzer has been sounding continuously due to one
// or both foils being disconnected and disable it until a valid hit
// is recorded
uint16_t last_reset_timestamp;

// The timestamp when the READY state was entered, used to determine when
// a disconnect condition has been cleared
uint16_t ready_timestamp;

// This is the number of times that a "hit" has been detected in rapid succession. If
// this is larger than DISCONNECT_DETECT_COUNT, it means one or both foils are probably
// disconnected.
uint8_t consecutive_activations;

// true if buzzer is armed (i.e., both foils are connected and hits are
// valid). false if the buzzer has been sounding continuously due to one
// or both foils being disconnected.
bool buzzer_armed;

// State variable for finite state machine in main loop
uint8_t state;

// Indicates if the tip is currently depressed (i.e., a hit will be registered
// if the tip remains depressed for MIN_HIT_TIME ms) 
bool red_pending;
bool green_pending;

// Indicates that a hit has been registered by one fencer, and now the other fencer
// has LOCKOUT_TIME ms to also register a hit
bool lockout_pending;

// Indicates that a score increment is pending - used to avoid false detections
bool red_inc_pending;
bool green_inc_pending;

// Timestamp when last score increment occurred
uint16_t score_cooldown_timestamp;

// Flag to indicate of score cooldown is active
bool score_cooldown_active;

// 7-segment digit bits in reverse order (i.e., .GFEDCBA) for MAX6977 LED driver
const uint8_t digits[10] = {
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01100111
};

volatile uint16_t ticks = 0; // 1 tick = 1 msec

void __interrupt() isr(void) {
    if (INTCONbits.PEIE) {
        if (PIE4bits.TMR2IE && PIR4bits.TMR2IF) {
            PIR4bits.TMR2IF = 0; // Interrupt flag must be cleared manually
            ticks++; // Count 1 msec
        }
    }
}

uint16_t get_ticks(void) {
    uint16_t t;
    
    INTCONbits.GIE = 0; // Disable interrupts to ensure atomic read
    t = ticks;
    INTCONbits.GIE = 1;
    
    return t;
}

void init_osc(void) {
    OSCCON1bits.NDIV = 0b0011; // 32 MHz HFINTOSC / 8 = 4 MHz Fosc = 1 MHz instruction clock

    while (!OSCCON3bits.ORDY); // Wait for clock switch to complete
}

void init_timers(void) {
    // Timer2 is used to provide a 1 msec system tick for hit detection and other purposes
    T2CLKCONbits.T2CS = 0b0001; // Clock source Fosc/4
    T2CONbits.T2CKPS = 0b000; // 1:1 prescaler = 1 MHz clock with Fosc = 4 MHz
    T2CONbits.T2OUTPS = 0b1001; // 1:10 postscaler = 100 kHz count rate
    
    T2PR = 100; // 100 counts = 1 msec
    
    T2CONbits.T2ON = 1; // Turn on Timer2
    
    // Timer4 is used as the period source for CCP1 PWM generator
    T4CLKCONbits.T4CS = 0b0001; // Clock source Fosc/4 as required by CCP module
    T4CONbits.T4CKPS = 0b101; // 1:32 prescaler = 31.25 kHz clock with Fosc = 4 MHz
    T4CONbits.T4OUTPS = 0b0000; // 1:1 postscaler (postscaler not used by CCP)
    
    T4PR = 127; // Default to lowest frequency (assuming 50% duty cycle)
    
    T4CONbits.T4ON = 1; // Turn on Timer4
}

void init_ccp(void) {
    CCP1CONbits.FMT = 1; // Left-aligned pulse width
    CCP1CONbits.MODE = 0b1111; // PWM mode
    
    CCPTMRS0bits.C1TSEL = 0b10; // CCP1 PWM based on Timer4           
}

void init_pps() {
    // Special PPS pre-unlock sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;

    // Unlock PPS
    PPSLOCKbits.PPSLOCKED = 0;
     
    RC1PPS = 0x15; // SSP1DAT output
    RC0PPS = 0x14; // SSP1CLK output
    
    RC7PPS = 0x09; // CCP1 output
    
    // Lock PPS
    PPSLOCKbits.PPSLOCKED = 1;
}

void init_mssp(void) {
    SSP1CON1bits.SSPM = 0b1010; // SPI Master mode, clock = FOSC/(4 * (SSPxADD+1))
    SSP1CON1bits.CKP = 0; // Clock idle state is low
    SSP1STATbits.CKE = 1; // Transmit on active-to-idle
    
    SSP1ADD = 0x04; // 200 kHz at Fosc = 4 MHz
    
    SSP1CON1bits.SSPEN = 1; // Enable MSSP1 for SPI
}

void init_interrupts(void) {
    PIE4bits.TMR2IE = 1; // Enable Timer2 period match interrupt
}

void init_ports(void) {
    // Disable all analog functions
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    // Set port directions
    TRISA = _TRISA_TRISA0_MASK | _TRISA_TRISA1_MASK;
    TRISB = 0x00;
    TRISC = _TRISC_TRISC2_MASK | _TRISC_TRISC3_MASK;

    // Enable weak pull-ups on inputs
    WPUA = _WPUA_WPUA0_MASK | _WPUA_WPUA1_MASK;
    WPUC = _WPUC_WPUC2_MASK | _WPUC_WPUC3_MASK;
}

void init_system(void) {
    init_osc();
    init_ports();
    init_timers();
    init_ccp();
    init_pps();
    init_mssp();
    init_interrupts();

    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1; // Enable global interrupts
}

void delay_ms(uint16_t delay_time) {
    uint16_t start_time;
    
    start_time = get_ticks();
    
    while ((get_ticks() - start_time) < delay_time); // Spin loop until time is expired
}

void write_spi(uint8_t val) {
    SSP1BUF = val;
    
    while (!PIR3bits.SSP1IF); // Wait for transmission to complete
    
    PIR3bits.SSP1IF = 0;
}

void buzzer_on(uint8_t period) {
    T4PR = period;
    
    CCPR1H = period / 2; // Always use 50% duty cycle, left-aligned
    CCPR1L = 0;
    
    CCP1CONbits.CCP1EN = 1;
}

void buzzer_off(void) {
    CCP1CONbits.CCP1EN = 0;
}

void update_display(uint8_t color, uint8_t score, bool hit) {
    // Avoid going beyond end of digits array
    if (score > 9) {
        score = 9;
    }
    
    if (color == RED_DISPLAY) {
        RED_SCORE_LATCH = 1;
        GREEN_SCORE_LATCH = 0;
    } else {
        RED_SCORE_LATCH = 0;
        GREEN_SCORE_LATCH = 1;
    }
    
    write_spi(digits[score]);
    
    if (hit) {
        write_spi(HIT_INDICATOR_ON);
    } else {
        write_spi(HIT_INDICATOR_OFF);
    }
    
    RED_SCORE_LATCH = 0;
    GREEN_SCORE_LATCH = 0;
}

void signal_increment(void) {
    // Sound a brief tone to indicate that the score has been incremented

    buzzer_on(BUZZER_PERIOD);
    delay_ms(SCORE_INC_BUZZER_TIME);
    buzzer_off();
}

void signal_reset(void) {
    // Sound 3 quick tones to indicate that the score has automatically reset

    uint8_t i;
    
    delay_ms(SCORE_RESET_TIME);
                    
    for (i = 0; i < 3; i++) {
        buzzer_on(BUZZER_PERIOD);
        delay_ms(SCORE_RESET_SIGNAL_TIME);
        buzzer_off();
        delay_ms(SCORE_RESET_SIGNAL_TIME);
    }
}

void main(void) {
    uint8_t red_score = 0;
    uint8_t green_score = 0;
    
    init_system();
    
    state = STATE_RESET;
    buzzer_armed = true;
    last_reset_timestamp = 0;
    consecutive_activations = 0;
    
    update_display(RED_DISPLAY, red_score, false);
    update_display(GREEN_DISPLAY, green_score, false);

    while (1) {
        switch (state) {
            case STATE_RESET:            
                // Record timestamp for disconnect detection
                last_reset_timestamp = get_ticks();
                
                // Turn off hit LEDs
                update_display(RED_DISPLAY, red_score, false);
                update_display(GREEN_DISPLAY, green_score, false);

                // Turn off buzzer
                buzzer_off();

                // Clear all event timestamps and flags
                red_start_timestamp = 0;
                green_start_timestamp = 0;
                lockout_start_timestamp = 0;
                
                red_inc_start_timestamp = 0;
                green_inc_start_timestamp = 0;
                
                score_cooldown_timestamp = 0;

                red_pending = false;
                green_pending = false;
                lockout_pending = false;
                
                red_inc_pending = false;
                green_inc_pending = false;
                
                score_cooldown_active = false;

                // Record timestamp to check for possible disconnect condition
                ready_timestamp = get_ticks();
                state = STATE_READY;
                break;
            case STATE_READY:
                // Check to see if buzzer should be rearmed (i.e., disconnect condition is cleared)
                if (!buzzer_armed) {
                    if (get_ticks() - ready_timestamp > DISCONNECT_DETECT_TIME) {
                        buzzer_armed = true;
                        consecutive_activations = 0;
                    }
                }
                
                // Check to see if score increment cooldown should be cleared
                if (score_cooldown_active) {
                    if (get_ticks() - score_cooldown_timestamp > SCORE_COOLDOWN_TIME) {
                        score_cooldown_active = false;
                    }
                }
                
                // If red score increment signal is active and cooldown time has elapsed since last
                // score increment
                if (RED_SCORE_INCREMENT && !score_cooldown_active) {
                    // If increment signal was previously active, check to see if the debounce time has elapsed
                    if (red_inc_pending) {
                        if (get_ticks() - red_inc_start_timestamp > SCORE_INCREMENT_TIME) {
                            // If both score signals are active, reset score
                            if (GREEN_SCORE_INCREMENT) {
                                red_score = 0;
                                green_score = 0;
                            } else {
                                red_score++;
                            }                            
                            
                            update_display(RED_DISPLAY, red_score, false);
                            update_display(GREEN_DISPLAY, green_score, false);
                            
                            signal_increment();
                            
                            red_inc_pending = false;
                            
                            // Set cooldown to prevent multiple increments in rapid succession
                            score_cooldown_timestamp = get_ticks();
                            score_cooldown_active = true;
                        }
                    } else {
                        // Set debounce timer on initial signal activation
                        red_inc_start_timestamp = get_ticks();
                        red_inc_pending = true;                         
                    }
                } else {
                    // Clear pending increment if no signal or if signal goes inactive prior to debounce time
                    red_inc_pending = false;
                }
                
                // If green score increment signal is active and cooldown time has elapsed since last
                // score increment
                if (GREEN_SCORE_INCREMENT && !score_cooldown_active) {
                    // If increment signal was previously active, check to see if the debounce time has elapsed
                    if (green_inc_pending) {
                        if ((get_ticks() - green_inc_start_timestamp > SCORE_INCREMENT_TIME)) {
                            // If both score signals are active, reset score
                            if (RED_SCORE_INCREMENT) {
                                red_score = 0;
                                green_score = 0;
                            } else {
                                green_score++;
                            }                            
                            
                            update_display(RED_DISPLAY, red_score, false);
                            update_display(GREEN_DISPLAY, green_score, false);
                            
                            signal_increment();
                            
                            green_inc_pending = false;
                            
                            // Set cooldown to prevent multiple increments in rapid succession
                            score_cooldown_timestamp = get_ticks();
                            score_cooldown_active = true;
                        }
                    } else {
                        // Set debounce timer on initial signal activation
                        green_inc_start_timestamp = get_ticks();
                        green_inc_pending = true;                         
                    }
                } else {
                    // Clear pending increment if no signal or if signal goes inactive prior to debounce time
                    green_inc_pending = false;
                }
                
                // Automatically reset score if the score limit is reached
                if ((red_score >= SCORE_LIMIT) || (green_score >= SCORE_LIMIT)) {
                    signal_reset();
                    
                    red_score = 0;
                    green_score = 0;
                    
                    update_display(RED_DISPLAY, red_score, false);
                    update_display(GREEN_DISPLAY, green_score, false);
                }
                
                // If red side tip is depressed and if it has been depressed for MIN_HIT_TIME ms
                if (RED_FOIL) {
                    if (red_pending) {
                        // Then if this is the first hit recorded in the sequence
                        // set the lockout timer to allow time for the other fencer to hit
                        if ((get_ticks() - red_start_timestamp) > MIN_HIT_TIME) {                        
                            if (!lockout_pending) {
                                lockout_start_timestamp = red_start_timestamp;
                                lockout_pending = true;
                            }

                            // Light up red hit indicator segments
                            update_display(RED_DISPLAY, red_score, true); 

                            // Sound buzzer if armed
                            if (buzzer_armed) {
                                buzzer_on(BUZZER_PERIOD); 
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        red_start_timestamp = get_ticks();
                        red_pending = true;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    red_pending = false;
                }
                
                // If green side tip is depressed and if it has been depressed for MIN_HIT_TIME ms
                if (GREEN_FOIL) {
                    // Then if this is the first hit recorded in the sequence
                    // set the lockout timer to allow time for the other fencer to hit
                    if (green_pending) {
                        if ((get_ticks() - green_start_timestamp) > MIN_HIT_TIME) {
                            if (!lockout_pending) {
                                lockout_start_timestamp = green_start_timestamp;
                                lockout_pending = true;
                            }

                            // Light up green hit indicator segments
                            update_display(GREEN_DISPLAY, green_score, true);

                            // Sound buzzer if armed
                            if (buzzer_armed) {
                                buzzer_on(BUZZER_PERIOD); 
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        green_start_timestamp = get_ticks();
                        green_pending = true;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    green_pending = false;
                }

                // Continue to check for a hit scored by the other fencer until
                // lockout time has expired
                if (lockout_pending && (get_ticks() - lockout_start_timestamp > LOCKOUT_TIME)) {
                    state = STATE_LOCKOUT; // Go into lockout state after LOCKOUT_TIME ms
                }
                break;
            case STATE_LOCKOUT:
                 // Check for multiple activations in rapid succession which can indicate a disconnected foil
                if (get_ticks() - last_reset_timestamp < DISCONNECT_DETECT_TIME) {
                    consecutive_activations++;
                    
                    // Subtract 2 as workaround for the way consecutive activations are counted
                    if (consecutive_activations > DISCONNECT_DETECT_COUNT - 2) {
                        buzzer_armed = false;
                    }
                }
                
                // Allow buzzer to sound for BUZZER_TIME ms, keep LEDs on for
                // additional ADD_LIGHT_TIME ms after that
                delay_ms(BUZZER_TIME);
                buzzer_off();
                delay_ms(ADDL_LIGHT_TIME);

                // Reset for next hit
                state = STATE_RESET;
                break;
        }
    }
}
