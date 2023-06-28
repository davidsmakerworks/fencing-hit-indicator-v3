
/*
 * Fencing Hit Indicator v3
 * Copyright (c) 2023 David Rice
 * 
 * github.com/davidsmakerworks
 * 
 * This is a simple fencing hit indicator that implements the hit time and
 * lockout time as specified by current foil fencing rules. Score display is als
 * implemented.
 * 
 * Designed for PIC 16F18855
 * 
 * INPUTS:
 * RA0 - Red fencer B-line
 * RC3 - Green fencer B-line
 * 
 * RA1 - Red fencer score increment
 * RC2 - Green fencer score increment 
 * 
 * OUTPUTS:
 * RB4 - Red LED
 * RC5 - Green LED
 * RC2 - Active buzzer
 * 
 * RB5 - Red 7-segment driver latch enable
 * RC4 - Green 7-segment driver latch enable
 * RB1 - 7-segment driver data (SSP1DAT)
 * RB0 - 7 segment driver clock (SSP1CLK)
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

#define FALSE 0
#define TRUE !FALSE

// Pins where the fencers' foils are connected
#define RED_FOIL PORTAbits.RA0
#define GREEN_FOIL PORTCbits.RC3

// Pins where hit indicator LED driver is connected
#define RED_LED LATBbits.LATB4
#define GREEN_LED LATCbits.LATC5

// Pin where active buzzer is connected
#define BUZZER LATCbits.LATC7

// Pins where score driver latch enables are connected
#define RED_SCORE_LATCH LATBbits.LATB5
#define GREEN_SCORE_LATCH LATCbits.LATC4

// Pins where score increment signals are connected
#define RED_SCORE_INCREMENT !PORTAbits.RA1
#define GREEN_SCORE_INCREMENT !PORTCbits.RC2

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
#define SCORE_INCREMENT_TIME 750

// Time to sound buzzer when score is incremented
#define SCORE_INC_BUZZER_TIME 100

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

// TRUE if buzzer is armed (i.e., both foils are connected and hits are
// valid). FALSE if the buzzer has been counding continuously due to one
// or both foils beind disconnected.
uint8_t buzzer_armed;

// State variable for finite state machine in main loop
uint8_t state;

// Indicates if the tip is currently depressed (i.e., a hit will be registered
// if the tip remains depressed for MIN_HIT_TIME ms) 
uint8_t red_pending;
uint8_t green_pending;

// Indicated that a hit has been registered by one fencer, and now the other fencer
// has LOCKOUT_TIME ms to also register a hit
uint8_t lockout_pending;

uint8_t red_inc_pending;
uint8_t green_inc_pending;

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

void init_osc(void) {
    OSCCON1bits.NDIV = 0b0011; // 32 MHz HFINTOSC / 8 = 4 MHz Fosc = 1 MHz instruction clock

    while (!OSCCON3bits.ORDY); // Wait for clock switch to complete
}

void init_timer(void) {
    T2CLKCONbits.T2CS = 0b0001; // Clock source Fosc/4
    T2CONbits.T2CKPS = 0b00; // 1:1 prescaler = 1 MHz clock with Fosc = 4 MHz
    T2CONbits.T2OUTPS = 0b1001; // 1:10 postscaler = 100 kHz count rate
    
    PR2 = 100; // 100 counts = 1 msec
    
    T2CONbits.TMR2ON = 1; // Turn on Timer2
}

void init_pps() {
    // Unlock PPS
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
     
    RB1PPS = 0x15; // SSP1DAT
    RB0PPS = 0x14; // SSP1CLK
    
    // Lock PPS
    PPSLOCKbits.PPSLOCKED = 1;
}

void init_spi(void) {
    SSP1CON1bits.SSPM = 0b1010; // SPI Master mode, clock = FOSC/(4 * (SSPxADD+1))
    SSP1CON1bits.CKP = 0; // Clock idle state is low
    SSP1STATbits.CKE = 1; // Transmit on active-to-idle
    
    SSP1ADD = 0x09; // 100 kHz at Fosc = 4 MHz
    
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
    init_timer();
    init_pps();
    init_spi();
    init_interrupts();

    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1; // Enable global interrupts
}

void delay_ms(uint16_t delay_time) {
    uint16_t start_time;
    
    start_time = ticks;
    
    while ((ticks - start_time) < delay_time); // Spin loop until time is expired
}

void write_spi(uint8_t val) {
    SSP1BUF = val;
    
    while (!PIR3bits.SSP1IF); // Wait for transmission to complete
    
    PIR3bits.SSP1IF = 0;
}

void update_score(uint8_t red_score, uint8_t green_score) {
    if (red_score > 9) {
        red_score = 9;
    }
    
    if (green_score > 9) {
        green_score = 9;
    }
    
    RED_SCORE_LATCH = 1;
    GREEN_SCORE_LATCH = 0;
    write_spi(digits[red_score]);
    
    RED_SCORE_LATCH = 0;
    GREEN_SCORE_LATCH = 1;
    write_spi(digits[green_score]);
    
    GREEN_SCORE_LATCH = 0;
}

void main(void) {
    // TODO: Maybe make these global variables since the program is so simple
    uint8_t red_score = 0;
    uint8_t green_score = 0;
    
    init_system();
    
    state = STATE_RESET;
    buzzer_armed = TRUE;
    last_reset_timestamp = 0;
    consecutive_activations = 0;
    
    update_score(red_score, green_score);

    while (1) {
        switch (state) {
            case STATE_RESET:            
                // Record timestamp for disconnect detection
                last_reset_timestamp = ticks;
                
                // Turn off LEDs
                RED_LED = 0;
                GREEN_LED = 0;

                // Turn off buzzer
                BUZZER = 0;

                red_start_timestamp = 0;
                green_start_timestamp = 0;
                lockout_start_timestamp = 0;
                
                red_inc_start_timestamp = 0;
                green_inc_start_timestamp = 0;

                red_pending = FALSE;
                green_pending = FALSE;
                lockout_pending = FALSE;
                
                red_inc_pending = FALSE;
                green_inc_pending = FALSE;

                ready_timestamp = ticks;
                state = STATE_READY;
                break;
            case STATE_READY:
                // Check to see if buzzer should be rearmed (i.e., disconnect condition is cleared)
                if (!buzzer_armed) {
                    if (ticks - ready_timestamp > DISCONNECT_DETECT_TIME) {
                        buzzer_armed = TRUE;
                        consecutive_activations = 0;
                    }
                }
                
                if (RED_SCORE_INCREMENT) {
                    if (red_inc_pending) {
                        if ((ticks - red_inc_start_timestamp > SCORE_INCREMENT_TIME)) {
                            if (GREEN_SCORE_INCREMENT) {
                                red_score = 0;
                                green_score = 0;
                            } else {
                                red_score++;
                            }                            
                            
                            update_score(red_score, green_score);
                            
                            BUZZER = 1;
                            delay_ms(SCORE_INC_BUZZER_TIME);
                            BUZZER = 0;
                            
                            red_inc_pending = FALSE;
                        }
                    } else {
                        red_inc_start_timestamp = ticks;
                        red_inc_pending = TRUE;                         
                    }
                } else {
                    red_inc_pending = FALSE;
                }
                
                if (GREEN_SCORE_INCREMENT) {
                    if (green_inc_pending) {
                        if ((ticks - green_inc_start_timestamp > SCORE_INCREMENT_TIME)) {
                            if (RED_SCORE_INCREMENT) {
                                red_score = 0;
                                green_score = 0;
                            } else {
                                green_score++;
                            }                            
                            
                            update_score(red_score, green_score);
                            
                            BUZZER = 1;
                            delay_ms(SCORE_INC_BUZZER_TIME);
                            BUZZER = 0;
                            
                            green_inc_pending = FALSE;
                        }
                    } else {
                        green_inc_start_timestamp = ticks;
                        green_inc_pending = TRUE;                         
                    }
                } else {
                    green_inc_pending = FALSE;
                }
                
                if (RED_FOIL) {
                    // If tip is depressed...
                    if (red_pending) {
                        // ...and if it has been depressed for MIN_HIT_TIME ms...
                        if ((ticks - red_start_timestamp) > MIN_HIT_TIME) {
                            // ...then if this is the first hit recorded in the sequence...
                            if (!lockout_pending) {
                                // ...set the lockout timer to allow time for the other fencer to hit
                                lockout_start_timestamp = red_start_timestamp;
                                lockout_pending = TRUE;
                            }
                            RED_LED = 1; // Light up red LED
                            if (buzzer_armed) {
                                BUZZER = 1; // Sound buzzer if armed
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        red_start_timestamp = ticks;
                        red_pending = TRUE;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    red_pending = FALSE;
                }
                
                if (GREEN_FOIL) {
                    // If tip is depressed...
                    if (green_pending) {
                        // ...and if it has been depressed for MIN_HIT_TIME ms...
                        if ((ticks - green_start_timestamp) > MIN_HIT_TIME) {
                            // ...then if this is the first hit recorded in the sequence...
                            if (!lockout_pending) {
                                // ...set the lockout timer to allow time for the other fencer to hit
                                lockout_start_timestamp = green_start_timestamp;
                                lockout_pending = TRUE;
                            }
                            GREEN_LED = 1; // Light up green LED
                            if (buzzer_armed) {
                                BUZZER = 1; // Sound buzzer if armed
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        green_start_timestamp = ticks;
                        green_pending = TRUE;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    green_pending = FALSE;
                }

                // Continue to check for a hit scored by the other fencer until
                // lockout time has expired
                if (lockout_pending && (ticks - lockout_start_timestamp > LOCKOUT_TIME)) {
                    state = STATE_LOCKOUT; // Go into lockout state after LOCKOUT_TIME ms
                }
                break;
            case STATE_LOCKOUT:
                 // Check for multiple activations in rapid succession which can indicate a disconnected foil
                if (ticks - last_reset_timestamp < DISCONNECT_DETECT_TIME) {
                    consecutive_activations++;
                    
                    // Subtract 2 as workaround for the way consecutive activations are counted
                    if (consecutive_activations > DISCONNECT_DETECT_COUNT - 2) {
                        buzzer_armed = FALSE;
                    }
                }
                
                // Allow buzzer to sound for BUZZER_TIME ms, keep LEDs on for
                // additional ADD_LIGHT_TIME ms after that
                delay_ms(BUZZER_TIME);
                BUZZER = 0;
                delay_ms(ADDL_LIGHT_TIME);

                state = STATE_RESET; // Reset for next hit
                break;
        }
    }
}
