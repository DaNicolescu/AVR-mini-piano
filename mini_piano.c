#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "lcd.h"
#include "sample1.h"
#include "sample2.h"
#include "sample3.h"
#include "drum.h"

int idx = 0;
int current_note = -1;
unsigned int keyboard;
unsigned char volume_div = 1;

char notes[8][4] = {"DO", "RE", "MI", "FA", "SOL", "LA", "SI", "DO"};

void timer0_start(void)
{
    // interrupt on compare A
    TIMSK0 |= (1 << OCIE0A);
    // CTC, top OCRA
    TCCR0B |= (0 << WGM02);
    TCCR0A |= (1 << WGM01) | (0 << WGM00);
}

void timer0_stop(void)
{
    TCCR0B = 0;
    TCCR0A = 0;
    TIMSK0 = 0;
    OCR0A = 0;
    TCNT0 = 0;
}

void timer1_start(void)
{
    // 8-bit FastPWM
    TCCR1B |= (1 << WGM12);
    TCCR1A |= (1 << WGM10);
    // channel B inverted
    TCCR1A |= (1 << COM1B0) | (1 << COM1B1);
    // prescaler 1
    TCCR1B |= (1 << CS10);
}

void timer1_stop(void)
{
    TCCR1B = 0;
    TCCR1A = 0;
    OCR1B = 0;
    TCNT1 = 0;
}

void timer2_start(void)
{
    // the value used for a frequency of 200Hz (every 0.02s)
    OCR2A = 78;

    // activate OCR2A compare interupt
    TIMSK2 = (1 << OCIE2A);

    // CTC with TOP at OCR0A
    // prescaler 1024
    // clear on compare
    TCCR2A = (1 << COM2A1) | (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
}

ISR(TIMER2_COMPA_vect)
{
    int i;

    for (i = 0; i < 8; ++i) {
        if (!(PINB & (1 << i))) {
            // if it has already been pressed the last time do nothing
            if ((keyboard & (1 << i)) > 0) {
            // the button has just been pressed
            } else {
                // sampling rate of 8kHz for 1, 3 and 7
                if (i == 0 || i == 2 || i == 6) {
                    OCR0A = 250;
                    TCCR0B |= (2 << CS00);
                // sampling rate of 13kHz 2
                } else if (i == 1) {
                    OCR0A = 154;
                    TCCR0B |= (1 << CS01);
                // sampling rate of 11kHz for 4
                } else if (i == 3) {
                    OCR0A = 182;
                    TCCR0B |= (1 << CS01);
                // sampling rate of 14 kHz for 5
                } else if (i == 4) {
                    OCR0A = 143;
                    TCCR0B |= (1 << CS01);
                // sampling rate of 16kHz for 6
                } else if (i == 5) {
                    OCR0A = 125;
                    TCCR0B |= (1 << CS01);
                // sampling rate of 11kHz for 8
                } else if (i == 7) {
                    OCR0A = 182;
                    TCCR0B |= (1 << CS01);
                }

                timer0_start();
                timer1_start();
                idx = 0;
                current_note = i;

                LCD_writeInstr(LCD_INSTR_clearDisplay);
                LCD_printAt(i * 2, notes[i]);
                LCD_printAt(LCD_INSTR_nextLine + i * 2, "##");  
                keyboard |= _BV(i);
            }
        } else {
            keyboard &= ~_BV(i);
        }
    }

    for (; i < 12; i++) {
        if (!(PINA & (1 << (i - 8)))) {
            // if it has already been pressed the last time do nothing
            if ((keyboard & (1 << i)) > 0) {
            // the button has just been pressed
            } else {
                // volume up
                if (i == 10) {
                    if (volume_div > 1) {
                        --volume_div;
                    }

                    LCD_writeInstr(LCD_INSTR_clearDisplay);
                    LCD_printAt(0, "Volume Up:");
                    LCD_putCharAt(11, 6 - volume_div + 48);
                // volume down
                } else if (i == 11) {
                    if (volume_div < 5) {
                        ++volume_div;
                    }

                    LCD_writeInstr(LCD_INSTR_clearDisplay);
                    LCD_printAt(0, "Volume Down:");
                    LCD_putCharAt(13, 6 - volume_div + 48);
                // drum
                } else if (i == 9) {
                    OCR0A = 250;
                    TCCR0B |= (2 << CS00);

                    timer0_start();
                    timer1_start();
                    idx = 0;
                    current_note = 8;
                // switch LCD backlight
                } else if (i == 8) {
                    PORTC ^= (1 << PC2);
                }
                
                keyboard |= _BV(i);
            }
        } else {
            keyboard &= ~_BV(i);
        }
    }
}

ISR(TIMER0_COMPA_vect)
{
    if (current_note < 2) {
        OCR1B = pgm_read_byte(&sample1[idx++]) / volume_div;

        if(idx == sample1_size) {
            idx = 0;
            
            // the note has been played, so we stop the pwm
            timer0_stop();
            timer1_stop();
        }
    } else if (current_note < 6) {
        OCR1B = pgm_read_byte(&sample2[idx++]) / volume_div;

        if(idx == sample2_size) {
            idx = 0;
            
            // the note has been played, so we stop the pwm
            timer0_stop();
            timer1_stop();
        }
    } else if (current_note < 8) {
        OCR1B = pgm_read_byte(&sample3[idx++]) / volume_div;

        if(idx == sample3_size) {
            idx = 0;
            
            // the note has been played, so we stop the pwm
            timer0_stop();
            timer1_stop();
        }
    } else {
         OCR1B = pgm_read_byte(&drum[idx++]) / volume_div;

        if(idx == drum_size) {
            idx = 0;
            
            // the note has been played, so we stop the pwm
            timer0_stop();
            timer1_stop();
        }
    }
}

void buttons_init(void)
{
    // initialize keyboard buttons
    DDRB = 0;                     
    PORTB = 0xff;

    // initialize control buttons
    DDRA &= ~(1 << PA0);
    DDRA &= ~(1 << PA1);
    DDRA &= ~(1 << PA2);
    DDRA &= ~(1 << PA3);

    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    PORTA |= (1 << PA2);
    PORTA |= (1 << PA3);
}

int main(void)
{
    int i = 0;

    // LCD backlight
    DDRC |= (1 << PC2);
    PORTC |= (1 << PC2);

    // initialize LCD
    LCD_init();

    // display Welcome message
    for (i = 0; i < 5; i++) {
        LCD_writeInstr(LCD_INSTR_clearDisplay);
        LCD_printAt(i, "Welcome");
        LCD_printAt(LCD_INSTR_nextLine + i, "Play me!");    

        _delay_ms(500);
    }

    // switch LCD backlight off
    PORTC &= ~(1 << PC2);

    buttons_init();

    timer2_start();

    sei();

    // speaker output
    DDRD |= (1 << PD4);

    for(;;) {
    }

    return 0;
}
