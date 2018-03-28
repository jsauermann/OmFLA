/*
    Copyright (C) 2018  Dr. Jürgen Sauermann

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  This is a helper program for calibrating the internal RC oscillator
  of an Atmel 4313 in order to produce a proper baud rate of 57600 Baud.

   There are quite a few different ways to achieve this:

   1a. Use a 3.686400 Hz crystal and the hardware UART of the ATtiny4313,
   1b. Use a 3.686400 Hz crystal and the software UART of the ATtiny4313,
   2.  Use a 4.000000 Hz crystal and the software UART of the ATtiny4313,
   3a. Use the (uncalibrated) built-in 4 MHz RC oscillator of the ATtiny4313
       and the software UART, and calibrate SOFT_COUNT to get the desired
       baud rate, 
   3b. Use the calibrated built-in 4 MHz RC oscillator of the ATtiny4313
       and the software UART, and leave SOFT_COUNT as is to get the desired
       baud rate,

   1a, 1b, and 2 have the disadvantage of using slighly more current, but
   give the precisest baud rate.

   2. is always worse than 1a or 1b. and should only be used if a very precise
      baudrate is needed and no 3.686400 Hz crystal is not available. It can be
      used for other crystal frequencies as well by scaling SOFT_COUNT
      accordingly.

   3b. is the method currently choosen by OmFLA. Recent current measurements
       have shown, though, that the extra current consumed by a crystal is
       only about 10% of the total current, so this choice will soon be
       reconsidered.

   Macro VARY_COUNT below selects if the SOFT_COUNT for the UART or the
   built-in 4 MHz RC oscillator shall be calibrated.
 */

#define VARY_COUNT 0

# define F_CPU 4000000
// # define F_CPU 3686400

#define SOFT_BAUD 57600
// #define SOFT_BAUD 9600
#define SOFT_PORT_1 PORTB
#define SOFT_PBIT_1 B_PROG_MISO
#define SOFT_PORT_2 PORTD
#define SOFT_PBIT_2 D_TCM_TxD

#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#ifndef __AVR_ATtiny4313__
#error "__AVR_ATtiny4313__ is not defined !!!"
#endif

enum CPU_cycles
   {
     DELAY_LOOP_LEN = 3,   // one iteration of _delay_loop_1()
     DELAY_PREAMBLE = 13,   // set/clear bit etc
                                                         // 9600   57600
     CYCLES_PER_BIT = F_CPU / SOFT_BAUD,                 //  384      64
     DELAY_PER_BIT  = CYCLES_PER_BIT - DELAY_PREAMBLE,   //  371      51
     SOFT_COUNT     = DELAY_PER_BIT/DELAY_LOOP_LEN,      //  123      17

     SOFT_MIN   = ( 4*SOFT_COUNT)/10,   // 40 % of SOFT_COUNT
     SOFT_MAX   = (25*SOFT_COUNT)/10,   // 250 % of SOFT_COUNT
   };

static bool soft_mode = true;
static uint16_t soft_count = SOFT_COUNT;
static uint8_t  initial_calib = 0;
static uint16_t batt_result = 0;

enum
{
   __A_XTAL_1  = 1 << PA0,   // XTAL1 (not used)
   __A_XTAL_2  = 1 << PA1,   // XTAL2 (not used)
   __A_RESET   = 1 << PA2,   // RESET (NEVER drive it high!)
   outputs_A   = __A_XTAL_1
               | __A_XTAL_2,
   pullup_A  = ~ outputs_A,

   B_BTEST_OUT = 1 << PB0,   // battery test output
   B_BTEST_IN  = 1 << PB1,   // battery test input
   B_MOSI      = 1 << PB2,   // MOSI to RFID reader
   B_TCM_POWER = 1 << PB3,   // Power for TCM 310 radio module
   B_LED_GREEN = 1 << PB4,   // green LED
   __PROG_MOSI = 1 << PB5,   // MOSI from serial programmer (not used)
   B_PROG_MISO = 1 << PB6,   // MISO to   serial programmer or soft UART Tx
   __PROG_SCL  = 1 << PB7,   // SCL  from serial programmer (not used)
   outputs_B1  = B_MOSI
               | B_TCM_POWER
               | B_LED_GREEN
               | B_BTEST_OUT
               | B_PROG_MISO,
   outputs_B   = outputs_B1 | B_BTEST_OUT,
   pullup_B  = ~ outputs_B,

   D_TCM_RxD   = 1 << PD0,   // RxD from TCM 310 radio module
   D_TCM_TxD   = 1 << PD1,   // TxD to   TCM 310 radio module
   D_SCLK      = 1 << PD2,   // Data clock to RFID reader
   D_SSEL      = 1 << PD3,   // Slave SELect to RFID reader
   D_LED_RED   = 1 << PD4,   // red LED
   D_MISO      = 1 << PD5,   // MISO from RFID reader
   D_BEEPER    = 1 << PD6,   // beeper
   outputs_D   = D_SCLK
               | D_SSEL
               | D_LED_RED
               | D_BEEPER,
   pullup_D  = ~ outputs_D,
};

#define get_pin(port, bit)    (PIN  ## port &    port ## _ ## bit ? 0xFF : 0x00)
#define set_pin(port, bit)    (PORT ## port |=   port ## _ ## bit)
#define clr_pin(port, bit)    (PORT ## port &= ~ port ## _ ## bit)
#define output_pin(port, bit) (DDR  ## port |=   port ## _ ## bit)
#define input_pin(port, bit)  (DDR  ## port &= ~ port ## _ ## bit)

//-----------------------------------------------------------------------------
static void
init_hardware()
{
   // set pins to a preliminary value (0), which will be overridden below
   //
   DDRA = outputs_A;
   DDRB = outputs_B;
   DDRD = outputs_D;

   PORTA = pullup_A;
   PORTB = pullup_B | B_PROG_MISO;
   PORTD = pullup_D | D_BEEPER | D_TCM_TxD;

   // CPU clock prescaler: x1
   //
   CLKPR = 0x80;   // enable write to CLKPR
   CLKPR = 0x00;   // x1 clock

   // hardware UART
   //
   enum
      {
        BAUDRATE = 57600,
        F_CPU_16 = F_CPU/16,
        BAUD_DIVISOR = (F_CPU_16 / BAUDRATE) - 1
      };

   UBRRH = BAUD_DIVISOR >> 8;
   UBRRL = BAUD_DIVISOR & 0xFF;

   UCSRB = 0 << RXEN | 1 << TXEN;
   UCSRC = 1 << USBS | 3 << UCSZ0;   // async, 2 stop, 8 data

   // set up pins again AFTER all alternate functions have been enabled...

   // set data directions, see 10.1.1 Configuring the Pin
   //
   DDRA = outputs_A;
   DDRB = outputs_B;
   DDRD = outputs_D;

   // enable pullups on inputs and drive all outputs except TxD low,
   // see 10.1.1 Configuring the Pin
   //
   PORTA = pullup_A;
   PORTB = pullup_B | B_PROG_MISO | B_TCM_POWER;
   PORTD = pullup_D | D_BEEPER | D_TCM_TxD;
}
//-----------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
}
//-----------------------------------------------------------------------------
ISR(ANA_COMP_vect)
{
   batt_result = TCNT1;
}
//-----------------------------------------------------------------------------
static void
hard_char(uint8_t ch)
{
   while (!(UCSRA & 1 << UDRE))   ;   // wait for DR empty
   UDR = ch;
}
//-----------------------------------------------------------------------------
static void
soft_char(uint8_t ch)
{
   //             ╔╦═════════════════  stop bit(s)
   //             ║║     ╔╦══════════  8 data bits
   //             ║║     ║║     ╔════  start bit
int16_t bits = (0xFF00 | ch) << 1;
   for (uint8_t j = 0; j < 12; ++j)   // 1 start + 8 data + 3 stop
       {
         if (bits & 1)
            {
              SOFT_PORT_1 |=  SOFT_PBIT_1;
              SOFT_PORT_2 |=  SOFT_PBIT_2;
            }
         else
            {
              SOFT_PORT_1 &= ~SOFT_PBIT_1;
              SOFT_PORT_2 &= ~SOFT_PBIT_2;
              SOFT_PORT_2 &= ~SOFT_PBIT_2;   // compensate sbrs skew
            }
         bits >>= 1;
         _delay_loop_1(soft_count);
       }
}
//-----------------------------------------------------------------------------
static void
print_char(uint8_t ch)
{
   if (soft_mode)   soft_char(ch);
   else             hard_char(ch);
}
//-----------------------------------------------------------------------------
static void
print_hex1(uint8_t ch)
{
   ch &= 0x0F;
   ch += (ch <= 9) ? '0' : '7';
   print_char(ch);
}
//-----------------------------------------------------------------------------
static void
print_hex2(uint8_t ch)
{
   print_hex1(ch >> 4);
   print_hex1(ch);
}
//-----------------------------------------------------------------------------
inline void
print_hex4(uint16_t ch)
{
   print_hex2(ch >> 8);
   print_hex2(ch);
}
//-----------------------------------------------------------------------------
static void
print_dec(uint32_t val)
{
char buf[10];
uint8_t plen = 1;

   for (uint8_t j = 0; j < sizeof(buf);)
       {
         if ((buf[j++] = val % 10))   plen = j;
         val /= 10;
       }

   while (plen)   print_char('0' + buf[--plen]);
}
//-----------------------------------------------------------------------------
static void
_print_string(const char * str, int16_t val = -1)
{
   for (uint8_t cc; (cc = pgm_read_byte(str++));)
       switch(cc)
          {
            case 0x80: print_hex2(val);   break;
            case 0x81: print_dec(val);    break;
            default:   print_char(cc);
          }
}
#define print_string(str)       _print_string(PSTR((str)), -1)
#define print_stringv(str, val) _print_string(PSTR((str)), val)
//-----------------------------------------------------------------------------
inline void
battery_test()
{
   ACSR = 0 << ACD     // DO NOT disable comparator
        | 1 << ACBG    // DO     use internal bandgap reference
        | 0 << ACI     // DO NOT clear interrupt
        | 0 << ACIE    // DO NOT enable interrupt
        | 0 << ACIC    // DO NOT enable input capture
        | 2 << ACIS0   // interrupt on falling edge
        ;
   _delay_ms(1);         // wait 1 ms for bandgap reference to start up

   TCCR1A = 0;
   TCCR1B = 1 << CS10;       // clock source: io-clk

   batt_result = 9999;
   ACSR = 0 << ACD     // DO NOT disable comparator
        | 1 << ACBG    // DO     use internal bandgap reference
        | 1 << ACI     // DO     clear interrupt
        | 1 << ACIE    // DO     enable interrupt
        | 0 << ACIC    // DO NOT enable input capture
        | 2 << ACIS0   // interrupt on falling edge
        ;

   input_pin(B, BTEST_OUT);  // BTEST_OUT: in: pullup charges capacitor
   set_pin(B, BTEST_OUT);    // enable pullup on AIN0

   TCNT1 = 0;
   sei();
   _delay_ms(1);
   cli();

   ACSR = 1 << ACD     // DO     disable comparator
        | 0 << ACBG    // DO NOT use internal bandgap reference
        | 1 << ACI     // DO     clear interrupt
        | 0 << ACIE    // DO NOT enable interrupt
        | 0 << ACIC    // DO NOT enable input capture
        | 2 << ACIS0   // interrupt on falling edge
        ;

   // discharge capacitor for the next masurement
   //
   clr_pin(B, BTEST_OUT);      // disable pullup on AIN0
   output_pin(B, BTEST_OUT);   // BTEST_OUT direction = out

   clr_pin(B, BTEST_OUT);      // drive AIN0 low
   _delay_ms(10);
   set_pin(B, BTEST_OUT);      // drive AIN0 high
   _delay_ms(10);
   clr_pin(B, BTEST_OUT);      // drive AIN0 low
   _delay_ms(10);
   set_pin(B, BTEST_OUT);      // drive AIN0 high
   _delay_ms(30);
   clr_pin(B, BTEST_OUT);      // drive AIN0 low
}
//-----------------------------------------------------------------------------
void
doit()
{
const uint8_t * e = 0;
   battery_test();
   print_string("\n\nF_CPU=");          print_dec(F_CPU);
   print_string(", soft_mode=");        print_dec(soft_mode);
   print_string(", SOFT_COUNT=");       print_dec(SOFT_MIN);
   print_string("/");                   print_dec(SOFT_COUNT);
   print_string("/");                   print_dec(SOFT_MAX);
   print_string(", soft_count=");       print_dec(soft_count);
   print_string(", OSCCAL=");           print_hex2(OSCCAL);
   print_string("\n    battery=");      print_dec(batt_result);
   print_string(", initial OSCCAL=");   print_hex2(initial_calib);
   print_string(", eeprom=");
   for (uint8_t j = 0; j < 6; ++j)
       {
         print_hex2(eeprom_read_byte(e++));
         print_string(" ");
       }
   print_string("...\n");
}
//-----------------------------------------------------------------------------
int
main(int, char *[])
{
   initial_calib = OSCCAL;

   init_hardware();

   _delay_ms(200);
   print_string("\n\neeprom[0] (calibration byte): 0x");
   print_hex2(eeprom_read_byte(0));
   print_string("...\n\n");

   soft_mode = true;
   for (;;)
       {
         PIND = D_LED_RED;   // toggle red LED

#if VARY_COUNT
         for (soft_count = SOFT_MIN; soft_count <= SOFT_MAX; ++soft_count)
             {
               doit();
               _delay_ms(1);
             }
#else
         for (uint8_t cal = 0; cal < 128; ++cal)
             {
               OSCCAL = cal;
               _delay_ms(10);   // allow RC oscillator to change
               doit();
             }
#endif
       }
}
//-----------------------------------------------------------------------------
