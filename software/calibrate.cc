#define F_CPU 4000000
#define F_IO  F_CPU

#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#ifndef __AVR_ATtiny4313__
#error "__AVR_ATtiny4313__ is not defined !!!"
#endif

static uint16_t batt_result = 0;

enum
{
   __A_XTAL_1  = 1 << PA0,   // XTAL1 (not used)
   __A_XTAL_2  = 1 << PA1,   // XTAL2 (not used)
   __A_RESET   = 1 << PA2,   // RESET (not used)
   outputs_A   = __A_XTAL_1
               | __A_XTAL_2
               | __A_RESET,
   pullup_A  = ~ outputs_A,

   B_BTEST_OUT = 1 << PB0,   // battery test output
   B_BTEST_IN  = 1 << PB1,   // battery test input
   B_MOSI      = 1 << PB2,   // MOSI to RFID reader
   B_TCM_POWER = 1 << PB3,   // Power for TCM 310 radio module
   B_LED_GREEN = 1 << PB4,   // green LED
   __PROG_MOSI = 1 << PB5,   // MOSI from serial programmer (not used)
   __PROG_MISO = 1 << PB6,   // MISO to   serial programmer (not used)
   __PROG_SCL  = 1 << PB7,   // SCL  from serial programmer (not used)
   outputs_B1  = B_MOSI
               | B_TCM_POWER
               | B_LED_GREEN
               | B_BTEST_OUT,
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
   PORTB = pullup_B;
   PORTD = pullup_D;

   // CPU clock prescaler: x1
   //
   CLKPR = 0x80;   // enable write to CLKPR
   CLKPR = 0x00;   // x1 clock

   // UART
   //
   enum
      {
        BAUDRATE = 57600,
        F_IO_16 = F_IO/16,
        BAUD_DIVISOR = (F_IO_16 / BAUDRATE) - 1
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
   PORTB = pullup_B;
   PORTD = pullup_D;
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
uint32_t
sleep_ms(uint32_t milli_secs)
{
  enum {
          // CTC frequencies after prescaler
          //
          PRE_1 = 1,   // prescaler: ÷1
          PRE_2 = 5,   // prescaler: ÷1024

          F_CTC_1   =  F_IO,          // 4 MHz,   Tmax = 16.384 msec
          F_CTC_2   =  F_IO / 1024,   // 3906 Hz, Tmax = 16.777 sec

          WGmode    = 4,              // alias CTC mode, p. 113
          WGmode_A  = (WGmode & 0x03) << WGM10,
          WGmode_B  = (WGmode >> 2)   << WGM12,
        };
   // timer in CRC mode with 20 ms interval
   //
   // WGM13..10 is 0100 p. 113, (split between TCCR1B and TCCR1A)
   TCCR1A = 0 << COM1A0   // normal mode, OC1A disconnected, see p. 111
          | 0 << COM1B0   // normal mode, OC1B disconnected, see p. 111
          | WGmode_A      // WGM11:WGM10 = 00  p. 111
          ;

   if (milli_secs < 16)   // short sleep
      {
        TCCR1B = WGmode_B        // WGM13:WGM12 = 01  p. 113
               | PRE_1 << CS10   // prescaler: io-clk
               ;

        OCR1A = F_CTC_1 * milli_secs / 1000;
      }
   else
      {
        if (milli_secs > 16000)   // very long sleep
           {
             milli_secs = 16000;
           }

        TCCR1B = WGmode_B        // WGM13:WGM12 = 01  p. 113
               | PRE_2 << CS10   // prescaler: io-clk
               ;

        OCR1A = F_CTC_2 * milli_secs / 1000;
      }

   TCNT1 = 0;
   TIFR  = 1 << OCIE1A;   // clear old interrupts
   TIMSK = 1 << OCIE1A;   // enable interrupts

   set_sleep_mode(SLEEP_MODE_IDLE);

   sleep_enable();
   sei();
   sleep_cpu();
   cli();
   sleep_disable();

   TIMSK = 0;             // disable timer interrupts
   return milli_secs;
}
//-----------------------------------------------------------------------------
inline void
beep(uint16_t ms)
{
   set_pin(D, BEEPER);
   sleep_ms(ms);
   clr_pin(D, BEEPER);
}
//-----------------------------------------------------------------------------
static void
print_char(uint8_t ch)
{
   while (!(UCSRA & 1 << UDRE))   ;   // wait for DR empty
   UDR = ch;
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
static void
print_hex4(uint16_t ch)
{
   print_hex2(ch >> 8);
   print_hex2(ch);
}
//-----------------------------------------------------------------------------
static void
print_dec(uint16_t val)
{
bool subseq = false;
uint8_t cc = val / 10000;
   if (cc)   { print_char('0' + cc);   subseq = true; }

   cc = val / 1000 % 10;
   if (cc || subseq)   { print_char('0' + cc);   subseq = true; }

   cc = val / 100 % 10;
   if (cc || subseq)   { print_char('0' + cc);   subseq = true; }

   cc = val / 10 % 10;
   if (cc || subseq)   { print_char('0' + cc);   subseq = true; }

   print_char('0' + val % 10);
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

void
battery_test()
{
   ACSR = 0 << ACD     // DO NOT disable comparator
        | 1 << ACBG    // DO     use internal bandgap reference
        | 0 << ACI     // DO NOT clear interrupt
        | 0 << ACIE    // DO NOT enable interrupt
        | 0 << ACIC    // DO NOT enable input capture
        | 2 << ACIS0   // interrupt on falling edge
        ;
   sleep_ms(1);         // wait 1 ms for bandgap reference to start up

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
   _delay_ms(1);          // DO NOT sleep_ms() !
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
   sleep_ms(10);
   set_pin(B, BTEST_OUT);      // drive AIN0 high
   sleep_ms(10);
   clr_pin(B, BTEST_OUT);      // drive AIN0 low
   sleep_ms(10);
   set_pin(B, BTEST_OUT);      // drive AIN0 high
   sleep_ms(30);
   clr_pin(B, BTEST_OUT);      // drive AIN0 low
}
//-----------------------------------------------------------------------------
//
// one pass, return the number of ms to sleep after this pass
uint16_t
doit(uint16_t j, uint8_t initial_calib)
{
   OSCCAL = j & 0x7F;

   battery_test();

   print_string("OSCCAL=");
   print_hex2(OSCCAL);
   print_string(", initial OSCCAL=");
   print_hex2(initial_calib);
   print_string(" j=");
   print_hex4(j);
   print_string(", battery=");
   print_dec(batt_result);
   print_char('\n');

   if (j & 1)   set_pin(D, LED_RED);
   else         clr_pin(D, LED_RED);

   return 100;
}
//-----------------------------------------------------------------------------
int
main(int, char *[])
{
   init_hardware();

const uint8_t initial_calib = eeprom_read_byte(0);
   if (initial_calib != 0xFF)
      {
        OSCCAL = initial_calib;   // explicit calibration
        sleep_ms(500);
        print_string("\n\neeprom calibration byte: 0x");
        print_hex2(initial_calib);
        print_string("...\n\n");
      }


// OSCCAL = 0x47;   // good for 4 MHz and 9600 baud
   OSCCAL = 0x3B;   // good for 4 MHz and 57600 baud

   for (uint16_t j = 0; j < 10; ++j)
       {
         clr_pin(B, LED_GREEN);
         set_pin(D, LED_RED);
         sleep_ms(500);
         clr_pin(D, LED_RED);
         set_pin(B, LED_GREEN);
         sleep_ms(300);
       }

   for (;;)
   for (uint16_t j = 0; j < 128; ++j)
       {
//       dump_registers();
         const uint16_t wait = doit(j, initial_calib);
         sleep_ms(wait);
       }
}
//-----------------------------------------------------------------------------
