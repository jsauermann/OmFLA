


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
 top-level functions for the OmFLA device
 */

// NOTE: ATtiny 4313 Fuse settings:
//
// Extended Fuse: 0xFF (default)
// High Fuse:     0xDF (default)
// Low Fuse:      set in Makefile and passed as -D $(FUSEL)
//
#if   FUSEL == 0xED
# define MAY_CALIBRATE 0   // CPU calibration has no effect (external Xtal used)
#elif FUSEL == 0xE2
# define MAY_CALIBRATE 1   // CPU calibration is possible (RC Oscillator used)
#else
# error "Bad/Unsupported Low Fuse Setting (expecting 0x6D or 0x62)"
#endif

#define F_CPU      3686400
#define  CPU_CALIB 0x3B
#define BAUDRATE   57600

#include "user_defined_parameters.hh"

enum EEPROM_addresses
{
   USER_PARAMS     = 0,      // copy of user_params (14 bytes)
   SENSOR_Cache    = 0x20,   // blocks 3..14 of the sensor (8 byte per block)

   SENSOR_Cache_3  = SENSOR_Cache,          // block 3:  0x20
   SENSOR_Cache_4  = SENSOR_Cache_3  + 8,   // block 4:  0x28
   SENSOR_Cache_5  = SENSOR_Cache_4  + 8,   // block 5:  0x30
   SENSOR_Cache_6  = SENSOR_Cache_5  + 8,   // block 6:  0x38
   SENSOR_Cache_7  = SENSOR_Cache_6  + 8,   // block 7:  0x40
   SENSOR_Cache_8  = SENSOR_Cache_7  + 8,   // block 8:  0x48
   SENSOR_Cache_9  = SENSOR_Cache_8  + 8,   // block 9:  0x50
   SENSOR_Cache_10 = SENSOR_Cache_9  + 8,   // block 10: 0x58
   SENSOR_Cache_11 = SENSOR_Cache_10 + 8,   // block 11: 0x60
   SENSOR_Cache_12 = SENSOR_Cache_11 + 8,   // block 12: 0x68
   SENSOR_Cache_13 = SENSOR_Cache_12 + 8,   // block 13: 0x70
   SENSOR_Cache_14 = SENSOR_Cache_13 + 8,   // block 14: 0x78
   E_pass              = 0x80,
   E_aver_2            = 0x81,
   E_initial_glucose_2 = 0x82,
   E_alarm_LOW__2      = 0x83,
   E_alarm_HIGH__2     = 0x84,
   GLUCO_trend_idx     = 0x85,
   GLUCO_hist_idx      = 0x86,

   GLUCO_trend         = 0xD0,
   GLUCO_history       = 0xE0,
};

static uint16_t cache = 0;
#define F_IO  F_CPU

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <string.h>
#include <util/delay.h>

uint16_t pass = 0;

/*
 delta_LOW resp. delta_HIGH are additional margins for the lower resp. upper
 alarm thresholds. If the OmFLA device is started with a moderate glucose
 level, i.e. a glucose level between (alarm_LOW + margin_LOW) and
 (alarm_HIGH - margin_HIGH), then delta_LOW = delta_HIGH = 0 and remain 0.
 However, if the OmFLA device is started with a glucose level close to
 alarm_LOW resp. alarm_HIGH, then the lower resp, upper alarm thresholds,
 then delta_LOW resp. delta_HIGH are temporarily set to a value > 0 (to avoid
 false alarms) and are decreased until a moderate glucose level is reached.
*/
static int8_t delta_LOW__2  = 0;   // additional margin for the LOW alarm
static int8_t delta_HIGH__2 = 0;   // additional margin for the HIGH alarm

/// a 1:1 copy of the EEPROM data, initialized at startup
User_defined_parameters user_params;

#ifndef __AVR_ATtiny4313__
# error "__AVR_ATtiny4313__ is not defined !!!"
#endif

static void print_char(char ch);

static void enable_enocean();
static void disable_enocean();
static void transmit_glucose(uint8_t gluco_2);
static void transmit_change_bitmap();
static void transmit_changed_values();
static uint8_t crc = 0;
static uint8_t changed_bitmap[13];   // blocks 3-15 incl.
static uint8_t changed_values[10];
static uint8_t changed_idx = 0;

#define get_pin(port, bit)    (PIN  ## port &    port ## _ ## bit ? 0xFF : 0x00)
#define set_pin(port, bit)    (PORT ## port |=   port ## _ ## bit)
#define clr_pin(port, bit)    (PORT ## port &= ~ port ## _ ## bit)
#define output_pin(port, bit) (DDR  ## port |=   port ## _ ## bit)
#define input_pin(port, bit)  (DDR  ## port &= ~ port ## _ ## bit)

static void
write_cache(uint8_t value)
{
const uint8_t old = eeprom_read_byte((const uint8_t *)cache);
   if (old != value)   // value change
      {
        const uint8_t changed_addr = cache - SENSOR_Cache;
        const uint8_t changed_byte = changed_addr >> 3;
        const uint8_t changed_bit = changed_addr & 7;
        changed_bitmap[changed_byte] |= (0x80 >> changed_bit);
        if (changed_idx < sizeof(changed_values))
           changed_values[changed_idx++] = value;
        eeprom_write_byte((uint8_t *)cache, value);
      }

   ++cache;
}

enum IO_pins
{
   __A_XTAL_1  = 1 << PA0,   // XTAL1 (not used)
   __A_XTAL_2  = 1 << PA1,   // XTAL2 (not used)
   __A_RESET   = 1 << PA2,   // RESET (NEVER drive it  high!)
   outputs_A   = __A_XTAL_1
               | __A_XTAL_2,
   pullup_A  = 0,

   B_BTEST_OUT = 1 << PB0,     // battery test output
   B_BTEST_IN  = 1 << PB1,     // battery test input
   B_MOSI      = 1 << PB2,     // MOSI to RFID reader
   B_TCM_POWER = 1 << PB3,     // Power for TCM 310 radio module
   B_LED_GREEN = 1 << PB4,     // green LED
   B_PROG_MOSI = 1 << PB5,     // MOSI from serial programmer
   B_PROG_MISO = 1 << PB6,     // MISO to   serial programmer (not used)
   B_PROG_SCL  = 1 << PB7,     // SCL  from serial programmer
   B_RFID_EN   = B_PROG_SCL,   // active high RFID enable output
   B_JUMPER    = B_PROG_MOSI,  // beep mute jumper

   outputs_B   = B_BTEST_OUT
               | B_MOSI
               | B_TCM_POWER
               | B_LED_GREEN
               | B_PROG_MISO  // soft UART TxD
               | B_RFID_EN,
   pullup_B = B_JUMPER,

   D_TCM_RxD   = 1 << PD0,   // RxD from TCM 310 radio module
   D_TCM_TxD   = 1 << PD1,   // TxD to   TCM 310 radio module
   D_SCLK      = 1 << PD2,   // Data clock to RFID reader
   D_SSEL      = 1 << PD3,   // Slave SELect to RFID reader
   D_LED_RED   = 1 << PD4,   // red LED
   D_MISO      = 1 << PD5,   // MISO from RFID reader
   D_BEEPER    = 1 << PD6,   // beeper
   outputs_D   = D_TCM_TxD
               | D_SCLK
               | D_SSEL
               | D_LED_RED
               | D_BEEPER,
   pullup_D  = 0,
};

enum BOARD_STATUS
{
   BSTAT_RESET         = 0,
   BSTAT_RFID_ERROR    = 3,
   BSTAT_RUNNING       = 7,
   BSTAT_ABOVE_INITIAL = 6,
   BSTAT_BELOW_INITIAL = 4,
};

#include "UART.cc"

static uint8_t  board_status = BSTAT_RESET;
static uint8_t  trend_idx = 0;
static uint8_t  hist_idx = 0;

static uint16_t batt_result = 0;
static uint8_t  initial_glucose_2 = 0;

//-----------------------------------------------------------------------------
/// wait for \b milli_secs ms, return time slept (which can be less than
/// the time requested if \b milli_secs is too large)
static uint32_t
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

        //      (max. <  64,000,000)
        OCR1A = (F_CTC_1 * milli_secs) / 1000;
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

        //      (max. <  64,000,000)
        OCR1A = (F_CTC_2 * milli_secs) / 1000;
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
#define MAX_FIFO 10
#include "RFID_functions.cc"

# include "enocean.cc"
//-----------------------------------------------------------------------------
static void
init_hardware()
{
   // set GPIO directions
   //
   DDRA = outputs_A;
   DDRB = outputs_B;
   DDRD = outputs_D;

   PORTA = pullup_A;
   PORTB = pullup_B | B_TCM_POWER;
   PORTD = pullup_D | D_BEEPER | D_SSEL;

   init_uart();

   // set analog comparator to inactive state

   ACSR = 1 << ACD     // DO     disable comparator
        | 0 << ACBG    // DO NOT use internal bandgap reference
        | 0 << ACI     // DO NOT clear interrupt
        | 0 << ACIE    // DO NOT enable interrupt
        | 0 << ACIC    // DO NOT enable input capture
        | 2 << ACIS0   // interrupt on falling edge
        ;

   DIDR = 1 << AIN0D    // disable AIN0 digital input
        | 1 << AIN1D;   // disable AIN1 digital input
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
print0(uint8_t format)
{
   print_char(format);
}
//-----------------------------------------------------------------------------
static void
print1(uint8_t format, uint16_t value)
 {
   print_char(format);
   print_char(value >> 8);
   print_char(value);
 }
//-----------------------------------------------------------------------------
static void
print2(uint8_t format, uint16_t value1, uint16_t value2)
 {
   print1(format, value1);
   print_char(value2 >> 8);
   print_char(value2);
 }
//-----------------------------------------------------------------------------
inline void
beep(uint8_t repeat, uint16_t ms_on, uint8_t ms_off)
{
   if ((PINB & B_JUMPER) == 0)   return;

//  print1(1, ms_on);
   for (int j = 0; j < repeat; ++j)
      {
        clr_pin(D, BEEPER);
        sleep_ms(ms_on);
        set_pin(D, BEEPER);
        sleep_ms(ms_off);
      }
}
//-----------------------------------------------------------------------------
static uint8_t gluco2_vec[15];   // glucose values (divided by 2!)
static uint8_t gluco_idx = 0;

/// store glucose (in mg% ÷ 2) in gluco2_vec
static void
gluco2(uint8_t rx_offset)
{
const uint16_t h = rx_data[rx_offset + 3] & 0x0F;   // upper 4 bits
const uint16_t l = rx_data[rx_offset + 2];          // lower 8 bits
const uint32_t raw_sensor = h << 8 | l;           // total 12 bits
const uint16_t glucose = user_params.sensor_offset
                       + ((raw_sensor * user_params.sensor_slope) / 1000);

    print2(2, raw_sensor, glucose);

   gluco2_vec[gluco_idx++] = glucose >> 1;
}
//-----------------------------------------------------------------------------
/// sort gluco2_vec (15 items)
void
gluco_sort()
{
   for (uint8_t base = 0; base < (sizeof(gluco2_vec) - 1); ++base)
       {
          uint8_t smallest = base;
          for (uint8_t j = base + 1; j < sizeof(gluco2_vec); ++j)
              {
                if (gluco2_vec[j] < gluco2_vec[smallest])   smallest = j;
              }

         // exchange gluco2_vec[base] and gluco2_vec[smallest]
         //
         const uint8_t base_val = gluco2_vec[base];
         gluco2_vec[base] = gluco2_vec[smallest];
         gluco2_vec[smallest] = base_val;
       }
}
//-----------------------------------------------------------------------------
static bool
decode_Block(uint8_t block)
{
const uint8_t FIFO_len = read_register(FIFO_STATUS) & 0x7F;
   if (FIFO_len > MAX_FIFO)
      {
         print2(3, FIFO_len, __LINE__);
        goto error_out;
      }

   SPI_transfer(fifo, rx_data, FIFO_len + 1);

   // rx_data[0] is the leading SPI dummy, and
   // rx_data[1] is the ISO error code if != 0
   //
   if (FIFO_len == 2 || rx_data[1] != 0)   // ISO error code
      {
         print2(4, rx_data[1], __LINE__);
        goto error_out;
      }

   if (FIFO_len != 9)
      {
         print2(5, FIFO_len, __LINE__);
        goto error_out;
      }

   // the trend table is contained in blocks 4...15
   //
   if (block < 3)     return false;   // OK
   if (block >= 16)   return false;   // OK

    print2(6, block, 8*block);

   for (uint8_t j = 2; j <= 9; ++j)
       {
          write_cache(rx_data[j]);
           print1(7, rx_data[11 - j]);
       }

   if (block == 3)
      {
        hist_idx  = rx_data[5];
        trend_idx = rx_data[4];
         print2(8, trend_idx, hist_idx);
        return false;   // OK
      }

   // every ISO block contains 8 bytes resp. 4 16-bit measurements. There are 3
   // kinds of measurements, say aaaa, GGGG, and bbbb. The GGGG measurements are
   // raw glocose values while the nature of of measurements aaaa and bbbb is
   // currently unknown (one of them is probably a temperature value).
   //
   switch(block % 3)
      {
        case 0:            // 0011-2233-4455-6677
                 print0(9);
                gluco2(4); //           ^^^^
                break;

        case 1:            // 0011-2233-4455-6677
                 print0(10);
                gluco2(2); //      ^^^^
                break;

        case 2:            // 0011-2233-4455-6677
                 print0(11);
                gluco2(6); //                ^^^^
                gluco2(0); // ^^^^
                break;
      }

   return false;   // OK

error_out:
   beep(3, 200, 100);

   if ((block % 3) == 1)   gluco2_vec[gluco_idx++] = 0;   // 2 values per block
   gluco2_vec[gluco_idx++] = 0;                       // 1 more gluco2_vec
    print1(12, block);
   return true;   // error
}
//-----------------------------------------------------------------------------
uint8_t iso_read_block[8] =
   {
     CMD_reset_FIFO,
     CMD_send_with_CRC,
     CONT | TX_LENGTH_BYTE_1,   // write continuous from 1D
     0, 0x30,   // length (3)
     ISO_FLAGS, // ISO15693 flags
     0x20,      // ISO Read Single Block command code
     0,         // block number
   };

static bool
read_Block(uint8_t block)
{
   // at this point we expect: FIFO empty and interrupts off.
   //
   {
     const uint16_t len = read_register(FIFO_STATUS);
     if (len)
        {
           print2(13, len, __LINE__);
          Reset_FIFO();
        }

     if (const uint16_t stat = read_ISR())
        {
           print2(14, stat, __LINE__);
        }
   }

   iso_read_block[sizeof(iso_read_block) - 1] = block;
   SPI_transfer(iso_read_block, 0, sizeof(iso_read_block));

   sleep_ms(30);

const uint8_t istat = read_ISR();
   if ((istat & 0xC0) != 0xC0)
      {
         print1(15, istat);
         print1(16, block);
        return true;
      }

   return false;
}
//-----------------------------------------------------------------------------
inline void
dump_registers()
{
const uint8_t which[32] =   // do not read ISR status (0x0C) or FIFO (0x1F)
   { 0x00,  0x01,  0x02,  0x03,  0x04,  0x05,  0x06,  0x07,
     0x08,  0x09,  0x0A,  0x0B,  0xFF,  0x0D,  0x0E,  0x0F,
     0x10,  0x11,  0x12,  0x13,  0x14,  0x15,  0x16,  0x17,
     0x18,  0x19,  0x1A,  0x1B,  0x1C,  0x1D,  0x1E,  0xFF };

int values[32];

   // read registers...
   //
   for (uint8_t w = 0; w < sizeof(which); ++w)
       {
         const Register reg = (Register)(which[w]);
         if (reg != 0xFF)   values[w] = read_register(reg);
         else               values[w] = -1;
       }

   // print registers...
   //
    print0(17);

   for (uint8_t w = 0; w < sizeof(which); ++w)
       {
         if ((w & 7) == 0)    print1(18, w);
         if      (values[w] == -1)     print0(19);
         else if (values[w] == -2)     print0(20);
         else
            {
               print1(21, values[w]);
            }
       }

    print0(22);
}
//-----------------------------------------------------------------------------
void
LED_01(uint8_t val)
{
   enum { LED_ON = 600, LED_OFF = 200 };   // LED timing (ms)

   if (val & 1)   // 1: blink green LED
      {
        set_pin(B, LED_GREEN);    // green LED on
        sleep_ms(LED_ON);
        clr_pin(B, LED_GREEN);   // green LED off
      }
   else           // 0: blink red LED
      {
        set_pin(D, LED_RED);      // red LED on
        sleep_ms(LED_ON);
        clr_pin(D, LED_RED);     // red LED off
      }

   sleep_ms(LED_OFF);
}
//-----------------------------------------------------------------------------
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
   _delay_ms(1);       // wait 1 ms for bandgap reference to start up

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

   // make B_BTEST_OUT an input with pullup. The pullup then charges
   // capacitor C6.
   //
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
}
//-----------------------------------------------------------------------------
static void
set_delta_LOW__2(uint8_t glucose2)
{
const uint8_t limit2 = user_params.alarm_LOW__2 + user_params.margin_LOW__2;
   if (glucose2 >= limit2)   delta_LOW__2 = 0;
   else                      delta_LOW__2 = limit2 - glucose2;
}
//-----------------------------------------------------------------------------
static void
set_delta_HIGH__2(uint8_t glucose2)
{
const uint8_t limit2 = user_params.alarm_HIGH__2 - user_params.margin_HIGH__2;
   if (glucose2 <= limit2)   delta_HIGH__2 = 0;
   else                      delta_HIGH__2 = glucose2 - limit2;
}
//-----------------------------------------------------------------------------
//
// one pass, return the number of ms to sleep after this pass
int32_t
doit()
{
   // blink according to board_status
   //
   LED_01(board_status >> 2);
   LED_01(board_status >> 1);
   LED_01(board_status);

   battery_test();

   // battery_test() sets batt_result to roughlu 800 (full battery) ...
   // 1200 (empty battery). To save bytes we scale batt_result down to uint8_t.
   if (pass == 0)   // power ON
      {
        const uint8_t br = batt_result >> 3;
        uint8_t battery_beeps = 5;

        if      (br >= user_params.battery_1__8)   battery_beeps = 1;
        else if (br >= user_params.battery_2__8)   battery_beeps = 2;
        else if (br >= user_params.battery_3__8)   battery_beeps = 3;
        else if (br >= user_params.battery_4__8)   battery_beeps = 4;

        beep(battery_beeps, 200, 200);
      }

    print2(23, pass, board_status);

    print2(24, batt_result, id2);
    print2(25, id3, id4);
    print1(26, id_valid);

   setup_RFID_reader();
   gluco_idx = 0;

   cache = SENSOR_Cache;
   memset(changed_bitmap, 0, sizeof(changed_bitmap));
   changed_idx = 0;

   for (uint8_t b = 3; b < 15; ++b)
       {
         if (read_Block(b) || decode_Block(b))
            {
               RF_Off();
               board_status = BSTAT_RFID_ERROR;
               enable_enocean();
               transmit_glucose(0);
               beep(3, 100, 100);
               return 8000*int32_t(user_params.read_error_retry__8);
            }

         read_ISR();   // clear interrupt register
       }
   RF_Off();
   clr_pin(B, RFID_EN);

   gluco_sort();

   /// compute the average of the 9 middle glucose values...
   //
   enum { end = sizeof(gluco2_vec) - 3 };

uint16_t aver_2 = 0;
   for (int8_t j = 3; j < end; ++j)  aver_2 += gluco2_vec[j];
   aver_2 /= (sizeof(gluco2_vec) - 6);

   write_cache(pass);
   write_cache(aver_2);
   write_cache(initial_glucose_2);
   write_cache(user_params.alarm_LOW__2);
   write_cache(user_params.alarm_HIGH__2);
   write_cache(trend_idx);
   write_cache(hist_idx);
   cache = GLUCO_trend + (trend_idx & 0x0F);
   write_cache(aver_2);
   cache = GLUCO_history + (hist_idx & 0x1F);
   write_cache(aver_2);

    print1(27, 2*aver_2);   // aver_2 is halved!

bool raise_alarm = false;
   if (initial_glucose_2 == 0)   // first glucose measurement
      {
        initial_glucose_2 = aver_2;
        board_status = BSTAT_RUNNING;
        set_delta_LOW__2(initial_glucose_2);
        set_delta_HIGH__2(initial_glucose_2);

         print2(28, 2*delta_LOW__2, 2*delta_HIGH__2);
      }
   else if (aver_2 >= initial_glucose_2)   // glucose has increased
      {
        // maybe increase the lower alarm threshold again
        //
        if (delta_LOW__2)
           {
             set_delta_LOW__2(aver_2);
              print1(29, 2*delta_LOW__2);
           }

        board_status = BSTAT_ABOVE_INITIAL;
        const uint8_t threshold = user_params.alarm_HIGH__2 + delta_HIGH__2;
        if (aver_2 > threshold)
           {
             set_pin(D, LED_RED);      // red LED on
             raise_alarm = true;
           }
      }
   else   // glucose has decreased
      {
        // maybe decrease the upper alarm thresholds again
        //
        if (delta_HIGH__2)
           {
             set_delta_HIGH__2(aver_2);
              print1(30, 2*delta_HIGH__2);
           }

        board_status = BSTAT_BELOW_INITIAL;
        const uint8_t threshold = user_params.alarm_LOW__2 - delta_LOW__2;
        if (aver_2 < threshold)
           {
             set_pin(B, LED_GREEN);    // green LED on
             raise_alarm = true;
           }
      }

   // transmit_glucose() also transmits board_status, so that we must not
   // call it before board_status was updated
   //
   enable_enocean();
   transmit_change_bitmap();
   transmit_changed_values();
   transmit_glucose(aver_2);

   if (raise_alarm)   // glucose is too low or too high
      {
         beep(172, 500, 200);   // beep for ~ 2 minutes
         return 2000;           // then wait 2 seconds
      }

   return 8000*int32_t(user_params.read_interval__8);
}
//-----------------------------------------------------------------------------
int
main(int, char *[])
{
   // read user definable parameters from EEPROM
   {
     uint8_t * up = (uint8_t *)&user_params;
     for (uint16_t e = USER_PARAMS; e < sizeof(user_params); ++e)
         up[e] = eeprom_read_byte((const uint8_t *)e);
   }

   init_hardware();
   sleep_ms(50);

#if MAY_CALIBRATE
const uint8_t calib = user_params.oscillator_calibration;
   if (calib != 0xFF)   OSCCAL = calib;        // calibration override
   else                 OSCCAL = CPU_CALIB;
#endif

#if MAY_CALIBRATE
    print2(31, OSCCAL, CLKPR);
#else
    print1(32, CLKPR);
#endif
    print2(33, user_params.sensor_slope, user_params.sensor_offset);
    print2(34, user_params.alarm_HIGH__2  << 1, user_params.alarm_LOW__2   << 1);
    print2(35, user_params.margin_HIGH__2 << 1, user_params.margin_LOW__2  << 1);
    print2(36, user_params.battery_1__8 << 3, user_params.battery_2__8 << 3);
    print2(37, user_params.battery_3__8 << 3, user_params.battery_4__8 << 3);
    print1(38, user_params.battery_5__8 << 3);
    print2(39, user_params.read_error_retry__8 << 3, user_params.read_interval__8 << 3);

   // transmit a glucose value of 0 as a restart indication and to
   // inform receiver(s) about the battery status.
   //
   enable_enocean();
   transmit_glucose(0);

   for (pass = 0;; ++pass)
       {
//       dump_registers();
         int32_t wait = doit();
         while (wait > 0)   wait -= sleep_ms(wait);
       }
}
//-----------------------------------------------------------------------------
