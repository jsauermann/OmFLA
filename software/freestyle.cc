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

#define LOUD

enum
{
   ABSOLUTE_HIGH   = 200,   // beep if glucose > ABSOLUTE_HIGH
   ABSOLUTE_LOW    =  80,   // beep if glucose < ABSOLUTE_LOW
   RELATIVE_HIGH   =  50,   // beep if glucose > initial + RELATIVE_HIGH
   RELATIVE_LOW    =  50,   // beep if glucose < initial - RELATIVE_LOW

   ABSOLUTE_HIGH_2 = ABSOLUTE_HIGH/2,
   ABSOLUTE_LOW_2  = ABSOLUTE_LOW/2,
   RELATIVE_HIGH_2 = RELATIVE_HIGH/2,
   RELATIVE_LOW_2  = RELATIVE_LOW/2,
};

#define SENSOR_SLOPE  (0.13)
#define SENSOR_OFFSET (-20)

enum
{
   __A_XTAL_1  = 1 << PA0,   // XTAL1 (not used)
   __A_XTAL_2  = 1 << PA1,   // XTAL2 (not used)
   __A_RESET   = 1 << PA2,   // RESET (not used)
   outputs_A   = __A_XTAL_1
               | __A_XTAL_2
               | __A_RESET,
   pullup_A  = 0,

   B_BTEST_OUT = 1 << PB0,   // battery test output
   B_BTEST_IN  = 1 << PB1,   // battery test input
   B_MOSI      = 1 << PB2,   // MOSI to RFID reader
   B_TCM_POWER = 1 << PB3,   // Power for TCM 310 radio module
   B_LED_GREEN = 1 << PB4,   // green LED
   B_PROG_MOSI = 1 << PB5,   // MOSI from serial programmer
   __PROG_MISO = 1 << PB6,   // MISO to   serial programmer (not used)
   __PROG_SCL  = 1 << PB7,   // SCL  from serial programmer (not used)
   outputs_B   = B_MOSI
               | B_TCM_POWER
               | B_LED_GREEN
               | B_PROG_MOSI
   ////        | __PROG_MISO connected to TxD
               | __PROG_SCL,
   pullup_B  = 0,

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
   pullup_D  = 0,
};

enum BOARD_STATUS
{
   BSTAT_RESET         = 0,
   BSTAT_RFID_ERROR    = 3,
   BSTAT_RUNNING       = 7,
   BSTAT_BELOW_INITIAL = 4,
   BSTAT_ABOVE_INITIAL = 6,
};
uint8_t board_status = BSTAT_RESET;

static uint16_t batt_result = 0;
static uint16_t initial_glucose_2 = 0;
static uint8_t initial_B = 0;

#define get_pin(port, bit)    (PIN  ## port &    port ## _ ## bit ? 0xFF : 0x00)
#define set_pin(port, bit)    (PORT ## port |=   port ## _ ## bit)
#define clr_pin(port, bit)    (PORT ## port &= ~ port ## _ ## bit)
#define output_pin(port, bit) (DDR  ## port |=   port ## _ ## bit)
#define input_pin(port, bit)  (DDR  ## port &= ~ port ## _ ## bit)

#define STR(x) #x
#define CATN(x, n) x STR(n)

//-----------------------------------------------------------------------------
static void
init_hardware()
{
   // set pins to a preliminary value (0), which will be overridden below
   //
   DDRA = outputs_A;
   DDRB = outputs_B & ~B_PROG_MOSI;   // make PROG_MOSI an input
   DDRD = outputs_D;

   PORTA = pullup_A;
   PORTB = pullup_B | B_PROG_MOSI;
   PORTD = pullup_D;

   // CPU clock prescaler: x1
   //
   CLKPR = 0x80;   // enable write to CLKPR
   CLKPR = 0x00;   // x1 clock

   // timer 1 off
   TCCR1A = 0;
   TCCR1B = 0;

   // UART
   //
   enum
      {
        BAUDRATE = 9600,
        F_IO_16 = F_IO/16,
        BAUD_DIVISOR = (F_IO_16 / BAUDRATE) - 1
      };

   UBRRH = BAUD_DIVISOR >> 8;
   UBRRL = BAUD_DIVISOR & 0xFF;

   UCSRB = 0 << RXEN | 1 << TXEN;
   UCSRC = 1 << USBS | 3 << UCSZ0;   // async, 2 stop, 8 data

   // set up pins again AFTER all alternate functions have been enabled...

   initial_B = PINB;
   PORTB = pullup_B;   // disable pullup on PROG_MOSI

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

   // initialize SPI interface to RFID reader
   //
   set_pin(D, SSEL);
   clr_pin(D, SCLK);
   clr_pin(B, MOSI);

   // enable watchdog, prescaler = 0, no interrupts
   //
// WDTCR = 1 << WDE;

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
/// wait for \b milli_secs ms, return time slept (which can be less than
/// requested if \b milli_secs is too large
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
static void
SPI_transfer(const uint8_t * tx, uint8_t * rx, uint8_t length)
{
   // this function is called with SSEL=1, SCLK=0, and MOSI=0, and shall
   // return in the same state

   // start (assert nSSEL)
   //
   clr_pin(D, SSEL);

   for (uint8_t byte = 0; byte < length; ++byte)
       {
         const uint8_t vtx = tx[byte];
         uint8_t vrx = 0;
         for (uint8_t bit = 0x80; bit; bit >>= 1)
             {
               if (vtx & bit)   set_pin(B, MOSI);
               else             clr_pin(B, MOSI);
               set_pin(D, SCLK);                // ↑
               _delay_ms(0.1);
               vrx |= get_pin(D, MISO) & bit;   // sample Rx data
               clr_pin(D,SCLK);                 // ↓
               _delay_ms(0.2);
             }
         if (rx)   rx[byte] = vrx;
         _delay_ms(0.5);
       }

   // start (deassert SSEL)
   //
   clr_pin(B, MOSI);
   set_pin(D, SSEL);
   sleep_ms(1);
}
//-----------------------------------------------------------------------------
enum   // Table 6-20
{
   CONT              = 0x20,         // continuous address mode
   READ              = 0x40,         // read bit (0 for write)
   CMD               = 0x80,         // command bit (0 for parameters
   CMD_idle          = CMD | 0x00,   // direct command: idle
   CMD_init          = CMD | 0x03,   // direct command: init chip
   CMD_reset_FIFO    = CMD | 0x0F,   // direct command: reset FIFO
   CMD_send_with_CRC = CMD | 0x11,   // direct command: send and append CRC
};

enum Register
{
  CHIP_STATE_CONTROL       = 0x00,
  ISO_CONTROL              = 0x01,
  ISO_14443B_OPTIONS       = 0x02,
  ISO_14443A_OPTIONS       = 0x03,
  TX_TIMER_EPC_HIGH        = 0x04,
  TX_TIMER_EPC_LOW         = 0x05,
  TX_PULSE_LENGTH_CONTROL  = 0x06,
  RX_NO_RESPONSE_WAIT_TIME = 0x07,
  RX_WAIT_TIME             = 0x08,
  MODULATOR_CONTROL        = 0x09,
  RX_SPECIAL_SETTINGS      = 0x0A,
  REGULATOR_CONTROL        = 0x0B,
  IRQ_STATUS               = 0x0C,
  IRQ_MASK                 = 0x0D,   // Collision Pos and Interrupt Mask
  COLLISION_POSITION       = 0x0E,
  RSSI_LEVELS              = 0x0F,
  SPECIAL_FUNCTION         = 0x10,
  RAM_START_ADDRESS        = 0x11,   // RAM is 6 bytes long (0x11 - 0x16)
  ADJUSTABLE_FIFO_LEVEL    = 0x14,
  NFCID                    = 0x17,
  NFC_TARGET_LEVEL         = 0x18,
  TEST_SETTINGS_1          = 0x1A,
  TEST_SETTINGS_2          = 0x1B,
  FIFO_STATUS              = 0x1C,
  TX_LENGTH_BYTE_1         = 0x1D,
  TX_LENGTH_BYTE_2         = 0x1E,
  FIFO                     = 0x1F,
};
//-----------------------------------------------------------------------------
enum Register_value
{
   // register values
   //
   CHIP_VCC_3V       = 0x00,
   CHIP_VCC_5V       = 0x01,
   CHIP_VCC          = CHIP_VCC_3V,

   OUT_POWER_FULL    = 0x00,
   OUT_POWER_HALF    = 0x10,
   OUT_POWER         = OUT_POWER_HALF,

   CHIP_STATE_RF_Off = CHIP_VCC | OUT_POWER,
   CHIP_STATE_RF_On  = CHIP_STATE_RF_Off | 0x20,  // full power

   ISO_MODE_low      = 0x00,   // low bit rate, one subcarrier, 1(4)
   ISO_MODE_high     = 0x02,   // high bit rate, one subcarrier, 1(4)
   ISO_FLAGS_low     = 0x00,   // low data rate
   ISO_FLAGS_high    = 0x02,   // high data rate

   ISO_MODE = ISO_MODE_high,
   ISO_FLAGS = ISO_FLAGS_high,
};
//-----------------------------------------------------------------------------
static void
DirectCommand(uint8_t command)
{
   SPI_transfer(&command, 0, 1);
}
//-----------------------------------------------------------------------------
static void
Reset_FIFO()
{
   DirectCommand(CMD_reset_FIFO);
}
//-----------------------------------------------------------------------------
static uint8_t
read_register(Register reg)
{
uint8_t cmd[] = { uint8_t(READ | reg), uint8_t(READ | reg) };
   SPI_transfer(cmd, cmd, sizeof(cmd));
   return cmd[1];
}
//-----------------------------------------------------------------------------
static void
write_register(Register reg, uint8_t value)
{
uint8_t cmd[] = { reg, value };
   SPI_transfer(cmd, 0, sizeof(cmd));
}
//-----------------------------------------------------------------------------
inline void
init_RFID_reader()
{
   DirectCommand(CMD_init);
   DirectCommand(CMD_idle);

   Reset_FIFO();

   write_register(CHIP_STATE_CONTROL, CHIP_STATE_RF_Off);

   write_register(ISO_CONTROL, 0x00);

   write_register(NFC_TARGET_LEVEL, 0x00);   // fix TI bug
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
   print_char(cc || subseq ? '0' + cc : ' ');
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
            case 0x81: print_hex4(val);   break;
            case 0x90: print_dec(val);    break;
            default:   print_char(cc);
          }
}
#define print_string(str)       _print_string(PSTR((str)), -1)
#define print_stringv(str, val) _print_string(PSTR((str)), val)

//-----------------------------------------------------------------------------
inline void
beep(uint8_t repeat, uint8_t ticks_on, uint8_t ticks_off)
{
#ifdef LOUD   // always beep
   print_stringv("beep \x90\n", ticks_on);
   for (int j = 0; j < repeat; ++j)
      {
        set_pin(D, BEEPER);
        sleep_ms(10 * ticks_on);
        clr_pin(D, BEEPER);
        sleep_ms(10 * ticks_off);
      }
#else         // jumper selects beep on/ogg
   if (initial_B & B_PROG_MOSI)
      {
        print_stringv("beep \x90\n", ticks_on);
      }
   else for (int j = 0; j < repeat; ++j)
      {
        set_pin(D, BEEPER);
        sleep_ms(10 * ticks_on);
        clr_pin(D, BEEPER);
        sleep_ms(10 * ticks_off);
      }
#endif
}
//-----------------------------------------------------------------------------
static void
_error(const char * str, uint16_t line, int16_t val)
{
   _print_string(PSTR("*** "));
   _print_string(str);
   _print_string(PSTR(" at line "));
   print_dec(line);
   if (val != -1)   _print_string(PSTR(" (\x80)"), val);
   print_char('\n');
}
#define error(str)         _error(PSTR(str), __LINE__, -1);
#define errorv(str, val)   _error(PSTR(str), __LINE__, val);

//-----------------------------------------------------------------------------
static uint8_t
read_ISR()
{
uint8_t cmd[] = { uint8_t(READ | CONT | 0x0C),
                  uint8_t(READ | 0x0D),   // quirk
                  0 };
   SPI_transfer(cmd, cmd, sizeof(cmd));
   return cmd[1];
}
//-----------------------------------------------------------------------------
static void
RF_Off()
{
   write_register(CHIP_STATE_CONTROL, CHIP_STATE_RF_Off);
}
//-----------------------------------------------------------------------------
#define MAX_FIFO 10

const uint8_t fifo[1 + MAX_FIFO] = { READ | CONT | FIFO, 0 };
uint8_t rx_data[sizeof(fifo)];

/// return glucose in mg% ÷ 2
static uint8_t
gluco(int rx_offs)
{
const int h = rx_data[rx_offs + 3];
const int l = rx_data[rx_offs + 2];
const uint16_t val = h << 8 | l;
const int glucose(SENSOR_OFFSET + (val & 0x0FFF)*SENSOR_SLOPE);
   return glucose >> 1;
}

int8_t gluco2_vec[16];   // glucose ÷ 2 values
uint8_t gluco_idx = 0;

void
gluco_sort()
{
   for (int8_t base = 0; base < 15; ++base)
       {
          uint8_t smallest = base;
          for (uint8_t j = base + 1; j < 16; ++j)
              {
                if (gluco2_vec[j] < gluco2_vec[smallest])   smallest = j;
              }

         const uint8_t base_val = gluco2_vec[base];
         gluco2_vec[base] = gluco2_vec[smallest];
         gluco2_vec[smallest] = base_val;
       }
}

static void
decode_Block(uint8_t block)
{
const uint8_t addr = 8*block;

const uint8_t FIFO_len = read_register(FIFO_STATUS) & 0x7F;
   if (FIFO_len > MAX_FIFO)
      {
        errorv(CATN("FIFO_len > ", MAX_FIFO), FIFO_len);
        goto error_out;
      }

   SPI_transfer(fifo, rx_data, FIFO_len + 1);

   // rx_data[0] is the leading SPI dummy, and
   // rx_data[1] is the ISO error code if != 0
   //
   if (FIFO_len == 2 || rx_data[1] != 0)   // ISO error code
      {
        errorv("ISO error ", rx_data[1]);
        goto error_out;
      }

   if (FIFO_len != 9)
      {
        errorv("bad FIFO length", FIFO_len);
        goto error_out;
      }

   // the trend table is contained in blocks 4...15
   //
   if (block < 4)     return;
   if (block >= 16)   return;

   print_stringv("blk \x90  [", block);
   print_stringv("\x90] ", addr);

   for (int j = 9; j >= 2; --j)   print_hex2(rx_data[j]);

   switch(block % 3)
      {
        case 1: // AAAA-BBBB-CCCC-AAAA
                print_string(" ABCA\n");
                gluco2_vec[gluco_idx++] = gluco(2);   // FIFO is mirrored!
                break;

        case 2: // CCCC-AAAA-BBBB-CCCC
                print_string(" CABC\n");
                gluco2_vec[gluco_idx++] = gluco(6);   // FIFO is mirrored!
                gluco2_vec[gluco_idx++] = gluco(0);   // FIFO is mirrored!
                break;

        case 0: // BBBB-CCCC-AAAA-BBBB
                print_string(" BCAB\n");
                gluco2_vec[gluco_idx++] = gluco(4);   // FIFO is mirrored!
      }

   return;

error_out:
   beep(3, 10, 5);

   if ((block % 3) == 1)   gluco2_vec[gluco_idx++] = 0;   // 2 values per block
   gluco2_vec[gluco_idx++] = 0;                       // 1 more gluco2_vec
   print_stringv("    failed block: #\x90", block);
}
//-----------------------------------------------------------------------------
const uint8_t setup[] =
{
    CONT | CHIP_STATE_CONTROL,
    CHIP_STATE_RF_On,   // chip status:
    ISO_MODE,
    0x00,        // ISO_14443B_OPTIONS
    0x00,        // ISO_14443A_OPTIONS
    0xC1,        // TX_TIMER_EPC_HIGH
    0xBB,        // TX_TIMER_EPC_LOW
    0x00,        // TX_PULSE_LENGTH_CONTROL
    0x30,        // RX_NO_RESPONSE_WAIT_TIME
    0x1F,        // RX_WAIT_TIME
    0x01,        // MODULATOR_CONTROL
    0x40,        // RX_SPECIAL_SETTINGS = 424-kHz subcarrier for ISO 15693
    0x03,        // REGULATOR_CONTROL
};
//-----------------------------------------------------------------------------
inline void
setup_chip()
{
   SPI_transfer(setup, 0, sizeof(setup));
   sleep_ms(10);   // > 6 ms
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

bool
read_Block(uint8_t block)
{
   // at this point we expect: FIFO empty and interrupts off.
   //
   {
     const uint8_t len = read_register(FIFO_STATUS);
     if (len)
        {
          errorv("FIFO len > 0", len);
          Reset_FIFO();
        }

     const uint8_t stat = read_ISR();
     if (stat)
        {
          errorv("IRQ_STATUS != 0", stat);
        }
   }

   iso_read_block[sizeof(iso_read_block) - 1] = block;
   SPI_transfer(iso_read_block, 0, sizeof(iso_read_block));

   sleep_ms(30);

const uint8_t istat = read_ISR();
   if ((istat & 0xC0) != 0xC0)
      {
        errorv("missing Rx or Tx Interrupt", istat);
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
   print_string("\n     TRF-7970 register dump:\n"
                   "-----+0-+1-+2-+3-+4-+5-+6-+7");

   for (uint8_t w = 0; w < sizeof(which); ++w)
       {
         if ((w & 7) == 0)
            {
              print_string("\nr");
               print_hex2(w);
               print_char(':');
            }
         if      (values[w] == -1)    print_string(" --");
         else if (values[w] == -2)    print_string(" ??");
         else
            {
              print_char(' ');
              print_hex2(values[w]);
            }
       }

   print_string("\n\n");
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
int32_t
doit(uint16_t j)
{
   // blink according to board_status
   //
   LED_01(board_status >> 2);
   LED_01(board_status >> 1);
   LED_01(board_status);

   battery_test();

   print_stringv("j=\x90", j);
   print_stringv(" iniB=\x80", initial_B);
   print_stringv(" status=\x80", board_status);
   print_stringv(" battery=\x90\n", batt_result);

   setup_chip();
   gluco_idx = 0;

bool errors;
   for (uint8_t b = 4; b < 16; ++b)
       {
         if ((errors = read_Block(b)))   break;
         decode_Block(b);
         read_ISR();   // clear interrupt register
       }
   RF_Off();

   if (errors)
      {
         board_status = BSTAT_RFID_ERROR;
         beep(3, 3, 3);
         return 60000;
      }

   gluco_sort();

   /// compute the average of the 10 middle glocose values...
   //
uint16_t aver_2 = 0;
   for (uint8_t j = 3; j < 13; ++j)   aver_2 += gluco2_vec[j];
   aver_2 /= 10;

   print_stringv("glucose: \x90\n", 2*aver_2);   // aver_2 is halved!

bool need_beep;
   if (initial_glucose_2 == 0)   // first glucose measurement
      {
        initial_glucose_2 = aver_2;
        board_status = BSTAT_RUNNING;
      }
   else if (aver_2 >= initial_glucose_2)   // glucose has increased
      {
        board_status = BSTAT_ABOVE_INITIAL;
        need_beep = aver_2 > ABSOLUTE_HIGH_2
                 || aver_2 > initial_glucose_2 + RELATIVE_HIGH_2;
        if (need_beep)
           {
             set_pin(D, LED_RED);      // red LED on
             beep(10, 50, 20);
           }
      }
   else   // glucose has decreased
      {
        need_beep = aver_2 < ABSOLUTE_LOW_2
                 || aver_2 < initial_glucose_2 - RELATIVE_LOW_2;
        if (need_beep)
           {
             set_pin(B, LED_GREEN);    // green LED on
             beep(10, 50, 20);
           }
        board_status = BSTAT_BELOW_INITIAL;
      }

   return 120000;
}
//-----------------------------------------------------------------------------
int
main(int, char *[])
{
   init_hardware();

   init_RFID_reader();

const uint8_t calib = eeprom_read_byte(0);
   if (calib != 0xFF)   OSCCAL = calib;   // explicit calibration
   else                 OSCCAL = 0x47;    // good for 4 MHz

   for (uint16_t j = 0;; ++j)
       {
//       dump_registers();
         int32_t wait = doit(j);
         while (wait > 0)   wait -= sleep_ms(wait);
       }
}
//-----------------------------------------------------------------------------
