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
 function for serial communication with either

 * a TCM 310 radio module, or
 * debug output on J15 pin 1

 In both cases the baudrate is 57600. For debug output a software UART is
 sufficient because only TxD is needed. For the TCM 310 radio module a hardware
 UART is needed because the TCM 310 reports its enocean ID via the RxD of the
 UART, which is almost impossible for a software UART at 57600 baud.
 */

# define SOFT_PORT_1 PORTB
# define SOFT_PBIT_1 B_PROG_MISO
# define SOFT_PORT_2 PORTD
# define SOFT_PBIT_2 D_TCM_TxD

enum CPU_cycles
   {
     DELAY_LOOP_LEN = 3,   // one iteration of _delay_loop_1()
     DELAY_PREAMBLE = 13,   // set/clear bit etc
                                                         // 9600   57600
     CYCLES_PER_BIT = F_CPU / BAUDRATE,                  //  384      64
     DELAY_PER_BIT  = CYCLES_PER_BIT - DELAY_PREAMBLE,   //  371      51
     SOFT_COUNT     = DELAY_PER_BIT/DELAY_LOOP_LEN,      //  123      17
   };

static uint8_t soft_count = SOFT_COUNT;

static void
print_byte(uint8_t ch)
{
   // update CRC if needed
   //
   enum { polynom = 0x07 };   // (x^8) + x^2 + x^1 + x^0
   crc ^= ch;
   for (uint8_t i = 0; i < 8; i++)
       {
         const bool crc_high = crc & 0x80;
         crc <<= 1;
         if (crc_high)   crc ^= polynom;
       }

   // transmit the character
   //
   while (!(UCSRA & 1 << UDRE))   ;   // wait for DR empty
   UDR = ch;
}

static void
print_char(char ch)
{
   //
   //             ╔╦═════════════════  stop bit(s)
   //             ║║     ╔╦══════════  8 data bits
   //             ║║     ║║     ╔════  start bit
int16_t bits = (0xFF00 | ch) << 1;

   for (uint8_t j = 0; j < 12; ++j)   // 1 start _ 8 data + 3 stop
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
inline void
init_uart()
{
   // UART
   //
   enum
      {
        F_IO_16 = F_IO/16,
        BAUD_DIVISOR = (F_IO_16 / BAUDRATE) - 1
      };

   UBRRH = BAUD_DIVISOR >> 8;
   UBRRL = BAUD_DIVISOR & 0xFF;

   disable_enocean();   // empty unless ENOCEAN is #defined as well
   UCSRB = 0 << RXEN | 1 << TXEN;
   UCSRC = 1 << USBS | 3 << UCSZ0;   // async, 2 stop, 8 data
}



