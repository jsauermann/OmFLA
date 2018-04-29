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
 functions for accessing a Texas Instruments TRF7970A via its SPI bus
 */

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
const uint8_t fifo[1 + MAX_FIFO] = { READ | CONT | FIFO, 0 };
uint8_t rx_data[sizeof(fifo)];
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
setup_RFID_reader()
{
   set_pin(B, RFID_EN);
   sleep_ms(10);
   init_RFID_reader();

   SPI_transfer(setup, 0, sizeof(setup));
   sleep_ms(10);   // > 6 ms
}
//-----------------------------------------------------------------------------
static void
RF_Off()
{
   write_register(CHIP_STATE_CONTROL, CHIP_STATE_RF_Off);
}
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

