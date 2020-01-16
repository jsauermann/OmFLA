
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#include <iostream>
#include <iomanip>

extern "C" {
#include <i2c/smbus.h>
}

using namespace std;

int fixed_contrast = 0;

//-----------------------------------------------------------------------------
/// broadcom pin numbers (GPIO) or PWM channels
enum Brodcom_pin
{
   PWM_0  = 0,
   PWM_1  = 1,
   BCM_7  = 7,
   BCM_8  = 8,
   BCM_9  = 9,    // pushbutton
   BCM_11 = 11,   // display: nReset
   BCM_16 = 16,   // display: E
   BCM_17 = 17,   // relay 1 (near R8 and RJ10 connectors)
   BCM_20 = 20,   // display: Rd/nW
   BCM_21 = 21,   // display: RS
   BCM_24 = 24,
   BCM_25 = 25,
   BCM_27 = 27,   // relay 2 (near R7 and power connectors)
};
//-----------------------------------------------------------------------------
enum Pin_Type
{
  PT_OUTPUT_0 = 0,   // output. initially low
  PT_OUTPUT_1 = 1,   // output. initially high
  PT_INPUT    = 2,   // input
};
//-----------------------------------------------------------------------------
class GPIO_pin
{
public:
   GPIO_pin(Brodcom_pin num, Pin_Type pt);
   ~GPIO_pin()
      { if (fd != -1)   close(fd); }

   int write(int on_off)   { return ::write(fd, on_off ? "1" : "0", 1); }
   int read() const
       {
         lseek(fd, 0, SEEK_SET);
         char cc = 255;
         if (1 != ::read(fd, &cc, 1))   perror("*** read()");
         return cc == '1';
       }

   int fd;
};
//-----------------------------------------------------------------------------
GPIO_pin::GPIO_pin(Brodcom_pin num, Pin_Type pt)
   : fd(-1)
{
char filename[40];
char value[10];

   // check if the pin is already exported (in that case the file
   // /sys/class/gpio/gpio%u/direction exists.
   //
   snprintf(filename, sizeof(filename),
            "/sys/class/gpio/gpio%u/direction", num);
   if (access(filename, F_OK))   // file does not exist (= is not exported)
      {
        char expo[40];
        snprintf(expo, sizeof(expo), "/sys/class/gpio/export");
        const int expo_fd = open(expo, O_WRONLY);
        if (expo_fd == -1)   perror("*** open export file");
        const int len = snprintf(value, sizeof(value), "%u\n", num);
        if (len != ::write(expo_fd, value, len))   perror("write(gpio/export)");
        close(expo_fd);
      }

   // at this point the direction file should exist. Set the direction
   //
int flags = 0;
   {
     const int dir_fd = open(filename, O_WRONLY);
     if (dir_fd == -1)   perror("open(gpio/direction)");
     if (pt == PT_INPUT)
        {
          flags = O_RDONLY;
          if (2 != ::write(dir_fd, "in", 2))   perror("write(gpio/direction)");
        }
     else
        {
          flags = O_WRONLY;
          if (3 != ::write(dir_fd, "out", 3))   perror("write(gpio/direction)");
        }
     close(dir_fd);
   }

   // if the pin is an output: set it low.
   //
   snprintf(filename, sizeof(filename),
            "/sys/class/gpio/gpio%u/value", num);
   fd = open(filename, flags);

   if      (pt == PT_OUTPUT_0)   write(false);
   else if (pt == PT_OUTPUT_1)   write(true);
}
//=============================================================================
class PWM_pin
{
public:
   PWM_pin(Brodcom_pin chan);

   int write_period(int len);
   int write_duty_cycle(int len);
   int write_enable(bool yes);

protected:
   const Brodcom_pin channel;
};
//-----------------------------------------------------------------------------
PWM_pin::PWM_pin(Brodcom_pin chan)
   : channel(chan)
{
char filename[40];
char value[10];

   // check if the pin is already exported (in that case the file
   // /sys/class/pwm/pwmchip0/pwm%u/enable exists.
   //
   snprintf(filename, sizeof(filename),
            "/sys/class/pwm/pwmchip0/pwm%u/period", chan);
   if (access(filename, F_OK))   // file does not exist (= is not exported)
      {
        char expo[40];
        snprintf(expo, sizeof(expo), "/sys/class/pwm/pwmchip0/export");
        const int expo_fd = open(expo, O_WRONLY);
        if (expo_fd == -1)   perror("*** open export file");
        const int len = snprintf(value, sizeof(value), "%u\n", chan);
        if (len != ::write(expo_fd, value, len))
           perror("write(pwm/pwmchip0/export)");
        close(expo_fd);
      }

   // at this point the period file should exist. Set the period
   //
   write_period(100000);
   write_duty_cycle(50000);
   write_enable(true);
}
//-----------------------------------------------------------------------------
int
PWM_pin::write_period(int total_len)
{
char filename[40];
char value[40];

   snprintf(filename, sizeof(filename),
            "/sys/class/pwm/pwmchip0/pwm%u/period", channel);
const int period_fd = open(filename, O_WRONLY);
const int len = snprintf(value, sizeof(value), "%u", total_len);   // 10 kHz
   if (len != ::write(period_fd, value, len))
   perror("write(pwm/pwmchip0/pwm0/period)");
   close(period_fd);
   return len;
}
//-----------------------------------------------------------------------------
int
PWM_pin::write_duty_cycle(int on_len)
{
char filename[40];
char value[40];

   snprintf(filename, sizeof(filename),
            "/sys/class/pwm/pwmchip0/pwm%u/duty_cycle", channel);

const int duty_fd = open(filename, O_WRONLY);
   const int len = snprintf(value, sizeof(value), "%u", on_len);   // 50%
   if (len != ::write(duty_fd, value, len))
      perror("write(pwm/pwmchip0/pwmN/duty_cycle)");

   close(duty_fd);
   return len;
}
//-----------------------------------------------------------------------------
int
PWM_pin::write_enable(bool yes)
{
char filename[40];

   snprintf(filename, sizeof(filename),
            "/sys/class/pwm/pwmchip0/pwm%u/enable", channel);
const int enable_fd = open(filename, O_WRONLY);
const char cc = yes ? '1' : '0';
   if (1 != ::write(enable_fd, &cc, 1))
      perror("write(pwm/pwmchip0/pwm0/enable)");

   close(enable_fd);
   return 1;
}
//-----------------------------------------------------------------------------
enum { LOW = 0, HIGH = 1 };
class Dog_display
{
public:
   Dog_display()
   : Data_D7(BCM_24, PT_OUTPUT_0),
     Data_D6(BCM_25, PT_OUTPUT_0),
     Data_D5(BCM_8,  PT_OUTPUT_0),
     Data_D4(BCM_7,  PT_OUTPUT_0),
     Enable (BCM_16, PT_OUTPUT_0),
     Rd_NWr (BCM_20, PT_OUTPUT_0),   // stays low
     Reg_Sel(BCM_21, PT_OUTPUT_0),
     Reset  (BCM_11, PT_OUTPUT_0)
   {}

   void reset(bool on_off)   { Reset.write(on_off); }

   /// write byte to instruction register
   void write_IR(int data)
      {
        Rd_NWr.write(LOW);       // WR = 0
        Reg_Sel.write(LOW);      // RS = 0, page 17
        write_byte(data);
      }

   /// write nibble to instruction register
   void write_IR7654(int data)
      {
        Rd_NWr.write(LOW);       // WR = 0
        Reg_Sel.write(LOW);      // RS = 0, page 17
        write_7654(data);
      }

   /// write to data register
   void write_DR(int data)
      {
        Rd_NWr.write(LOW);       // WR = 0
        Reg_Sel.write(HIGH);     // RS = 1, page 17
        write_byte(data);
      }

protected:
   /// set pins D7-D4 to bits 7-4 of data
   void write_7654(int data)
      {
        Enable.write(HIGH);      // de-assert E (just in case)
        Data_D7.write(data & 0x80);
        Data_D6.write(data & 0x40);
        Data_D5.write(data & 0x20);
        Data_D4.write(data & 0x10);
        Enable.write(LOW);       // assert E
        Enable.write(HIGH);      // de-assert E
      }

   void write_byte(int data)
      {
        // precondition: Rd_NWr low
        write_7654(data);        // set upper nibble
        write_7654(data << 4);   // set lower nibble
      }

   GPIO_pin Data_D7;     // D7
   GPIO_pin Data_D6;     // D6
   GPIO_pin Data_D5;     // D5
   GPIO_pin Data_D4;     // D4
   GPIO_pin Enable;      // E, active high
   GPIO_pin Rd_NWr;      // Read, active high / Write, active low
   GPIO_pin Reg_Sel;     // Register Select
   GPIO_pin Reset;       // RESET, active low
};
//-----------------------------------------------------------------------------
bool
wait_or_stop(int count, GPIO_pin & button)
{
    for (int t = 0; t < count; ++t)
        {
          if (const bool pushed = !button.read())
             {
               cerr << "button was pushed" << endl;
               while (!button.read())    usleep(100000);
               return true;
             }

          usleep(100000);
        }

   return false;   // timeout
}
//-----------------------------------------------------------------------------
void
display_test(Dog_display & display, GPIO_pin & button)
{
   cerr << "display test..." << endl;
#define ms *1000
#define us

   // pages 42 ff.
   //
   // Most likely the instruction set(s) in "extension mode" page 27
   // are being used.
   //
   // the code below follows page 41 (4-bit interface, fosc=380 KHz)...
   //
   display.reset(LOW);    usleep(  1 ms);   // assert RESET
   display.reset(HIGH);   usleep( 40 ms);   // de-assert RESET

   display.write_IR7654(0x30);    usleep( 2 ms);   // FUNCTION SET (DL=8-bit)
   display.write_IR7654(0x30);    usleep(30 us);   // FUNCTION SET (DL=8-bit)
   display.write_IR7654(0x30);    usleep(30 us);   // FUNCTION SET (DL=8-bit)
   display.write_IR7654(0x20);    usleep(30 us);   // FUNCTION SET (DL=4-bit)
   //
   // the last write above was in 8-bit mode and has supposedly switched
   // the interface to 4-bit mode.

   display.write_IR(0x29);    usleep(30 us);   // FUNCTION SET N=1 DH=0 IS=01
   //             0010 1001
   //             ││││ ││││
   //             ││││ ││└┴── IS=01 instruction table 1
   //             ││││ │└──── DH=0   5x8 font
   //             ││││ └───── N=1    2-line display
   //             │││└─────── DL=0   4-bit interface
   //             └┴┴──────── 001:   FUNCTION SET

   display.write_IR(0x15);    usleep(30 us);
   //             0001 1101
   //             ││││ ││││
   //             ││││ │││└── FX=1
   //             ││││ ││└─── 0
   //             ││││ │└──── 1
   //             ││││ └───── BS=1
   //             └┴┴┴─────── BIAS SET

   display.write_IR(0x78);    usleep(30 us);   // contrast          C3210=1000
   //             0111 1000
   //             ││││ ││││
   //             ││││ │││└── C0=0
   //             ││││ ││└─── C1=0
   //             ││││ │└──── C2=0
   //             ││││ └───── C3=1
   //             └┴┴┴─────── CONTRAST SET

   display.write_IR(0x5E);    usleep(30 us);   // power Ion=1 Bon=1 C54=01
   //             0101 1110
   //             ││││ ││││
   //             ││││ │││└── C4=0  contrast
   //             ││││ ││└─── C5=1  contrast
   //             ││││ │└──── Bon=1 booster ON
   //             ││││ └───── Ion=1 icon display
   //             └┴┴┴─────── 0101: FOLLOWER MODE

   display.write_IR(0x6A);    usleep(30 us);   // follower Fon=1 Rab=010
   //             0110 1010
   //             ││││ ││││
   //             ││││ │└┴┴── Rab=010 amplifier ratio
   //             ││││ └───── F=1     follower ON
   //             └┴┴┴─────── 0110: FOLLOWER CONTROL

   display.write_IR(0x0C);    usleep(30 us);
   //             0000 1100
   //             ││││ ││││
   //             ││││ │││└── B=1  cursor blink ON
   //             ││││ ││└─── C=1  cursor ON
   //             ││││ │└──── D=1  display ON
   //             └┴┴┴─┴───── 00001: DISPLAY ON/OFF

   display.write_IR(0x01);    usleep( 2 ms);
   //             0000 0001
   //             ││││ ││││
   //             └┴┴┴─┴┴┴┴── 00000001: CLEAR DISPLAY

   display.write_IR(0x06);    usleep(30 us);
   //             0000 0110
   //             ││││ ││││
   //             ││││ │││└── S=0   do not shift
   //             ││││ ││└─── I/D=1 increment
   //             └┴┴┴─┴┴──── ENTRY MODE SET


   // say Hello...
   //
   for (const char * h = "Hello"; *h; ++h)
       {
         display.write_DR(*h);
         usleep(30 us);
       }

   // display.write_IR(0x);    usleep(30 us);
   //             XXXX XXXX
   //             ││││ ││││
   //             ││││ │││└── 
   //             ││││ ││└─── 
   //             ││││ │└──── 
   //             ││││ └───── 
   //             │││└─────── 
   //             ││└──────── 
   //             │└───────── 
   //             └────────── 

   if (fixed_contrast)
      {
        cerr << "fixed contrast: " << fixed_contrast << endl;
        display.write_IR(0x70 | fixed_contrast & 0x0F );    usleep(30 us);
        display.write_IR(0x5C | fixed_contrast >> 4);       usleep(30 us);
        return;
      }

   for (;;)
       {
         for (size_t c = 50; c < 64; ++c)
             {
               cerr << "Contrast: " << c << endl;
               display.write_IR(0x70 | c & 0x0F );    usleep(30 us);
               display.write_IR(0x5C | c >> 4);       usleep(30 us);

               display.write_IR(0x80 + 0x10);   usleep(30 us);   // AC=0x10;
               for (const char * txt = "Contrast: "; *txt; ++txt)
                   { display.write_DR(*txt);    usleep(30 us); }
               display.write_DR('0' + c/10);
               display.write_DR('0' + c%10);

               if (wait_or_stop(4, button))   return;
             }
       }
}
//-----------------------------------------------------------------------------
void
pushbutton_test(GPIO_pin & button)
{
   cerr << "pushbutton test (push the button 3 times)..." << endl;

bool state = false;   // not pushed
int edges = 0;

   for (;;)
       {
         usleep(100000);
         const bool pushed = !button.read();
         if (state == pushed)   continue;   // no change

         state = pushed;
         edges;
         cerr << "    button " << (pushed ? "pushed   (" : "released (")
              << (1 + edges/2) << ")" << endl;
         if (++edges == 6)   break;
       }

   cerr << "pushbutton test stopped" << endl;
}
//-----------------------------------------------------------------------------
void
pwm_test(PWM_pin & pwm_0, PWM_pin & pwm_1, GPIO_pin & button)
{
   cerr << "PWM test (push button to stop)..." << endl;
   for (;;)
       {
         for (int pw = 10000; pw <= 90000; pw += 10000)
             {
               pwm_0.write_duty_cycle(pw);
               pwm_1.write_duty_cycle(100000 - pw);
               if (wait_or_stop(50, button))
                  {
                    cerr << "PWM test stopped" << endl;
                    return;
                  }
             }
       }
}
//-----------------------------------------------------------------------------
void
relay_test(GPIO_pin & relay1, GPIO_pin & relay2, GPIO_pin & button)
{
   cerr << "Relay test (push button to stop)..." << endl;
   relay2.write(LOW);
   for (;;)
       {
         if (wait_or_stop(10, button))   break;
         cerr << "relay 1 OFF" << endl;
         relay1.write(LOW);
         usleep(1000000);

         if (wait_or_stop(10, button))   break;
         cerr << "relay 2 OFF" << endl;
         relay2.write(LOW);
         usleep(1000000);

         if (wait_or_stop(10, button))   break;
         cerr << "relay 1 ON" << endl;
         relay1.write(HIGH);
         usleep(1000000);

         if (wait_or_stop(10, button))   break;
         cerr << "relay 2 ON" << endl;
         relay2.write(HIGH);
         usleep(1000000);
       }

   relay1.write(LOW);
   relay2.write(LOW);
   cerr << "Relay test stopped" << endl;
}
//-----------------------------------------------------------------------------

int
main(int argc, const char * argv[])
{
const char * all[] = { argv[0], "0", "1", "2", "3", };
enum { allc = sizeof(all) / sizeof(*all) };

   if (argc == 2 && strtol(argv[1], 0, 10) > 10)
      {
        fixed_contrast =  strtol(argv[1], 0, 10);
        argv = all;
        argc = 2;
      }

   if (argc < 2)   // no arguments
      {
        argv = all;
        argc = allc;
      }

   for (int a = 1; a < argc; ++a)
       {
         const int test_num =  strtol(argv[a], 0, 10);
         if (test_num == 0)   // display test
            {
              Dog_display display;
              GPIO_pin pin_9(BCM_9,   PT_INPUT);      // pushbutton
              display_test(display, pin_9);
            }
         else if (test_num == 1)   // pushbutton test
            {
              GPIO_pin pin_9(BCM_9,   PT_INPUT);      // pushbutton
              pushbutton_test(pin_9);
            }
         else if (test_num == 2)   // PWM test
            {
              PWM_pin pwm_0(PWM_0);
              PWM_pin pwm_1(PWM_1);
              GPIO_pin pin_9(BCM_9,   PT_INPUT);      // pushbutton
              pwm_test(pwm_0, pwm_1, pin_9);
            }
         else if (test_num == 3)   // Relay test
            {
              GPIO_pin pin_17(BCM_17, PT_OUTPUT_0);   // upper relay
              GPIO_pin pin_27(BCM_27, PT_OUTPUT_0);   // lower relay
              GPIO_pin pin_9(BCM_9,   PT_INPUT);      // pushbutton
              relay_test(pin_17, pin_27, pin_9);
            }
         else if (test_num == 4)   // i2c temperature sensor test
            {
              enum { address = 0x18 };   // i2c address

              const int fd = open("/dev/i2c-1", O_RDWR);
              if (fd == -1)   perror("open /dev/i2c-1");

              if (ioctl(fd, I2C_SLAVE, address) < 0)
                 perror("ioctl(I2C_SLAVE)");

              int vendor = i2c_smbus_read_word_data(fd, 6);
              if (vendor < 0)   perror("i2c_smbus_read_word_data(register 6)");
              vendor = (vendor >> 8) | (vendor << 8 & 0xFF00);
              cerr << "vendor = 0x" << hex << vendor << dec
                   << " (expect 0x54 for a MCP9808 sensor)" << endl;

              int value = i2c_smbus_read_word_data(fd, 5);
              if (value < 0)   perror("i2c_smbus_read_word_data(register 5)");
              value = (value >> 8) | (value << 8 & 0xFF00);

              cerr << "raw value = 0x" << hex << value << dec << endl;
              close(fd);

              if (value & 0x1000)   value |= ~0x0FFF;   // freezing
              else                  value &=  0x0FFF;

              cerr << "int value = 0x" << hex << value << dec << endl
                   << "temperature: " << (value/16.0)         << endl;
            }
         else
            {
               cerr                         << endl <<
"*** BAD TEST number: " << test_num         << endl <<
"Usage: " << argv[0] << " [ test]"          << endl <<
"    test 0: LCD display test"              << endl <<
"    test 1: pushbutton test"               << endl <<
"    test 2: PWM channels test"             << endl <<
"    test 3: relays test"                   << endl;
              return 1;
            }
       }

   return 0;
}
//-----------------------------------------------------------------------------

