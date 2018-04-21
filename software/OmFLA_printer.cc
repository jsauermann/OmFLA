#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

using namespace std;

#define PRINTF_FLAGS  "#0- +'I"
#define PRINTF_DIGITS ".0123456789"
#define PRINTF_CONV   "cdiouxXn%"

//-----------------------------------------------------------------------------
const struct _format
{
   const char * src_file;
   int src_line;
   int format_id;
   int arg_count;
   const char * format;
} formats[] =
{
#define m(file, line, fmt, acnt, str) { #file, line, fmt, acnt, str },
#include "string_table.incl"
};
enum { FORMATS = sizeof(formats) / sizeof(*formats) };

//-----------------------------------------------------------------------------
int
read_byte(int fd)
{
char cc;
const ssize_t len = read(fd, &cc, 1);
   if (len != 1)   exit(0);
   return cc & 0xFF;
}
//-----------------------------------------------------------------------------
int
read_word(int fd)
{
char ch;
unsigned char cl;
ssize_t len = read(fd, &ch, 1);   if (len != 1)   exit(0);
        len = read(fd, &cl, 1);   if (len != 1)   exit(0);

   if (ch & 0x80)   return ~0xFFFF | ch << 8 | cl;
   else             return           ch << 8 | cl;
}
//-----------------------------------------------------------------------------
void
read_chars(int fd)
{
   for (;;)
      {
        const int fmt = read_byte(fd);
        if (fmt == 0)   continue;   // sync char
        if (fmt >= FORMATS)   continue;

        const _format & format = formats[fmt];
        if   (format.arg_count == 0)
           {
              printf("%s", format.format);
           }
        else if (format.arg_count == 1)
           {
             const int val1 = read_word(fd);
              printf(format.format, val1);
           }
        else if (format.arg_count == 2)
           {
             const int val1 = read_word(fd);
             const int val2 = read_word(fd);
              printf(format.format, val1, val2);
           }
        else assert(0 && "bad format argument count");
      }
}
//-----------------------------------------------------------------------------
int
check_format(int f)
{
const _format & fmt = formats[f];
int ret = 0;
const char * str = fmt.format;

   while (const char cc = *str++ & 0xFF)
       {
         if (cc != '%')   continue;
         while (strchr(PRINTF_FLAGS,  *str))   ++str;   // skip flags
         while (strchr(PRINTF_DIGITS, *str))   ++str;   // skip field width
         if (strchr(PRINTF_CONV, *str))
            {
               if (*str++ != '%')   ++ret;              // skip conversion
            }
         else
            {
              cerr << "*** Bad/unsuported conversion '" << *str++
                   << "' in format string \"" << fmt.format << "\"" << endl;
            }
       }

   if (ret != fmt.arg_count)
      cerr << "argument count mismatch in format #" << f << "." << endl
           << "    format string:         \"" << fmt.format << "\"" << endl
           << "     args in format string: " << ret  << endl
           << "     formats["<< f << "].arg_count:  " << formats[f].arg_count
           << endl;

   return ret;
}
//-----------------------------------------------------------------------------
int
main(int, char *[])
{
int total_chars = 0;
int values = 0;

   // check format strings...
   //
   for (int f = 0; f < FORMATS; ++f)
       {
          total_chars += strlen(formats[f].format);
          values += check_format(f);
       }

   cout << FORMATS << " formats,"    << endl
        << total_chars    << " characters," << endl
        << values    << " values."     << endl;

#define tty_name "/dev/ttyAMA0"
const int fd = open(tty_name, O_RDONLY);
   if (fd == -1)
      {
         perror("open( " tty_name " ) failed");
         return 1;
      }

termios tio;
   if (tcgetattr(fd, &tio))
      {
         perror("tcgetattr failed");
         return 2;
      }

   cfmakeraw(&tio);

   if (cfsetspeed(&tio, 57600))
      {
         perror("cfsetspeed(57600) failed");
         return 3;
      }

   if (tcsetattr(fd, TCSAFLUSH, &tio))
      {
         perror("tcsetattr failed");
         return 2;
      }

   read_chars(fd);
   return 0;
}
//-----------------------------------------------------------------------------
