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
 simple formatting functions
 */

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
#if !ENOCEAN
            case 0x81: print_hex4(val);   break;
            case 0x90: print_dec(val);    break;
#endif
            default:   print_char(cc);
          }
}
//-----------------------------------------------------------------------------
static void
_error(const char * str, uint16_t line, int16_t val)
{
   _print_string(PSTR("*** "), -1);
   _print_string(str, -1);
   _print_string(PSTR(" at line "), -1);
   print_dec(line);
   if (val != -1)   _print_string(PSTR(" (\x80)"), val);
   print_char('\n');
}
//-----------------------------------------------------------------------------

