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

// This is a simple helper program for generating the EEPROM content according
// to user_defined_parameters.hh. Makefile builds and calls it automatically
// when user_defined_parameters.hh has been changed.
//
#include <stdio.h>
#include "user_defined_parameters.hh"

User_defined_parameters user_params;

int
main(int, char *[])
{
   user_params.oscillator_calibration = CPU_CALIBRATION;
   user_params.sensor_slope           = SENSOR_SLOPE;
   user_params.sensor_offset          = SENSOR_OFFSET;
   user_params.alarm_HIGH__2          = ALARM_HIGH__2;
   user_params.alarm_LOW__2           = ALARM_LOW__2;
   user_params.margin_HIGH__2         = MARGIN_HIGH__2;
   user_params.margin_LOW__2          = MARGIN_LOW__2;
   user_params.battery_1__8           = BATTERY_1__8;
   user_params.battery_2__8           = BATTERY_2__8;
   user_params.battery_3__8           = BATTERY_3__8;
   user_params.battery_4__8           = BATTERY_4__8;
   user_params.battery_5__8           = BATTERY_5__8;
   user_params.read_error_retry__8    = READ_ERROR_RETRY__8;
   user_params.read_interval__8       = READ_INTERVAL__8;

const uint8_t * p = (const uint8_t *)&user_params;
   for (int u = 0; u < sizeof(user_params); ++u) putchar(*p++);
}
