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

#ifndef __USER_DEFINED_PARAMETERS_DEFINED__
#define __USER_DEFINED_PARAMETERS_DEFINED__

//=============================================================================
// ======== USER-CONFIGURABLE PARAMETERS ======================================
//=============================================================================

// NOTE: calibration of the CPU frequency is only needed if:
//
// 1. serial communication is used (i.e. for the debug print and enocean
//    targets), AND
//
// 2. the internal RC oscillator is used and its frequency is not close to
//    an integer multiple of 16 times the baudrate used (which is 57600 baud)
//
// Most ordinary mortals are expected to use the no-debug target (firmware file
// freestyle_nod.hex) and therefore do not need any CPU calibration.
//
// A more elegant approach is to use a 3,6864 MHz crystal instead of the
// internal 4 MHz RC oscillator. In that case the crystal frequency is exactly
// 4*16*57600 and no CPU calibration is needed. The downside is probably a
// slightly higher current consumption.
//
// In all cases, make sure that the macro F_CPU in every C++ file is set
// to the actual (crystal or calibrated) frequency of the CPU. All internal
// timings depend on that frequency. but only the baudrate of the serial
// communication needs a frequency within 2% of the nominal value, while
// the other timings are fine with a 10% deviation from the nominal frequency.

enum CPU_Calibration
{
   NO_CPU_CALIBRATION = 0xFF,   // no CPU calibration

// CPU_CALIBRATION    = NO_CPU_CALIBRATION   // good luck!
// CPU_CALIBRATION    = 0x46                 // PCB #1
   CPU_CALIBRATION    = 0x3F                 // PCB #2: Ralf
// CPU_CALIBRATION    = 0x40                 // PCB #3

// CPU_CALIBRATION    = 0x4C                 // PCB #4: 47-51 5V USB
                                             //         46-50 battery
};

/// alarm thresholds define at which glucose level an alarm is raised
enum Alarm_thresholds
{
   ALARM_HIGH   = 250,   // mg/dl, beep if glucose > ALARM_HIGH
   ALARM_LOW    =  66,   // mg/dl, beep if glucose < ALARM_LOW
   MARGIN_HIGH  =  40,   // mg/dl, minimum between initial and limit_HIGH
   MARGIN_LOW   =  20,   // mg/dl, minimum between initial and limit_LOW
};

/// how the raw 12-bit glucose sensor value translate to glucose in mg/dl
// the formula used is:
//
// glucose = SENSOR_OFFSET + SENSOR_SLOPE/1000 * raw_sensor_value
//
//
enum Sensor_Calibration
{
   SENSOR_SLOPE  = 130,   // uint8_t / 1000 = (0.13)
   SENSOR_OFFSET = 236    // int8_t         = -20
};

/// how battery levels map to beep counts (at power-on of the device)
enum Battery_levels
{
   BATTERY_1 = 1200,   // beep once (battery is low)
   BATTERY_2 = 1100,
   BATTERY_3 = 1000,
   BATTERY_4 =  900,
   BATTERY_5 =  800,   // beeep 5 times (battery is full)
};

/// measurement intervals (seconds)
enum
{
   READ_INTERVAL    = 300,   // read RFID every 5 minutes
   READ_ERROR_RETRY =  60,   // retry after 60 seconds
};

//=============================================================================
// ======== END OF USER-CONFIGURABLE PARAMETERS ===============================
//=============================================================================

#include <stdint.h>

struct User_defined_parameters
{
   // some parameters are shifted so that they fint into a uint8_t
   //
   uint8_t oscillator_calibration;   //
   uint8_t sensor_slope;             //
    int8_t sensor_offset;            //
   uint8_t alarm_HIGH__2;            // >> 1
   uint8_t alarm_LOW__2;             // >> 1
   uint8_t margin_HIGH__2;           // >> 1
   uint8_t margin_LOW__2;            // >> 1
   uint8_t battery_1__8;             // >> 3
   uint8_t battery_2__8;             // >> 3
   uint8_t battery_3__8;             // >> 3
   uint8_t battery_4__8;             // >> 3
   uint8_t battery_5__8;             // >> 3
   uint8_t read_error_retry__8;      // >> 3
   uint8_t read_interval__8;         // >> 3
};

/// integers > 255 shifted so that they fit into a uint8_t
enum
{
   ALARM_HIGH__2       =  ALARM_HIGH >> 1,
   ALARM_LOW__2        =  ALARM_LOW  >> 1,
   MARGIN_HIGH__2      = MARGIN_HIGH >> 1,
   MARGIN_LOW__2       = MARGIN_LOW  >> 1,
   BATTERY_1__8        = BATTERY_1   >> 3,
   BATTERY_2__8        = BATTERY_2   >> 3,
   BATTERY_3__8        = BATTERY_3   >> 3,
   BATTERY_4__8        = BATTERY_4   >> 3,
   BATTERY_5__8        = BATTERY_5   >> 3,
   READ_ERROR_RETRY__8 = READ_ERROR_RETRY >> 3,
   READ_INTERVAL__8    = READ_INTERVAL >> 3,
};

#endif // __USER_DEFINED_PARAMETERS_DEFINED__
