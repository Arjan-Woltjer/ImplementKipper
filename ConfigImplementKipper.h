/*
  Config file for ImplementKipper
Copyright (C) 2011-2014 J.A. Woltjer.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ConfigImplementKipper_h
#define ConfigImplementKipper_h

#define VOORSERIE
//#define DEBUG

#ifndef VOORSERIE
// Digital inputs 12V -> 5V conversion
//#define xxx               A2    // (input 1 connector)
//#define xxx               A3    // (input 2 connector)

// Digital outputs
#define OUTPUT_LED          17

// FET outputs (PWM)
#define OUTPUT_WIDE         9
#define OUTPUT_NARROW       10
#define OUTPUT_BYPASS       11
//#define xxx               12

// Analog input
#define ANGLE_SENS_PIN      A0    // for potmeter input (input 1 connector)
#define STEER_SENS_PIN      A1    // for potmeter input (input 2 connector)


#else
// Digital inputs 12V -> 5V conversion
//#define                   10    // (input 1 connector)
//#define xxx               A3    // (input 2 connector)

// Digital outputs
#define OUTPUT_LED          13

// FET outputs (PWM)
#define OUTPUT_WIDE         A1
#define OUTPUT_NARROW       A2
#define OUTPUT_BYPASS       A0
//#define xxx               12

// Analog input
#define ANGLE_SENS_PIN      A3    // for potmeter input (input 1 connector)
#define STEER_SENS_PIN      A4    // for potmeter input (input 2 connector)

#endif

#define SHUTOFF             3000 // 3 seconds

#endif
