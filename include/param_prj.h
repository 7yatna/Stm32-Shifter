/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */

//Define a version string of your firmware here
#define VER 1.00AK

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 33
//Next value Id: 2028
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
	PARAM_ENTRY(CAT_IO,    	   	Mode,       OPMODES,    0,      1,      0,   1) \
	PARAM_ENTRY(CAT_IO,        	M1_LOCK,    OFFON,     	0,      1,      0,   2) \
	PARAM_ENTRY(CAT_IO,        	Brake_IN,   OFFON,     	0,      1,      0,   3) \
	PARAM_ENTRY(CAT_IO,        	CanCtrl,    OFFON,     	0,      1,      0,   4) \
	PARAM_ENTRY(CAT_IO,         NodeId,     "",     	1,      63,     5,   5) \
	VALUE_ENTRY(MODE,        	OPMODES,	2000 )\
	VALUE_ENTRY(Brake,			OFFON, 		2001 )\
	VALUE_ENTRY(GEAR, 	   	   	GEARS,   	2002 )\
	VALUE_ENTRY(VOLTAGE, 	   	"V",		2003 )\
	VALUE_ENTRY(KNOB_LOCK, 	   	OFFON,		2004 )\
	VALUE_ENTRY(Angle, 	   		"Degree",	2005 )\
	VALUE_ENTRY(Lock1, 	   	   	"",			2006 )\
	VALUE_ENTRY(Lock2, 	   	   	"",			2007 )\
	VALUE_ENTRY(version,       	VERSTR,		2008 )\
    VALUE_ENTRY(CPU_LOAD,       "%", 		2009 )


/***** Enum String definitions *****/
#define OPMODES      "0=OFF, 1=RUN"
#define GEARS        "0=PARK, 1=REVERSE, 2=NEUTRAL, 3=DRIVE, 4=ERROR"
#define OFFON        "0=OFF, 1=ON"
#define CAT_IO   	 "Digital I/O Control"

#define VERSTR STRINGIFY(4=VER)

/***** enums ******/




enum _modes
{
    MOD_OFF = 0,
    MOD_RUN,
    MOD_LAST
};

//Generated enum-string for possible errors
extern const char* errorListString;
