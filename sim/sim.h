/*	Oscar, a hardware abstraction framework for the LeanXcam and IndXcam.
	Copyright (C) 2008 Supercomputing Systems AG
	
	This library is free software; you can redistribute it and/or modify it
	under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation; either version 2.1 of the License, or (at
	your option) any later version.
	
	This library is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
	General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*! @file
 * @brief Private simulation module definition
 * 
 */
#ifndef SIM_PRIV_H_
#define SIM_PRIV_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "oscar.h"

/*! @brief Prefix of the test image file names.
 * 
 * The test image file names are generated by concatenating this prefix,
 * a number corresponding to the timestep and TEST_IMG_SUFFIX. */
#define TEST_IMG_PREFIX "testdata/Bogen001_"
/*! @brief The digits in the test image sequence numbers.
 * (Is it Img00001.bmp or Img001.bmp?) */
#define TEST_IMG_SEQ_NR_DIGITS 5
/*! @brief Suffix (image type) of the test image file names */
#define TEST_IMG_SUFFIX ".bmp"
/*! @brief Maximal number of callback functions allowed to register */
#define MAX_NUMBER_CALLBACK_FUNCTION 100


/*! @brief The object struct of the simulation module */
struct OSC_SIM_OBJ
{
	/*! @brief The current timestep, i.e. image analysed */
	uint32          curTimeStep;
	/*! @brief List with Callback functions to call every cycle */
	uint16 numCycleCallback;
#if defined(OSC_HOST) || defined(OSC_SIM)
	void (*aryCycleCallback[ MAX_NUMBER_CALLBACK_FUNCTION])(void);
#endif /*OSC_HOST*/
};

/*======================= Private methods ==============================*/


#endif /*SIM_PRIV_H_*/
