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

/*! @file Bilinear debayering to BGR format on host. */
/* Helper functions */
#include "DebayerBilinear.h"

#ifndef TARGET_TYPE_RASPI_CAM
/* use to replace asm file , just an empty implementation (apparently not used) */
OSC_ERR DebayerBilinearBGR(uint8 *pDst, 
			   uint8 *pSrc, 
			   uint32 width, 
			   uint32 height, 
			   uint8 *pTmp, 
			   enum EnBayerOrder enBayerOrder)
{
  
  return SUCCESS;
}

#endif /* #ifdef TARGET_TYPE_RASPI_CAM */

