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
 * @brief Camera module implementation for target
	
 * On the OSC-specific hardware featuring
 * a Micron MT9V032 CMOS image sensor.
 */

#include "cam.h"


#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifdef TARGET_TYPE_RASPI_CAM
	#include <linux/types.h>
	#include <linux/videodev2.h>

	/*! @brief buffer for grabber with v4l2 */
	struct v4l2_buffer grabBuf;

	OSC_ERR OscSetupV4l2();
	OSC_ERR OscStartStreamV4l2();
	OSC_ERR OscDeqBufV4l2(uint8* fb);
	OSC_ERR OscQueBufV4l2();
#endif

	OSC_ERR OscCamCreate();
	OSC_ERR OscCamDestroy();

	/*! @brief The module definition. */
	struct OscModule OscModule_cam = {
		.name = "cam",
		.create = OscCamCreate,
		.destroy = OscCamDestroy,
		.dependencies = {
			&OscModule_log,
			NULL // To end the flexible array.
		}
	};

	struct OSC_CAM cam; /*!< @brief The camera module singelton instance */

#ifdef TARGET_TYPE_RASPI_CAM
	/*! @brief Representation of a mt9v032 CMOS-Sensor register for
	 * simulation */
	struct CAM_REGISTER
	{
		uint16 addr; /*!< @brief The address of the register */
		uint16 value; /*!< @brief The value of the register */
		char name[50]; /*!< @brief String describing the register */
	};

	/*! @brief The default values for the mt9v032 CMOS-Sensor registers */
	const struct CAM_REGISTER default_reg_values[] =
	{
	{ 0x00, 0x1313, "Chip Version" },
	{ 0x01, 0x0001, "Column Start" },
	{ 0x02, 0x0004, "Row Start" },
	{ 0x03, 0x01e0, "Window Height" },
	{ 0x04, 0x02f0, "Window Width" },
	{ 0x05, 0x002b, "Horizontal Blanking" },
	{ 0x06, 0x002d, "Vertical Blanking" },
	{ 0x07, 0x0298, "Chip Control" },
	{ 0x08, 0x01bb, "Shutter Width 1" },
	{ 0x09, 0x01d9, "Shutter Width 2" },
	{ 0x0A, 0x0164, "Shutter Width Ctrl" },
	{ 0x0B, 0x05dc, "Total Shutter Width" },
	{ 0x0C, 0x0000, "Reset" },
	{ 0x0D, 0x0320, "Read Mode" },
	{ 0x0E, 0x0000, "Monitor Mode" },
	{ 0x0F, 0x0015, "Pixel Operation Mode" },
	{ 0x1B, 0x0000, "LED_OUT Ctrl" },
	{ 0x1C, 0x0002, "ADC Mode Control" },
	{ 0x20, 0x03d5, "Reserved" },
	{ 0x2C, 0x0004, "VREF_ADC Control" },
	{ 0x31, 0x001d, "V1" },
	{ 0x32, 0x0018, "V2" },
	{ 0x33, 0x0015, "V3" },
	{ 0x34, 0x0004, "V4" },
	{ 0x35, 0x0010, "Analog Gain (16-64)" },
	{ 0x36, 0x0040, "Max Analog Gain" },
	{ 0x42, 0x0002, "Frame Dark Average" },
	{ 0x46, 0x231d, "Dark Avg Thresholds" },
	{ 0x47, 0x8080, "BL Calib Control" },
	{ 0x48, 0x007f, "BL Calibration Value" },
	{ 0x4C, 0x0002, "BL Calib Step Size" },
	{ 0x70, 0x0014, "Row Noise Corr Ctrl 1" },
	{ 0x72, 0x002a, "Row Noise Constant" },
	{ 0x73, 0x02f7, "Row Noise Corr Ctrl 2" },
	{ 0x74, 0x0000, "Pixclk, FV, LV" },
	{ 0x7F, 0x0000, "Digital Test Pattern" },
	/*      {0x80, 0x00f4, "Tile Weight/Gain X0_Y0"}, No access in driver
		{0x81, 0x00f4, "Tile Weight/Gain X1_Y0"},
		{0x82, 0x00f4, "Tile Weight/Gain X2_Y0"},
		{0x83, 0x00f4, "Tile Weight/Gain X3_Y0"},
		{0x84, 0x00f4, "Tile Weight/Gain X4_Y0"},
		{0x85, 0x00f4, "Tile Weight/Gain X0_Y1"},
		{0x86, 0x00f4, "Tile Weight/Gain X1_Y1"},
		{0x87, 0x00f4, "Tile Weight/Gain X2_Y1"},
		{0x88, 0x00f4, "Tile Weight/Gain X3_Y1"},
		{0x89, 0x00f4, "Tile Weight/Gain X4_Y1"},
		{0x8A, 0x00f4, "Tile Weight/Gain X0_Y2"},
		{0x8B, 0x00f4, "Tile Weight/Gain X1_Y2"},
		{0x8C, 0x00f4, "Tile Weight/Gain X2_Y2"},
		{0x8D, 0x00f4, "Tile Weight/Gain X3_Y2"},
		{0x8E, 0x00f4, "Tile Weight/Gain X4_Y2"},
		{0x8F, 0x00f4, "Tile Weight/Gain X0_Y3"},
		{0x90, 0x00f4, "Tile Weight/Gain X1_Y3"},
		{0x91, 0x00f4, "Tile Weight/Gain X2_Y3"},
		{0x92, 0x00f4, "Tile Weight/Gain X3_Y3"},
		{0x93, 0x00f4, "Tile Weight/Gain X4_Y3"},
		{0x94, 0x00f4, "Tile Weight/Gain X0_Y4"},
		{0x95, 0x00f4, "Tile Weight/Gain X1_Y4"},
		{0x96, 0x00f4, "Tile Weight/Gain X3_Y4"},
		{0x98, 0x00f4, "Tile Weight/Gain X4_Y4"},
		{0x99, 0x0000, "Tile Coord. X 0/5"},
		{0x9A, 0x0096, "Tile Coord. X 1/5"},
		{0x9B, 0x012c, "Tile Coord. X 2/5"},
		{0x9C, 0x01c2, "Tile Coord. X 3/5"},
		{0x9D, 0x0258, "Tile Coord. X 4/5"},
		{0x9E, 0x02f0, "Tile Coord. X 5/5"},
		{0x9F, 0x0000, "Tile Coord. Y 0/5"},
		{0xA0, 0x0060, "Tile Coord. Y 1/5"},
		{0xA1, 0x00c0, "Tile Coord. Y 2/5"},
		{0xA2, 0x0120, "Tile Coord. Y 3/5"},
		{0xA3, 0x0180, "Tile Coord. Y 4/5"},
		{0xA4, 0x01e0, "Tile Coord. Y 5/5"},*/
	{ 0XA5, 0x003a, "AEC/AGC Desired Bin" },
	{ 0xA6, 0x0002, "AEC Update Frequency" },
	{ 0xA8, 0x0000, "AEC LPF" },
	{ 0xA9, 0x0002, "AGC Update Frequency" },
	{ 0xAB, 0x0002, "AGC LPF" },
	{ 0xAF, 0x0000, "AEC/AGC Enable" },
	{ 0xB0, 0xabe0, "AEC/AGC Pix Count" },
	/*      {0xB1, 0x0002, "LVDS Master Ctrl"},
		{0xB2, 0x0010, "LVDS Shift Clk Ctrl"},
		{0xB3, 0x0010, "LVDS Data Ctrl"}, */
	{ 0xB4, 0x0000, "Data Stream Latency" },
	/*      {0xB5, 0x0000, "LVDS Internal Sync"},
		{0xB6, 0x0000, "LVDS Payload Control"},
		{0xB7, 0x0000, "Stereoscop. Error Ctrl"},
		{0xB8, 0x0000, "Stereoscop. Error Flag"},
		{0xB9, 0x0000, "LVDS Data Output"}, */
	{ 0xBA, 0x0010, "AGC Gain Output" },
	{ 0XBB, 0x05dc, "AEC Gain Output" },
	{ 0xBC, 0x003f, "AGC/AEC Current Bin" },
	{ 0xBD, 0x01e0, "Maximum Shutter Width" },
	{ 0xBE, 0x0014, "AGC/AEC Bin Difference Threshold" },
	{ 0xBF, 0x0016, "Field Blank" },
	{ 0xC0, 0x000a, "Mon Mode Capture Ctrl" },
	{ 0xC1, 0x015f, "Temperature" },
	{ 0xC2, 0x0840, "Analog Controls" },
	{ 0xC3, 0x0000, "NTSC FV & LV Ctrl" },
	{ 0xC4, 0x4416, "NTSC Horiz Blank Ctrl" },
	{ 0xC5, 0x4421, "NTSC Vert Blank Ctrl" },
	{ 0xF0, 0x2100, "Bytewise Addr" },
	{ 0xFE, 0xbeef, "Register Lock" },
	{ 0xFF, 0x1313, "Chip Version" },
	};

	/*********************************************************************//*!
	 * @brief Host only: Initialize simulated camera registers with defaults
	 *//*********************************************************************/
	static void OscCamResetRegs()
	{
		int i;
		uint32 len;

		/* Number of entries in the default value array */
		len = sizeof(default_reg_values)/sizeof(struct CAM_REGISTER);
		if (len > NUM_CAM_REGS)
		{
			len = NUM_CAM_REGS;
		}

		for (i = 0; i < len; i++)
		{
			cam.regs[i].addr = default_reg_values[i].addr;
			cam.regs[i].value = default_reg_values[i].value;
		}
	}

	/*********************************************************************//*!
	 * @brief Host only: Find the register by the specified register address
	 *
	 * @param addr Address of the desired register
	 * @return Pointer to the structure describing the register or NULL
	 *//*********************************************************************/
	static struct reg_info * OscCamFindReg(const uint32 addr)
	{
		int i;
		for (i = 0; i < NUM_CAM_REGS; i++)
		{
			if (cam.regs[i].addr == addr)
			{
				return &cam.regs[i];
			}
		}
		return NULL;
	}

#endif /* TARGET_TYPE_RASPI_CAM */



OSC_ERR OscCamCreate()
{
	OSC_ERR err;
	uint16 dummy;
	
	cam = (struct OSC_CAM) { };
	
	err = SUCCESS;

#ifdef TARGET_TYPE_RASPI_CAM
	/* Initialize camera registers */
	OscCamResetRegs();

	/* Open the video device file.
		 * We will be communicating with the camera driver by issuing
		 * IOCTLs over this file descriptor. */
	cam.vidDev = open(VIDEO_DEVICE_FILE, O_RDWR);

	err |=  OscSetupV4l2();

#else
	/* Open the video device file.
	 * We will be communicating with the camera driver by issuing
	 * IOCTLs over this file descriptor. */
	cam.vidDev = open(VIDEO_DEVICE_FILE, 2, 0);
#endif /* TARGET_TYPE_RASPI_CAM */
	if(unlikely(cam.vidDev < 0))
	{
		printf("%s: Error: Unable to open video device file %s (%s).\n",
				__func__,
				VIDEO_DEVICE_FILE,
				strerror(errno));
		cam.vidDev = 0;
		return -ENO_VIDEO_DEVICE_FOUND;
	}
	
#ifdef TARGET_TYPE_LEANXCAM
	/* Disable LED_OUT on leanXcam so the GPIOs can function correctly.
	 * This output is or'ed with GPIO_OUT2_N. */
	err |=  OscCamSetRegisterValue(CAM_REG_LED_OUT_CONTROL, 0x01);
#endif /* TARGET_TYPE_LEANXCAM */

	/* Read the current camera register values and build a model of the
	 * current state from them. */
	err |= OscCamGetShutterWidth(&cam.curExpTime);
	err |= OscCamGetRegisterValue(CAM_REG_HORIZ_BLANK,
			&cam.curHorizBlank);
	/* Read back the area of interest to implicitely update
	 * cam.curCamRowClks. */
	err |= OscCamGetAreaOfInterest(&dummy, &dummy, &dummy, &dummy);
	if(err != SUCCESS)
	{
		printf("%s: ERROR: Unable to read current settings from "
				"camera!\n",
				__func__);
	}
	
	return SUCCESS;
}

OSC_ERR OscCamDestroy()
{
	close(cam.vidDev);

	return SUCCESS;
}

OSC_ERR OscCamSetFileNameReader(void* hReaderHandle)
{
	/* Stump implementation on target platform. */
	return SUCCESS;
}

OSC_ERR OscCamSetAreaOfInterest(const uint16 lowX,
								const uint16 lowY,
								const uint16 width,
								const uint16 height)
{
#ifdef TARGET_TYPE_RASPI_CAM

	OscLog(INFO,   "%s: currently 'OscCamSetAreaOfInterest()' option "
					"not supported by raspi-cam; (%d, %d, %d, %d, %d, %d)\n",
					__func__,
					width, height, lowX,lowY,
					MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT);
			return SUCCESS;
#endif
	int ret;
	uint16 w, h, x, y; /* effective width/height/lowX/lowY */
	struct capture_window capWinNotMirror;

	/* Input validation */
	if(width%2 != 0 ||
			lowX + width > MAX_IMAGE_WIDTH ||
			lowY + height > MAX_IMAGE_HEIGHT)
	{
		OscLog(ERROR,
				"%s: Invalid parameter (%dx%d at %d/%d). "
				"Must fit %dx%d and width must be even\n",
				__func__,
				width, height, lowX,lowY,
				MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT);
		return -EINVALID_PARAMETER;
	}
		
	/* Reset the window to the default. */
	if((width == 0) || (height == 0))
	{
		x = 0;
		y = 0;
		w = MAX_IMAGE_WIDTH;
		h = MAX_IMAGE_HEIGHT;
	} else {
		x = lowX;
		y = lowY;
		w = width;
		h = height;
	}
	
	capWinNotMirror.col_off = x;
	capWinNotMirror.row_off = y;
	capWinNotMirror.width   = w;
	capWinNotMirror.height  = h;
	
	/* Apply flip bits depending on the perspective setup */
	if( cam.flipHorizontal)
	{
		x = MAX_IMAGE_WIDTH - (x + w);
	}
	if( cam.flipVertical)
	{
		y = MAX_IMAGE_HEIGHT - (y + h);
	}
		
	cam.capWin.col_off = x;
	cam.capWin.row_off = y;
	cam.capWin.width   = w;
	cam.capWin.height  = h;
	
	/* Communicate the desired window to the driver.
	 * The driver does input validation */
	ret = ioctl(cam.vidDev, CAM_SWINDOW, &cam.capWin);
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to set capture window: \
				IOCTL failed with %d.\n",
				__func__, errno);
		
		/* An error has occured */
		if(errno == EINVAL) /* Invalid parameter */
		{
			OscLog(ERROR, "%s(%u, %u, %u, %u): Invalid parameter!\n",
					__func__, lowX, lowY, width, height);
			return -EINVALID_PARAMETER;
		}
		return -EDEVICE;
	}
	
	/* The row time of the camera consists of the pixel readout time
	 * for the width of the row plus the horizontal blanking. The time
	 * has a lower bound. */
	cam.curCamRowClks = cam.capWin.width + cam.curHorizBlank;
	if(cam.curCamRowClks < CAM_MIN_ROW_CLKS)
	{
		cam.curCamRowClks = CAM_MIN_ROW_CLKS;
	}
	
	OscLog(DEBUG, "%s: Area of interest set to %dx%d at %d/%d.\n",
			__func__,
			capWinNotMirror.width,
			capWinNotMirror.height,
			capWinNotMirror.col_off,
			capWinNotMirror.row_off);
	
	return SUCCESS;
}

OSC_ERR OscCamSetRegisterValue(const uint32 reg, const uint16 value)
{
#ifdef TARGET_TYPE_RASPI_CAM
	struct reg_info * pReg;

	pReg = OscCamFindReg(reg);
	if(pReg == NULL)
	{
		return -EINVALID_PARAMETER;
	}
	pReg->value = value;

	return SUCCESS;
#else
	int                 ret;
	struct reg_info     reg_info;
	
	reg_info.addr = reg;
	reg_info.value = value;

	/* Communicate the desired register value to the driver.
	 * No input validation is done to retain flexibility */
	ret = ioctl(cam.vidDev, CAM_SCAMREG, &reg_info);
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to set register 0x%x to 0x%x: \
				IOCTL failed with %d.\n",
				__func__, reg, value, errno);
		
		/* An error has occured */
		if(errno == -EINVAL) /* Invalid parameter */
		{
			OscLog(ERROR, "%s(%u, %u): Invalid parameter!\n",
					__func__, reg, value);
			return -EINVALID_PARAMETER;
		}
		return -EDEVICE;
	}
	return SUCCESS;
#endif /* TARGET_TYPE_RASPI_CAM */
}

OSC_ERR OscCamGetRegisterValue(const uint32 reg, uint16 *pResult)
{
#ifdef TARGET_TYPE_RASPI_CAM
	struct reg_info * pReg;

	if(pResult == NULL)
	{
		return -EINVALID_PARAMETER;
	}

	pReg = OscCamFindReg(reg);
	if(pReg == NULL)
	{
		return -EINVALID_PARAMETER;
	}
	*pResult = pReg->value;

	return SUCCESS;
#else
	int                 ret;
	struct  reg_info    reg_info;
	
	reg_info.addr = reg;
	reg_info.value = 0;
	
	/* Input validation */
	if(unlikely(pResult == NULL))
	{
		return -EINVALID_PARAMETER;
	}
	
	/* Communicate the desired register value to the driver.
	 * No input validation is done to retain flexibility */
	ret = ioctl(cam.vidDev, CAM_GCAMREG, &reg_info);
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to get register 0x%x: \
				IOCTL failed with %d.\n",
				__func__, reg, errno);
		
		/* An error has occured */
		if(errno == -EINVAL) /* Invalid parameter */
		{
			OscLog(ERROR, "%s(%u, 0x%x): Invalid parameter!\n",
					__func__, reg, pResult);
			return -EINVALID_PARAMETER;
		}
		return -EDEVICE;
	}
	*pResult = reg_info.value;

	return SUCCESS;
#endif /* TARGET_TYPE_RASPI_CAM */
}

OSC_ERR OscCamSetFrameBuffer(const uint8 fbID,
		const uint32 uSize,
		const void * pData,
		const int bCached)
{
	struct frame_buffer     fb;
	int                     ret;
	int                     i;
	
	/* Input validation */
	if(unlikely(fbID > MAX_NR_FRAME_BUFFERS ||
			(pData == NULL && uSize != 0)))
	{
		OscLog(ERROR,
				"%s(%d, %d, 0x%x, %d): \
				Invalid parameter.\n",
				__func__, fbID, uSize, pData, bCached);
		return -EINVALID_PARAMETER;
	}
	
	fb.id = fbID;
	fb.size = uSize;
	fb.data = (void *)pData;
	fb.flags = 0;
	if(bCached)
	{
		fb.flags = FB_FLAG_CACHED;
	}
	
	if(unlikely(pData == NULL))
	{
		OscLog(INFO, "%s: Deleting frame buffer number %d.\n",
				__func__, fbID);
		/* Check whether the deleted frame buffer belongs to a multi
		 * buffer. */
		for(i = 0; i < cam.multiBuffer.multiBufferDepth; i++)
		{
			if(cam.multiBuffer.fbIDs[i] == fbID)
			{
				OscLog(ERROR,
						"%s Deleting frame buffer %d being part of \
						a multi buffer!.\n",
						__func__, fbID);
				return -ECANNOT_DELETE;
			}
		}
	}
	
#ifdef TARGET_TYPE_RASPI_CAM
	struct v4l2_buffer buf;

	memset(&buf, 0, sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.index = fbID;
	buf.m.userptr = (unsigned long) pData;
	buf.length = uSize;

	ret = ioctl(cam.vidDev, VIDIOC_QBUF, &buf);

	cam.fbStat[fb.id] = STATUS_READY;
#else
	ret = ioctl(cam.vidDev, CAM_SFRAMEBUF, &fb);
#endif /* TARGET_TYPE_RASPI_CAM */
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to set framebuffer %d: \
				IOCTL failed with %d.\n",
				__func__, fbID, errno);

		switch(errno)
		{
		case EINVAL:  /* Invalid parameter */
			OscLog(ERROR, "%s(%u, %u, 0x%x, %d): Invalid parameter!\n",
					__func__, fbID, uSize, pData, bCached);
			return -EINVALID_PARAMETER;
			break;
		case EBUSY:
			OscLog(ERROR,
					"%s: Camera device is busy.\n",
					__func__);
			return -EDEVICE_BUSY;
			break;
		default:
			OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
						__func__, strerror(errno), errno);
			return -EDEVICE;
		}
	}
	
	/* Store the info for later */
	memcpy(&cam.fbufs[fb.id], &fb, sizeof(struct frame_buffer));
	
	return SUCCESS;
}

OSC_ERR OscCamSetupCapture(uint8 fbID)
{
	struct capture_param   cp;
	int                    ret;
	uint8                  fb;
	
	if(unlikely((fbID != OSC_CAM_MULTI_BUFFER) &&
			(fbID > MAX_NR_FRAME_BUFFERS)))
	{
		return -EINVALID_PARAMETER;
	}
	
	if(unlikely(cam.capWin.width == 0 || cam.capWin.height == 0))
	{
		OscLog(ERROR, "%s: No area of interest set!\n",
				__func__);
		return -ENO_AREA_OF_INTEREST_SET;
	}
	
	cp.trigger_mode = TRIGGER_MODE_EXTERNAL;
	
	/* If the caller is using automatic multibuffer management,
	 * get the correct frame buffer. */
	fb = fbID;
	if(fbID == OSC_CAM_MULTI_BUFFER)
	{
		/* Get the buffer ID to write to next */
		fb = OscCamMultiBufferGetCapBuf(&cam.multiBuffer);
	}
	cp.frame_buffer = fb;
	cp.window = cam.capWin;
	
	OscLog(DEBUG,
				"%s: Setting up capture of %ux%d picture " \
				"on frame buffer %d.\n",
				__func__, cam.capWin.width, cam.capWin.height, fb);

#ifdef TARGET_TYPE_RASPI_CAM
	if(cam.lastValidID == OSC_CAM_INVALID_BUFFER_ID)
	{
		/* first time the function is called; start streaming */
		ret = OscStartStreamV4l2();
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: Unable to start streaming: \
					IOCTL failed with %d.\n",
					__func__, errno);

			return -EDEVICE;
		}
		/* dequeue first buffer */
		ret = OscDeqBufV4l2(&fb);
	}
#else
	ret = ioctl(cam.vidDev, CAM_CCAPTURE, &cp);
#endif /*  TARGET_TYPE_RASPI_CAM */
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to set up capture on framebuffer %d: \
				IOCTL failed with %d.\n",
				__func__, fb, errno);
		
		switch(errno)
		{
		case EINVAL:  /* Invalid parameter */
			OscLog(ERROR, "%s(%u): Invalid parameter!\n",
					__func__, fbID);
			return -EINVALID_PARAMETER;
			break;
		case EBUSY:
			OscLog(ERROR,
					"%s: Camera device is busy.\n",
					__func__);
			return -EDEVICE_BUSY;
			break;
		default:
			OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
					__func__, strerror(errno), errno);
			return -EDEVICE;
		}
	}
	
	/* The operation was successful */
	
	if(fbID == OSC_CAM_MULTI_BUFFER)
	{
		/* Allow the multi buffer to update is status according to this
		 * successful capture. */
		OscCamMultiBufferCapture(&cam.multiBuffer);
	}
	return SUCCESS;
}

OSC_ERR OscCamCancelCapture()
{
#ifndef TARGET_TYPE_RASPI_CAM
	int ret;
	
	ret = ioctl(cam.vidDev, CAM_CABORTCAPT, 0);
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to cancel capture: \
				IOCTL failed with %d.\n",
				__func__, errno);

		/* An error has occured */
		if(errno == -ENOENT) /* Picture already finished */
		{
			OscLog(WARN, "%s: Cancel request when no picture \
					transfer to cancel.\n",
					__func__);
			return -ENOTHING_TO_ABORT;
		}
		return -EDEVICE;
	}
#endif /* TARGET_TYPE_RASPI_CAM */
	return SUCCESS;
}

OSC_ERR OscCamReadPicture(const uint8 fbID, 
		uint8 ** ppPic, 
		const uint16 maxAge,
		const uint16 timeout)
{
	struct sync_param   sp;
	int                 ret;
	OSC_ERR             err = SUCCESS;
	uint8               fb;
	
	/* If the caller is using automatic multibuffer management,
	 * get the correct frame buffer. */
	fb = fbID;
	if(fb == OSC_CAM_MULTI_BUFFER)
	{
		/* Get the correct buffer ID */
		fb = OscCamMultiBufferGetSyncBuf(&cam.multiBuffer);
		if(fb == OSC_CAM_INVALID_BUFFER_ID)
		{
			return -ENO_CAPTURE_STARTED;
		}
	}
		
	/* Input validation */
	if(unlikely(ppPic == NULL || fb > MAX_NR_FRAME_BUFFERS ||
			cam.fbufs[fb].data == NULL))
	{
		return -EINVALID_PARAMETER;
	}
	
	*ppPic = NULL;     /* Precaution */
	
	sp.frame = fb;
	sp.max_age = maxAge;
	sp.timeout = timeout;
	
	OscLog(DEBUG,
			"%s(%u, 0x%x, %u, %u): Syncing capture on frame buffer %d.\n",
			__func__, fbID, ppPic, maxAge, timeout, fb);

#ifdef TARGET_TYPE_RASPI_CAM
	/* queue buffer from last grab */
	ret = OscQueBufV4l2();
	/* dequeue a new buffer for processing */
	if(ret == 0) {
		OscDeqBufV4l2(&fb);
	}
#else
	ret = ioctl(cam.vidDev, CAM_CSYNC, &sp);
#endif
	if(ret < 0)
	{
		switch(errno)
		{
		case EINVAL: /* Invalid parameter */
			OscLog(ERROR, "%s(%u, 0x%x, %u, %u): Invalid parameter!\n",
					__func__, fb, ppPic, maxAge, timeout);
			return -EINVALID_PARAMETER;
			break;
		case EINTR: /* Interrupt */
			OscLog(WARN, "%s: Sync on frame buffer %d got interrupted!\n",
				__func__, fb);
			return -EINTERRUPTED;
			break;
		case EAGAIN: /* Timeout */
			OscLog(DEBUG, "%s: Sync on frame buffer %d timed out.\n",
					__func__, fb);
			return -ETIMEOUT;
			break;
		case ERANGE: /* Too old */
			OscLog(DEBUG,
					"%s: Sync on frame buffer %d returned too late.\n",
					__func__, fbID);
			err = -EPICTURE_TOO_OLD;
			break; /* User may still want picture */
		default:
			OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
					__func__, strerror(errno), errno);
			return -EDEVICE;
		}
	}
	
	*ppPic = cam.fbufs[fb].data;
	
#ifdef TARGET_TYPE_RASPI_CAM
#else
	/* Apply image correction */
	if( cam.pCallback)
	{
		err = (*cam.pCallback)(
				*ppPic,
				cam.capWin.col_off,
				cam.capWin.row_off,
				cam.capWin.width,
				cam.capWin.height);
	}
#endif /* TARGET_TYPE_RASPI_CAM */
	
	/* The operation was successful */
	if(fbID == OSC_CAM_MULTI_BUFFER)
	{
		/* Allow the multi buffer to update is status according to this
		 * successful read. */
		OscCamMultiBufferSync(&cam.multiBuffer);
	}
	return err;
}

OSC_ERR OscCamReadLatestPicture(uint8 ** ppPic)
{
#ifdef TARGET_TYPE_RASPI_CAM

#else
	struct image_info image_info;
	int ret;
	
	if(unlikely(ppPic == NULL))
	{
		return -EINVALID_PARAMETER;
	}
	
	image_info.window = cam.capWin;
	/* The driver will automatically find the last image with the
	 * specified size */
	ret = ioctl(cam.vidDev, CAM_GLASTFRAME, &image_info);
	if(unlikely(ret < 0))
	{
		OscLog(ERROR,
				"%s: Unable to get last frame: \
				IOCTL failed with %d.\n",
				__func__, errno);

		switch(errno)
		{
		case EINVAL:
			return -EINVALID_PARAMETER;
			break;
		case ENOENT:
			OscLog(ERROR, "%s: No matching picture found.\n",
					__func__);
			return -ENO_MATCHING_PICTURE;
			break;
		default:
			OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
					__func__, strerror(errno), errno);
			return -EIO;
		}
	}
	*ppPic = (uint8*)image_info.fbuf;
#endif /* TARGET_TYPE_RASPI_CAM */
	return SUCCESS;
}

OSC_ERR OscCamRegisterCorrectionCallback(
		int (*pCallback)(
				uint8 *pImg,
				const uint16 lowX,
				const uint16 lowY,
				const uint16 width,
				const uint16 height))
{
#ifdef TARGET_TYPE_RASPI_CAM

#else
	cam.pCallback = pCallback;
#endif /* TARGET_TYPE_RASPI_CAM */
	return SUCCESS;
}

#ifdef TARGET_TYPE_LEANXCAM

OSC_ERR OscCamConfigSensorLedOut(bool bSensorLedOut, bool bInvert)
{
	OSC_ERR err;
	
	if(bSensorLedOut)
	{
		if(bInvert)
		{
			/* Enable and invert the LED_OUT pin of the CMOS sensor. */
			err = OscCamSetRegisterValue(CAM_REG_LED_OUT_CONTROL, 0x02);
		} else {
			/* Enable the LED_OUT pin of the CMOS sensor. */
			err = OscCamSetRegisterValue(CAM_REG_LED_OUT_CONTROL, 0x00);
		}
	} else {
		/* Disable the LED_OUT pin of the CMOS sensor. */
		err = OscCamSetRegisterValue(CAM_REG_LED_OUT_CONTROL, 0x01);
	}
	return err;
}

#endif /* TARGET_TYPE_LEANXCAM */


#ifdef TARGET_TYPE_RASPI_CAM
	OSC_ERR OscSetupV4l2()
	{
		int ret, i;
		struct v4l2_format fmt;
		struct v4l2_fmtdesc fmtdesc;

		/* log the available video format */
		for (i = 0; ; i++) {
			fmtdesc.index = i;
			fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			ret = ioctl(cam.vidDev, VIDIOC_ENUM_FMT, &fmtdesc);
			if (ret < 0)
				break;

			/* log with WARN level so that it is always written */
			OscLog(INFO, "%s: format[%d]: %s.\n",
						__func__, fmtdesc.index, fmtdesc.description);
		}

		fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width       = MAX_IMAGE_WIDTH;
		fmt.fmt.pix.height      = MAX_IMAGE_HEIGHT;
#if COLOR_TYPE == 1
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
#endif
		fmt.fmt.pix.field       = V4L2_FIELD_ANY;

		/* set the desired format, everything is fix currently */
		ret = ioctl(cam.vidDev, VIDIOC_S_FMT, &fmt);
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: Unable to set capture window: \
					IOCTL failed with %d.\n",
					__func__, errno);

			/* An error has occured */
			if(errno == EINVAL) /* Invalid parameter */
			{
				OscLog(ERROR, "%s(%u, %u, %u, %u): Invalid parameter!\n",
						__func__, fmt.type, fmt.fmt.pix.width, fmt.fmt.pix.height,
						fmt.fmt.pix.pixelformat);
				return -EINVALID_PARAMETER;
			}
			return -EDEVICE;
		}


		/* set two video buffers */
		struct v4l2_requestbuffers req;

		memset(&req, 0, sizeof(req));

		/* reserve enough space for buffers; but not all are used */
		req.count  = MAX_NR_FRAME_BUFFERS;
		req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req.memory = V4L2_MEMORY_USERPTR;
		ret = ioctl(cam.vidDev, VIDIOC_REQBUFS, &req);
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: Unable to set capture buffers: \
					IOCTL failed with %d.\n",
					__func__, req.count, errno);

			/* An error has occured */
			if(errno == EINVAL) /* Invalid parameter */
			{
				OscLog(ERROR, "%s(%u, %u, %u): Invalid parameter!\n",
						__func__, req.count, req.type, req.memory);
				return -EINVALID_PARAMETER;
			}
			return -EDEVICE;
		}

		/* log the result */
		OscLog(DEBUG, "field order: ");
		switch (fmt.fmt.pix.field) {
			case V4L2_FIELD_NONE:
				OscLog(INFO, "progressive\n");
				break;
			case V4L2_FIELD_INTERLACED:
				OscLog(INFO, "interlaced\n");
				break;
			case V4L2_FIELD_SEQ_TB:
				OscLog(INFO, "sequential TB\n");
				break;
			case V4L2_FIELD_SEQ_BT:
				OscLog(INFO, "sequential BT\n");
				break;
			default:
				OscLog(INFO, "unknown field order");
				break;
		}
		OscLog(INFO, "size = %lu\n", fmt.fmt.pix.sizeimage);
		OscLog(INFO, "bytesperline = %lu\n", fmt.fmt.pix.bytesperline);
		OscLog(INFO, "pixelformat: ");
		switch (fmt.fmt.pix.pixelformat) {
			case V4L2_PIX_FMT_UYVY:
				OscLog(INFO, "UYVY\n");
				break;
			case V4L2_PIX_FMT_YUYV:
				OscLog(INFO, "YUYV\n");
				break;
			case V4L2_PIX_FMT_RGB565:
				OscLog(INFO, "RGB565\n");
				break;
			case V4L2_PIX_FMT_RGB444:
				OscLog(INFO, "RGB444\n");
				break;
			default:
				OscLog(INFO, "unknown pixel format\n");
				break;
		}

		cam.capWin.col_off = 0;
		cam.capWin.row_off = 0;
		cam.capWin.width   = fmt.fmt.pix.width;
		cam.capWin.height  = fmt.fmt.pix.height;

		cam.lastValidID = OSC_CAM_INVALID_BUFFER_ID;

		return SUCCESS;
	}


	OSC_ERR OscStartStreamV4l2()
	{
		enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		int ret = ioctl(cam.vidDev, VIDIOC_STREAMON, &type);
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: start video streaming: \
					IOCTL failed with %d.\n",
					__func__, errno);

			switch(errno)
			{
			case EBUSY:
				OscLog(ERROR,
						"%s: Camera device is busy.\n",
						__func__);
				return -EDEVICE_BUSY;
				break;
			default:
				OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
						__func__, strerror(errno), errno);
				return -EDEVICE;
			}
		}
		return SUCCESS;
	}


	OSC_ERR OscDeqBufV4l2(uint8* fb)
	{
		int ret, i;
		/* dequeue a first buffer for processing */
		memset(&grabBuf, 0, sizeof(grabBuf));
		grabBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		grabBuf.memory = V4L2_MEMORY_USERPTR;

		ret = ioctl(cam.vidDev, VIDIOC_DQBUF, &grabBuf);
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: Unable to set up capture on new framebuffer: \
					IOCTL failed with %d.\n",
					__func__, errno);

			switch(errno)
			{
			case EBUSY:
				OscLog(ERROR,
						"%s: Camera device is busy.\n",
						__func__);
				return -EDEVICE_BUSY;
				break;
			default:
				OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
						__func__, strerror(errno), errno);
				return -EDEVICE;
			}
		}

		/* consistency check */
		for (i = 0; i < MAX_NR_FRAME_BUFFERS; ++i)
			if (grabBuf.m.userptr == (unsigned long) cam.fbufs[i].data
				&& grabBuf.length == cam.fbufs[i].size)
					break;

		*fb = (uint8) i;

		if(unlikely(i == MAX_NR_FRAME_BUFFERS))
		{
			OscLog(ERROR,
					"%s: ioctl(.., VIDIOC_DQBUF,..) did not give a valid buffer id: %d\n",
					__func__, i);
			return -EINVALID_PARAMETER;
		}

		cam.fbStat[*fb] = STATUS_VALID;
		cam.lastValidID = *fb;

		return SUCCESS;
	}


	OSC_ERR OscQueBufV4l2()
	{
		int	ret;

		/* queue buffer from last grabbing */
		ret = ioctl(cam.vidDev, VIDIOC_QBUF, &grabBuf);
		if(unlikely(ret < 0))
		{
			OscLog(ERROR,
					"%s: Unable to queue buffer from last grab: \
					IOCTL failed with %d.\n",
					__func__, errno);

			switch(errno)
			{
			case EINVAL:  /* Invalid parameter */
				OscLog(ERROR, "%s(%u): Invalid parameter!\n",
						__func__, grabBuf.index);
				return -EINVALID_PARAMETER;
				break;
			case EBUSY:
				OscLog(ERROR,
						"%s: Camera device is busy.\n",
						__func__);
				return -EDEVICE_BUSY;
				break;
			default:
				OscLog(ERROR, "%s: IOCTL Error \"%s\" (%d)\n",
						__func__, strerror(errno), errno);
				return -EDEVICE;
			}
		}

		/* set status again to capturing */
		cam.fbStat[cam.lastValidID] = STATUS_CAPTURING_SINGLE;

		return SUCCESS;
	}

#endif /* TARGET_TYPE_RASPI_CAM */

