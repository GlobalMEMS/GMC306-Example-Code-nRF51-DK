/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmc306.h
 *
 * Date : 2017/02/16
 *
 * Usage: GMC306 sensor driver header file
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file gmc306.h
 *  @brief  GMC306 Sensor Driver Header File 
 *  @author Joseph FC Tseng
 */
 
#ifndef __GMC306_H__
#define __GMC306_H__

#include "bus_support.h"

#define GMC306_7BIT_I2C_ADDR		0x0C
#define GMC306_RAW_DATA_SENSITIVITY     (100.0/15.0)  //raw data one code = 0.15 uT

#define GMC306_REG_ST1		  0x00
#define GMC306_REG_HXH	 	  0x01
#define GMC306_REG_HXL		  0x02
#define GMC306_REG_HYH	 	  0x03
#define GMC306_REG_HYL		  0x04
#define GMC306_REG_HZH	 	  0x05
#define GMC306_REG_HZL		  0x06
#define GMC306_REG_TMPS		  0x07
#define GMC306_REG_ST2		  0x08
#define GMC306_REG_CNTL1	  0x10
#define GMC306_REG_CNTL2 	  0x11
#define GMC306_REG_CNTL3	  0x12
#define GMC306_REG_CMPID 	  0x50
#define GMC306_REG_DEVID	  0x51
#define GMC306_REG_INFO 	  0x52
#define GMC306_REG_ASAX	 	  0x53
#define GMC306_REG_ASAY	 	  0x54
#define GMC306_REG_ASAZ	 	  0x55

/* PID */
#define GMC306_PID__REG                GMC306_REG_CMPID
/* Soft Rest bit */
#define GMC306_RST__REG		       GMC306_REG_CNTL2
#define GMC306_RST__MSK		       0x01
#define GMC306_RST__POS		       0
/* Data bit resolution */
#define GMC306_BIT__REG                GMC306_REG_CNTL1
#define GMC306_BIT__MSK                0x80
#define GMC306_BIT__POS                7
/* Low pass filter control bit */
#define GMC306_FILON__REG              GMC306_REG_CNTL1
#define GMC306_FILON__MSK              0x40
#define GMC306_FILON__POS              6
/* Operation mode bits */
#define GMC306_OP_MODE__REG	       GMC306_REG_CNTL1
#define GMC306_OP_MODE__MSK	       0x1F
#define GMC306_OP_MODE__POS	       0
/* Measurement duration selection */
#define GMC306_MDS__REG	               GMC306_REG_CNTL3
#define GMC306_MDS__MSK	               0xC0
#define GMC306_MDS__POS	               6
/* Data read start register */
#define GMC306_DATA_READ_START__REG    GMC306_REG_HXH
/*ASA XYZ start register */
#define GMC306_ASA_XYZ_START__REG      GMC306_REG_ASAX


typedef enum {GMC306_OP_MODE_POWER_DOWN = 0x00,
	      GMC306_OP_MODE_SINGLE_MEASUREMENT = 0x01,
	      GMC306_OP_MODE_CM_10HZ = 0x02,
	      GMC306_OP_MODE_CM_20HZ = 0x04,
	      GMC306_OP_MODE_CM_50HZ = 0x06,
	      GMC306_OP_MODE_CM_100HZ = 0x08,
	      GMC306_OP_MODE_CM_200HZ = 0x0C,
	      GMC306_OP_MODE_SELF_TEST = 0x10,	
	      GMC306_OP_MODE_FUSE_ROM_ACCESS = 0x1F} GMC306_OP_MODE_T;

typedef enum{ GMC306_OFF = 0, GMC306_ON = 1} GMC306_ON_OFF_T;

typedef enum{ GMC306_14BIT_MODE = 0, GMC306_16BIT_MODE = 1} GMC306_BIT_DEPTH_T;

typedef enum{ GMC306_MDS_1P8_MS = 1, GMC306_MDS_3P6_MS = 0, GMC306_MDS_7P2_MS = 2} GMC306_MDS_T;

#define GMC306_GET_BITSLICE(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define GMC306_SET_BITSLICE(regvar, bitname, val)			\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success, number of bytes read
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 gmc306_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success, number of bytes write
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmc306_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len);


/*!
 * @brief GMC306 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_bus_init(bus_support_t* pbus);

/*!
 * @brief GMC306 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_soft_reset(void);

/*!
 * @brief GMC306 read data XYZ
 *
 * @param pxyzt Data buffer to store the values
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_read_data_xyz(raw_data_xyzt_t* pxyzt);

/*!
 * @brief GMC306 set operation mode.
 *
 * @param opMode: operation mode
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_set_operation_mode(GMC306_OP_MODE_T opMode);

/*!
 * @brief GMC306 turn on/off lowpass filter
 *
 * @param on_off GMC306_ON to turn it on, GMC306_OFF to turn it off
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_set_lowpass_filter(GMC306_ON_OFF_T on_off);

/*!
 * @brief GMC306 set data bit-depth
 *
 * @param depth GMC306_14BIT_MODE for 14bit, GMC306_16BIT_MODE for 16bit mode
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_set_data_bit_depth(GMC306_BIT_DEPTH_T depth);

/*!
 * @brief GMC306 set measurement duration
 *
 * @param mds
 *        1.8ms: GMC306_MDS_1P8_MS
 *        3.6ms: GMC306_MDS_3P6_MS
 *        7.2ms: GMC306_MDS_7P2_MS
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_set_measurement_duration(GMC306_MDS_T mds);

/* @brief GMC306 get XYZ sensitivity adjustment value
 *  Read ASAX/Y/Z and calculate the sensitivity adjustment value to return
 *
 * @param pAdjustVal Data buffer to store the sensitivity adjustment value
 * 	  pAdjustVal->u.x = 1 + ASAX / 128
 *        pAdjustVal->u.y = 1 + ASAY / 128
 *        pAdjustVal->u.z = 1 + ASAZ / 128
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_get_sensitivity_adjust_val(float_xyzt_t* pAdjustVal);

#endif // __GMC306_H___
