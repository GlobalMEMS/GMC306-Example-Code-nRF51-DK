/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmc306.c
 *
 * Date : 2017/02/16
 *
 * Usage: GMC306 sensor driver file
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
 
/*! @file gmc306.c
 *  @brief  GMC306 Sensor Driver File 
 *  @author Joseph FC Tseng
 */
 
#include <stddef.h>
#include "nrf_error.h"
#include "gmc306.h"
 
static bus_support_t* pBus_support = 0;
 
/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmc306_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_read(pBus_support->p_app_twi, pBus_support->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes read
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;
}
 

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmc306_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_write(pBus_support->p_app_twi, pBus_support->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes write
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;	
}

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
s8 gmc306_bus_init(bus_support_t* pbus){
	
  s8 comRslt = -1;
  u8 u8Data;
	
  //assign the I2C/SPI bus
  if(pbus == NULL)
    return -127;
  else
    pBus_support = pbus;
	
  //Read chip ID
  comRslt = gmc306_burst_read(GMC306_REG_DEVID, &u8Data, 1);
	
  return comRslt;
}
 
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
s8 gmc306_soft_reset(void){
	
  s8 comRslt = -1;
  u8 u8Data = 0;
	
  u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_RST, 1);

  //Set the RST bit
  comRslt = gmc306_burst_write(GMC306_RST__REG, &u8Data, 1);
	
  return comRslt;
}

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
s8 gmc306_set_operation_mode(GMC306_OP_MODE_T opMode){

  s8 comRslt = -1;
  u8 u8Data = 0;

  //Read the CTRL1 register first
  comRslt = gmc306_burst_read(GMC306_OP_MODE__REG, &u8Data, 1);
  if(comRslt < 0) goto EXIT;

  //set power down mode bits
  u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_OP_MODE, GMC306_OP_MODE_POWER_DOWN);

  //set to the power down mode
  comRslt += gmc306_burst_write(GMC306_OP_MODE__REG, &u8Data, 1);
  if(comRslt < 0) goto EXIT;

  if(opMode != GMC306_OP_MODE_POWER_DOWN){

    //set the target OP mode bits
    u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_OP_MODE, opMode);
	
    //set to power down mode first, then write operation mode register to change mode
    comRslt += gmc306_burst_write(GMC306_OP_MODE__REG, &u8Data, 1);
  }
	
 EXIT:
  return comRslt;
}

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
s8 gmc306_set_lowpass_filter(GMC306_ON_OFF_T on_off){

  s8 comRslt = -1;
  u8 u8Data = 0;

  //Read the CTRL1 register first
  comRslt = gmc306_burst_read(GMC306_OP_MODE__REG, &u8Data, 1);
  if(comRslt < 0) goto EXIT;

  //set the filter bit and write to the CTRL1 register
  u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_FILON, on_off);
  comRslt += gmc306_burst_write(GMC306_OP_MODE__REG, &u8Data, 1);

 EXIT:
  return comRslt;
}

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
s8 gmc306_set_data_bit_depth(GMC306_BIT_DEPTH_T depth){

  s8 comRslt = -1;
  u8 u8Data = 0;

  //Read the CTRL1 register first
  comRslt = gmc306_burst_read(GMC306_OP_MODE__REG, &u8Data, 1);
  if(comRslt < 0) goto EXIT;

  //set the bit-depth bit and write to the CTRL1 register
  u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_BIT, depth);
  comRslt += gmc306_burst_write(GMC306_OP_MODE__REG, &u8Data, 1);

 EXIT:
  return comRslt;
}

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
s8 gmc306_set_measurement_duration(GMC306_MDS_T mds){

  s8 comRslt = -1;
  u8 u8Data = 0;

  //Read the CTRL3 register first
  comRslt = gmc306_burst_read(GMC306_MDS__REG, &u8Data, 1);
  if(comRslt < 0) goto EXIT;

  //set the MDS bit and write to the CTRL3 register
  u8Data = GMC306_SET_BITSLICE(u8Data, GMC306_MDS, mds);
  comRslt += gmc306_burst_write(GMC306_MDS__REG, &u8Data, 1);

 EXIT:
  return comRslt;
}

/*!
 * @brief GMC306 read data XYZ
 *
 * @param pxyz Data buffer to store the values
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmc306_read_data_xyz(raw_data_xyzt_t* pxyz){
	
  s8 comRslt = -1;
  u8 u8Data[6];
  s16 s16Tmp, i;
	
  comRslt = gmc306_burst_read(GMC306_DATA_READ_START__REG, u8Data, 6);
	
  if(comRslt < 0) goto EXIT;
	
  for(i = 0; i < 3; ++i){
    s16Tmp = (u8Data[2*i] << 8) | (u8Data[2*i + 1]);
    pxyz->v[i] = s16Tmp;
  }
	
 EXIT:
  return comRslt;
}

/* @brief GMC306 get XYZ sensitivity adjustment value
 * Read ASAX/Y/Z and calculate the sensitivity adjustment value to return
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
s8 gmc306_get_sensitivity_adjust_val(float_xyzt_t* pAdjustVal){
	
  s8 comRslt = -1;
  u8 u8Data[3];
  s32 i;
	
  //Read 3 bytes starting from 53h
  comRslt = gmc306_burst_read(GMC306_ASA_XYZ_START__REG, u8Data, 3);
	
  if(comRslt < 0) goto EXIT;
	
  for(i = 0; i < 3; ++i)
    pAdjustVal->v[i] = 1.0f + (u8Data[i] / 128.0f);
	
 EXIT:
  return comRslt;	
}
