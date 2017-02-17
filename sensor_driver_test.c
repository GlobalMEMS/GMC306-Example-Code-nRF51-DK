/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Date : 2017/2/17
 *
 * Usage: GMC306 Sensor Driver Test for nRF51-DK
 *
 ****************************************************************************
 * @section License
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
 
/*! @file sensor_driver_test.c
 *  @brief  GMC306 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf.h"
#include "bsp.h"
#include "gmc306.h"
#include "app_twi.h"

#define UART_TX_BUF_SIZE            256                  // UART TX buffer size
#define UART_RX_BUF_SIZE            1                    // UART RX buffer size
#define MAX_PENDING_TRANSACTIONS    5                    // TWI (I2C)
#define DELAY_MS(ms)	            nrf_delay_ms(ms)

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static void event_handler_uart(app_uart_evt_t * p_event){

  uint8_t cr;

  switch (p_event->evt_type){

  case APP_UART_DATA_READY: //echo

    while(app_uart_get(&cr) == NRF_SUCCESS){
      printf("%c", cr);
    }
    break;
  case APP_UART_TX_EMPTY:
    //do nothin
    break;
  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;
  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

void init_lfclk(void){

  uint32_t err_code;

  // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
  // buttons with the use of APP_TIMER

  err_code = nrf_drv_clock_init(NULL);
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request();

}

void init_uart(void)
{
  uint32_t err_code;

  app_uart_comm_params_t const comm_params =
    {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
      APP_UART_FLOW_CONTROL_DISABLED,
      false,
      UART_BAUDRATE_BAUDRATE_Baud115200
    };

  APP_UART_FIFO_INIT(&comm_params,
		     UART_RX_BUF_SIZE,
		     UART_TX_BUF_SIZE,
		     event_handler_uart,
		     APP_IRQ_PRIORITY_LOW,
		     err_code);

  APP_ERROR_CHECK(err_code);
}

/**
 * Initialize two wire interface (I2C)
 */
void init_twi(nrf_twi_frequency_t clk){

  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = clk,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };

  APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
  APP_ERROR_CHECK(err_code);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  s32 i;
  bus_support_t gmc306_bus;
  raw_data_xyzt_t rawData;
  float_xyzt_t calibData;
  float_xyzt_t adjustVal = { 1.0, 1.0, 1.0, 0.0 };

  //Config and initialize LFCLK
  init_lfclk();

  //Config. and initialize UART
  init_uart();

  //Config. and initialize TWI (I2C)
  init_twi(NRF_TWI_FREQ_400K);
	
  /* GMC306 I2C bus setup */
  bus_init_I2C(&gmc306_bus, &m_app_twi, GMC306_7BIT_I2C_ADDR);  //Initialize I2C bus
  gmc306_bus_init(&gmc306_bus); //Initailze GMC306 bus to I2C

  /* GMC306 soft reset */
  gmc306_soft_reset();
	
  /* Wait 10ms for reset complete */
  DELAY_MS(10);
	
  /* GMC306 get the sensitivity adjust values */
  gmc306_get_sensitivity_adjust_val(&adjustVal);
	
  printf("Sadj=%d.%d, %d.%d, %d.%d\n",
	 (s32)adjustVal.u.x, abs((s32)((adjustVal.u.x - (s32)adjustVal.u.x)*10000)),
	 (s32)adjustVal.u.y, abs((s32)((adjustVal.u.y - (s32)adjustVal.u.y)*10000)),
	 (s32)adjustVal.u.z, abs((s32)((adjustVal.u.z - (s32)adjustVal.u.z)*10000)));
									
  //Set to CM 10Hz
  gmc306_set_operation_mode(GMC306_OP_MODE_CM_10HZ);

  for (;;){
		
    //Read XYZ raw
    gmc306_read_data_xyz(&rawData);
		
    //Sensitivity adjustment
    for(i = 0; i < 3; ++i)
      calibData.v[i] = rawData.v[i] * adjustVal.v[i];
		
    printf("XYZ=%d.%d, %d.%d, %d.%d\n",
	   (s32)calibData.u.x, abs((s32)((calibData.u.x - (s32)calibData.u.x)*100)),
	   (s32)calibData.u.y, abs((s32)((calibData.u.y - (s32)calibData.u.y)*100)),
	   (s32)calibData.u.z, abs((s32)((calibData.u.z - (s32)calibData.u.z)*100)));

    /* Delay 1 sec */
    DELAY_MS(1000);
  }
}
