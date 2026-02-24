/*****************************************************************************
| File        :   tvoc_sensor.h
| Author      :   Waveshare team
| Function    :   Hardware underlying interface
| Info        :   This file provides the interface for interacting with a TVOC sensor.
----------------
| This version:   V1.0
| Date        :   2025-03-12
| Info        :   Initial release of the TVOC sensor library.

#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
*****************************************************************************/

#ifndef _TVOC_SENSOR_H_
#define _TVOC_SENSOR_H_

#if defined (__AVR_ATmega328P__)
  #include <SoftwareSerial.h>

  // Define the UART interface and pins used for TVOC sensor communication
  #define tvoc_rx 10        // Define the RX pin for UART communication
  #define tvoc_tx 11        // Define the TX pin for UART communication

  //Redefine serial port name 重定义串口名
  extern SoftwareSerial tvoc_uart;
#else
  //Redefine serial port name 重定义串口名
  #define tvoc_uart  Serial1
#endif

#define tvoc_rst 9       // Reset pin (used to reset the sensor)
#define tvoc_alm 8       // Alarm pin (used to receive alarm signals from the sensor)

// 声明传感器全局变量，供外部文件（ino）访问
extern float tvoc;        // TVOC浓度，单位ppm
extern uint16_t co2;      // CO2浓度，单位ppm
extern uint16_t ch2o;     // 甲醛浓度，单位ppb


/**
 * Initialize the TVOC sensor hardware interface.
 * This function sets up the UART communication and prepares the sensor for use.
 * It also configures the reset and alarm pins.
 */
void tvoc_init();

/**
 * Set the TVOC sensor to active mode.
 * In active mode, the sensor continuously sends data without needing a query.
 * This mode is suitable for real-time monitoring applications.
 */
void tvoc_set_device_active_mode();

/**
 * Set the TVOC sensor to query mode.
 * In query mode, the sensor only sends data when explicitly requested.
 * This mode is suitable for applications where data is only needed periodically.
 */
void tvoc_set_device_query_mode();

/**
 * Get data from the TVOC sensor in active mode.
 * This function retrieves the latest data sent by the sensor in active mode.
 * It assumes that the sensor is already configured in active mode.
 */
void tvoc_get_active_device_data();

/**
 * Get data from the TVOC sensor in query mode.
 * This function requests and retrieves data from the sensor in query mode.
 * It assumes that the sensor is already configured in query mode.
 */
void tvoc_get_query_device_data();

#endif