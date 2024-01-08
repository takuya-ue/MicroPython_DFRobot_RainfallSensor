# -*- coding: utf-8 -*
'''!
  @file        MicroPython_DFRobot_RainfallSensor.py
  @brief       Customized version of SEN0575 device library for Raspberry Pi Pico with MicroPython
  @modification  This file is a modified version of the DFRobot_RainfallSensor.py, adapted for Raspberry Pi Pico with MicroPython.
  @author      takuya-ue(takuya-ue@nifty.com)
  @version     V1.1
  @date        2024-01-08
  @url         https://github.com/takuya-ue/MicroPython_DFRobot_RainfallSensor
  
  @file        DFRobot_RainfallSensor.py
  @brief       SEN0575 device library
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      fary(feng.yang@dfrobot.com)
  @version     V1.0
  @date        2023-02-28
  @url         https://github.com/DFRobor/DFRobot_RainfallSensor
'''

from machine import Pin, I2C
import time


class DFRobot_RainfallSensor(object):
  ## SEN0575 stores the address of the input register for PID in memory.
  SEN0575_INPUT_REG_PID                          = 0x0000
  ## The address of the input register for VID in memory.
  SEN0575_INPUT_REG_VID                          = 0x0001
  ## The address of the input register for device address in memory.
  SEN0575_INPUT_REG_ADDR                         = 0x0002
  ## The address of the input register for device baudrate in memory.
  SEN0575_INPUT_REG_BAUD                         = 0x0003
  ## The address of the input register for RS485 parity bit and stop bit in memory.
  SEN0575_INPUT_REG_VERIFYANDSTOP                = 0x0004
  ## The address of the input register for firmware version in memory.
  SEN0575_INPUT_REG_VERSION                      = 0x0005
  
  ## The address of the input register for low 16-bit cumulative rainfall in set time.
  SEN0575_INPUT_REG_TIME_RAIN_FALL_L             = 0x0006
  ## The address of the input register for high 16-bit cumulative rainfall in set time.
  SEN0575_INPUT_REG_TIME_RAIN_FALL_H             = 0x0007
  ## The address of the input register for low 16-bit cumulative rainfall since working started.
  SEN0575_INPUT_REG_CUMULATIVE_RAINFALL_L        = 0x0008  
  ## The address of the input register for high 16-bit cumulative rainfall since working started.
  SEN0575_INPUT_REG_CUMULATIVE_RAINFALL_H        = 0x0009  
  ## The address of the input register for raw data (low 16-bit) in memory.
  SEN0575_INPUT_REG_RAW_DATA_L                   = 0x000A  
  ## The address of the input register for raw data (high 16-bit) in memory.
  SEN0575_INPUT_REG_RAW_DATA_H                   = 0x000B  
  ## The address of the input register for system working time in memory.
  SEN0575_INPUT_REG_SYS_TIME                     = 0x000C
  ## Set the time to calculate cumulative rainfall.
  SEN0575_HOLDING_REG_RAW_RAIN_HOUR              = 0x0006
  ## Set the base rainfall value.
  SEN0575_HOLDING_REG_RAW_BASE_RAINFALL          = 0x0007
  
  ## The address of the input register for PID in memory.
  SEN0575_I2C_REG_PID                          = 0x00
  ## The address of the input register for VID in memory.
  SEN0575_I2C_REG_VID                          = 0x02
  ## The address of the input register for firmware version in memory.
  SEN0575_I2C_REG_VERSION                      = 0x0A
  ## SEN0575 Stores the cumulative rainfall within the set time
  SEN0575_I2C_REG_TIME_RAINFALL                = 0x0C
  ## SEN0575 Stores the cumulative rainfall since starting work
  SEN0575_I2C_REG_CUMULATIVE_RAINFALL          = 0x10
  ## SEN0575 Stores the low 16 bits of the raw data
  SEN0575_I2C_REG_RAW_DATA                     = 0x14
  ## SEN0575 Stores the system time
  SEN0575_I2C_REG_SYS_TIME                     = 0x18
  
  ## Sets the time for calculating the cumulative rainfall
  SEN0575_I2C_REG_RAW_RAIN_HOUR                = 0x26
  ## Sets the base value of accumulated rainfall
  SEN0575_I2C_REG_RAW_BASE_RAINFALL            = 0x28

  '''!
    @brief Define DFRobot_RainfallSensor basic class
    @details Drive the sensor
  '''
  def __init__(self):
    self.vid = 0
    self.pid = 0
  
  def begin(self):
    '''!
      @brief This function will attempt to communicate with a slave device and determine if the communication is successful based on the return value.
      @return Communication result
      @retval true  Succeed
      @retval false Failed
    '''
    return self.get_pid_vid()

  def get_firmware_version(self):
    '''!
      @brief  get firmware version
      @return  Return  firmware version
    '''
    list = self._read_register(self.SEN0575_I2C_REG_VERSION, 2)
    version = list[0] | ( list[1] << 8 )
    return str(version >> 12) + '.'+ str( ( (version >> 8) & 0x0F ) ) + '.' + str( ( (version >> 4) & 0x0F ) ) + '.' + str( (version & 0x0F) )

  def get_sensor_working_time(self):
    '''!
      @brief Obtain the sensor working time
      @return Working time of the sensor, in hours
    '''
    list = self._read_register(self.SEN0575_I2C_REG_SYS_TIME, 2)
    working_time = list[0] | (list[1] << 8)
    return working_time / 60.0

  def get_pid_vid(self):
    '''!
      @brief  Get the PID and VID of the device.
      @return   Return true if the data is obtained correctly, false if failed or incorrect data.
    '''
    ret = False
    list = self._read_register(self.SEN0575_I2C_REG_PID, 4)
    self.pid = list[0] | ( list[1] << 8 ) | ( ( list[3] & 0xC0 ) << 10 )
    self.vid = list[2] | ( ( list[3] & 0x3F ) << 8 )
    if (self.vid == 0x3343) and (self.pid == 0x100C0):
      ret = True
    return ret

  def get_rainfall(self):
    '''!
      @brief  Get cumulative rainfall
      @return Cumulative rainfall value
    '''
    list = self._read_register(self.SEN0575_I2C_REG_CUMULATIVE_RAINFALL, 4)
    rainfall = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    return rainfall / 10000.0

  def get_rainfall_time(self,hour):
    '''!
      @brief  Get the cumulative rainfall within the specified time
      @param hour Specified time (valid range is 1-24 hours)
      @return Cumulative rainfall
    '''
    if hour>24:
      return 0
    list = [hour]
    self._write_register(self.SEN0575_I2C_REG_RAW_RAIN_HOUR, list)
    list = self._read_register(self.SEN0575_I2C_REG_TIME_RAINFALL, 4)
    rainfall = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    return rainfall / 10000.0

  def get_raw_data(self):
    '''!
      @brief Get raw data
      @return Number of tipping bucket counts, unit: count
    '''
    list = self._read_register(self.SEN0575_I2C_REG_RAW_DATA, 4)
    raw_data = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    return raw_data

  def set_rain_accumulated_value(self, value):
    '''!
      @brief Set the rainfall accumulation value
      @param value Rainfall accumulation value, in millimeters
      @return Returns 0 if setting succeeded, other values if setting failed
    '''
    data = int(value * 10000)
    list=[data & 0xFF , data >> 8]
    ret=self._write_register(self.SEN0575_I2C_REG_RAW_BASE_RAINFALL, list)
    return ret

  def _write_reg(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    # Low level register writing, not implemented in base class
    raise NotImplementedError()

  def _read_reg(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    # Low level register writing, not implemented in base class
    raise NotImplementedError()


class DFRobot_RainfallSensor_I2C(DFRobot_RainfallSensor):
  '''!
    @brief Define DFRobot_RainfallSensor_I2C class
    @details Drive the sensor
  '''
  def __init__(self,sda,scl):
    '''!
      @brief Initialize I2C
    '''
    self._addr = 0x1D
    self.i2c = I2C(0, sda=sda, scl=scl, freq=40000)
    super(DFRobot_RainfallSensor_I2C, self).__init__()

  def _write_register(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    try:
      self.i2c.writeto_mem(self._addr, reg, data)
      time.sleep(0.050)
    except:
      pass
    return True

  def _read_register(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    buf = bytearray(length)
    try:      
      self.i2c.readfrom_mem_into(self._addr, reg, buf)
    except:
      pass
    return buf