import time
from machine import Timer
import micropython

BMP280_ADDR = 0x76

BMP280_REG_ID = 0xD0
BMP280_ID = 0x58

BMP280_REG_COMPENSATION_PARAMETERS = 0x88
BMP280_COMPENSATION_PARAMETERS_LENGTH = 26

BMP280_REG_CTRL_MEAS = 0xF4

BMP280_CTRL_MEAS_MODE_MASK = 0xb11
BMP280_CTRL_MEAS_MODE_NORMAL = 0b11
BMP280_CTRL_MEAS_MODE_FORCED = 0b01

BMP280_REG_STATUS = 0xF3
BMP280_STATUS_MEASURING = 1 << 3
BMP280_STATUS_IM_UPDATE = 1

BMP280_REG_CONFIG = 0xF5
BMP280_CONFIG_T_SB_MASK = 0b111

BMP280_REG_PRESS = 0xF7
BMP280_REG_PRESS_LENGTH = 3
BMP280_REG_TEMP = 0xF9
BMP280_REG_TEMP_LENGTH = 3

COMPENSATION_PARAMETER_SIGN_BIT = 1 << 15

class InvalidDevice(Exception):
  def __init__(self, id):
    super().__init__("Device ID was " + id + " but should be " + BMP280_ID)

class Timeout(Exception):
  pass

class BMP280():

  CONFIG_FILTER_OFF = 0b000
  CONFIG_FILTER_2 = 0b001
  CONFIG_FILTER_4 = 0b010
  CONFIG_FILTER_8 = 0b011
  CONFIG_FILTER_16 = 0b100

  CTRL_MEAS_OVERSAMPLING_T_OFF = 0b000
  CTRL_MEAS_OVERSAMPLING_T_1 = 0b001
  CTRL_MEAS_OVERSAMPLING_T_2 = 0b010
  CTRL_MEAS_OVERSAMPLING_T_4 = 0b011
  CTRL_MEAS_OVERSAMPLING_T_8 = 0b100
  CTRL_MEAS_OVERSAMPLING_T_16 = 0b101

  CTRL_MEAS_OVERSAMPLING_P_OFF = 0b000
  CTRL_MEAS_OVERSAMPLING_P_1 = 0b001
  CTRL_MEAS_OVERSAMPLING_P_2 = 0b010
  CTRL_MEAS_OVERSAMPLING_P_4 = 0b011
  CTRL_MEAS_OVERSAMPLING_P_8 = 0b100
  CTRL_MEAS_OVERSAMPLING_P_16 = 0b101

  def __init__(self, i2c, sdo = 0):
    self.i2c = i2c
    if (sdo != 0) and (sdo != 1):
      raise ValueError("Invalid Adress bit: " + sdo)
    self.addr = BMP280_ADDR + sdo

  def initialize(self, config_filter = CONFIG_FILTER_16, ctrl_meas_oversampling_t = CTRL_MEAS_OVERSAMPLING_T_2, ctrl_meas_oversampling_p = CTRL_MEAS_OVERSAMPLING_P_16):
    self.__self_check()
    self.__read_compensation_parameters()
    self.i2c.writeto_mem(self.addr, BMP280_REG_CONFIG, bytes([config_filter << 2]))
    self.i2c.writeto_mem(self.addr, BMP280_REG_CTRL_MEAS, bytes([(ctrl_meas_oversampling_t << 5 | ctrl_meas_oversampling_p << 2) & ~BMP280_CTRL_MEAS_MODE_MASK]))

  def __self_check(self):
    id = self.i2c.readfrom_mem(self.addr, BMP280_REG_ID, 1)[0]
    if (id != BMP280_ID):
      raise InvalidDevice(id)

  def __read_compensation_parameters(self):
    compensation_parameters = self.i2c.readfrom_mem(self.addr, BMP280_REG_COMPENSATION_PARAMETERS, BMP280_COMPENSATION_PARAMETERS_LENGTH)
    self.dig_T1 = self.__to_unsigned_parameter(compensation_parameters, 0)
    self.dig_T2 = self.__to_signed_parameter(compensation_parameters, 1)
    self.dig_T3 = self.__to_signed_parameter(compensation_parameters, 2)
    self.dig_P1 = self.__to_unsigned_parameter(compensation_parameters, 3)
    self.dig_P2 = self.__to_signed_parameter(compensation_parameters, 4)
    self.dig_P3 = self.__to_signed_parameter(compensation_parameters, 5)
    self.dig_P4 = self.__to_signed_parameter(compensation_parameters, 6)
    self.dig_P5 = self.__to_signed_parameter(compensation_parameters, 7)
    self.dig_P6 = self.__to_signed_parameter(compensation_parameters, 8)
    self.dig_P7 = self.__to_signed_parameter(compensation_parameters, 9)
    self.dig_P8 = self.__to_signed_parameter(compensation_parameters, 10)
    self.dig_P9 = self.__to_signed_parameter(compensation_parameters, 11)

  def __to_unsigned_parameter(self, buffer, word_offset = 0):
    buffer_offset = word_offset << 1
    return buffer[1 + buffer_offset] << 8 | buffer[0 + buffer_offset]

  def __to_signed_parameter(self, buffer, word_offset = 0):
    unsigned_parameter = self.__to_unsigned_parameter(buffer, word_offset)
    if (unsigned_parameter & COMPENSATION_PARAMETER_SIGN_BIT) != 0:
      return ~((COMPENSATION_PARAMETER_SIGN_BIT << 1) - 1) | unsigned_parameter
    else:
      return unsigned_parameter

  def trigger_single_measurement(self, callback = None):
    self.__trigger_single_measurement()

    if (callback == None):
      return self.__single_measurement_without_callback()
    else:
      self.__single_measurement_with_callback(callback)

  def __trigger_single_measurement(self):
    ctrl_meas = self.i2c.readfrom_mem(self.addr, BMP280_REG_CTRL_MEAS, 1)[0]
    ctrl_meas = (ctrl_meas & ~BMP280_CTRL_MEAS_MODE_MASK) | BMP280_CTRL_MEAS_MODE_FORCED
    self.i2c.writeto_mem(self.addr, BMP280_REG_CTRL_MEAS, bytes([ctrl_meas]))

  def __single_measurement_without_callback(self):
    # Todo: Adjust sleep time to measurement parameters.
    time.sleep(0.044)
    if (self.__measuring_finished() == True):
      return self.read_measurements()
    else:
      raise Timeout()

  def __single_measurement_with_callback(self, callback):
    def finish_measurement(_):
      if (self.__measuring_finished() == True):
        return callback(self.read_measurements())
      else:
        raise Timeout()

    Timer().init(mode = Timer.ONE_SHOT, period = 44, callback = lambda timer : micropython.schedule(finish_measurement, None))

  def __measuring_finished(self):
    status = self.i2c.readfrom_mem(self.addr, BMP280_REG_STATUS, 1)[0]
    return (status & BMP280_STATUS_MEASURING == 0) and (status & BMP280_STATUS_IM_UPDATE == 0)

  def start_periodic_measurement(self, standby_time = 4000):
    ctrl_meas = self.i2c.readfrom_mem(self.addr, BMP280_REG_CTRL_MEAS, 1)[0]
    ctrl_meas = (ctrl_meas & ~BMP280_CTRL_MEAS_MODE_MASK) | BMP280_CTRL_MEAS_MODE_NORMAL
    self.i2c.writeto_mem(self.addr, BMP280_REG_CTRL_MEAS, bytes([ctrl_meas]))
    config = self.i2c.readfrom_mem(self.addr, BMP280_REG_CONFIG, 1)[0]
    config = (config & ~BMP280_CONFIG_T_SB_MASK) | self.__ms_to_standby_value(standby_time)
    self.i2c.writeto_mem(self.addr, BMP280_REG_CONFIG, bytes([config]))

  def __ms_to_standby_value(self, standby_time_ms):
    if (standby_time_ms >= 4000):
      return 0xb111
    elif (standby_time_ms >= 2000):
      return 0xb110
    elif (standby_time_ms >= 1000):
      return 0xb101
    elif (standby_time_ms >= 500):
      return 0xb100
    elif (standby_time_ms >= 250):
      return 0xb011
    elif (standby_time_ms >= 120):
      return 0xb010
    elif (standby_time_ms >= 62.5):
      return 0xb001
    else:
      return 0b000

  def read_measurements(self):
    raw = self.i2c.readfrom_mem(self.addr, BMP280_REG_PRESS, BMP280_REG_PRESS_LENGTH + BMP280_REG_TEMP_LENGTH)

    adc_temperature = self.__to_raw_value(raw, BMP280_REG_PRESS_LENGTH)
    [temperature, t_fine] = self.__compensate_temperature(adc_temperature)

    adc_pressure = self.__to_raw_value(raw, 0)
    pressure = self.__compensate_pressure(adc_pressure, t_fine)

    return temperature, pressure

  def __to_raw_value(self, buffer, offset = 0):
    return (buffer[0 + offset] << 12) | (buffer[1 + offset] << 4) | (buffer[2 + offset] >> 4)

  def __compensate_temperature(self, adc_value):
    # Check datasheet for formula
    v1 = (((adc_value >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
    v2 = ((((adc_value >> 4) - self.dig_T1) * ((adc_value >> 4) - self.dig_T1)) >> 12) * (self.dig_T3 >> 14)
    t_fine = v1 + v2
    t = (t_fine * 5 + 128) >> 8
    return [ t / 100, t_fine]

  def __compensate_pressure(self, adc_value, t_fine):
    # Check datasheet for formula
    v1 = t_fine - 128000
    v2 = v1 * v1 * self.dig_P6
    v2 = v2 + ((v1 * self.dig_P5) << 17)
    v2 = v2 + (self.dig_P4 << 35)
    v1 = ((v1 * v1 * self.dig_P3) >> 8) + ((v1 * self.dig_P2) << 12)
    v1 = (((1 << 47) + v1) * self.dig_P1) >> 33
    if (v1 == 0):
      return 0
    p = 1048576 - adc_value
    p = int(((p << 31) - v2) * 3125 / v1)
    v1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
    v2 = (self.dig_P8 * p) >> 19
    p = ((p + v1 + v2) >> 8) + (self.dig_P7 << 4)
    return p / 256
