/*
  This is an updated v5.x ESP32 IDF library written for the Sensirion SHT4x
  Digital Humidity and Temperature Sensors.

  It is a port by Jefferson J Hunt @ OOE, August 28th, 2024 from the
  ESP-IDF driver by Ruslan V. Uss (https://github.com/UncleRus) 2021

  SHT4x is a digital sensor platform for measuring relative humidity and 
  temperature at different accuracy classes. Its I2C interface provides 
  several preconfigured I2C addresses while maintaining an ultra-low power
  budget (0.4 μW). The power-trimmed internal heater can be used at three
  heating levels thus enabling sensor operation in demanding environments.

  Original Ruslan V. Uss driver
  https://github.com/UncleRus/esp-idf-lib

  ESP32 IDF port by Jefferson J Hunt
  https://github.com/jeffersonjhunt/esp32-idf-sh4x
*/

#ifndef SHT4X_h
#define SHT4X_h

#include <freertos/FreeRTOS.h>
#include <driver/i2c_master.h>
#include <driver/i2c_types.h>
#include <esp_log.h>

#define SHT4X_I2CADDR_DEFAULT 0x44 // Default I2C address for the SHT4X

typedef uint8_t sht4x_data_t[4];

typedef enum
{
  CMD_MEAS_HIGHREP = 0xfd,
  CMD_MEAS_MEDREP = 0xf6,
  CMD_MEAS_LOWREP = 0xe0,
  CMD_MEAS_HIGHREP_LONG = 0x39,
  CMD_MEAS_HIGHREP_SHORT = 0x32,
  CMD_MEAS_MEDREP_LONG = 0x2f,
  CMD_MEAS_MEDREP_SHORT = 0x24,
  CMD_MEAS_LOWREP_LONG = 0x1e,
  CMD_MEAS_LOWREP_SHORT = 0x15
} sht4x_cmd_t;

class SHT4X
{
public:
  /**
   * @brief Construct a new SHT4X object and bind it the I2C bus
   *
   * @param bus_handle I2C bus handle
   * @param dev_config I2C device configuration
   */
  SHT4X(i2c_master_bus_handle_t *bus_handle, i2c_device_config_t *dev_config);

  /**
   * @brief Free the SHT4X device from the I2C bus
   * 
   */
  ~SHT4X();

  /**
   * @brief Initialize the SHT4X sensor for basic function
   * 
   * If initialize is true, the sensor will be reset and the serial number will
   * be read. If initialize is false, the sensor will be checked for connection
   * only.
   * 
   * @param initialize If true, initialize and calibrate the sensor
   * @return true if sensor is connected
   */
  bool begin(bool initialize); // Check communication and initialize sensor

  /**
   * @brief Measure temperature and humidity in Celsius and %RH
   * 
   * This is the basic measurement function that should be used in most cases.
   * It does not provide thread safety, if you are using multiple threads/tasks
   * to access the sensor, you should use the read() and write() functions with
   * the appropriate locks.
   * 
   * @param cmd mesurement to perform (see datasheet for details) 
   * @param temperature 
   * @param humidity 
   * @return esp_err_t 
   */
  esp_err_t measure(sht4x_cmd_t cmd, float *temperature, float *humidity);

  /**
   * @brief 
   * 
   * @param cmd 
   * @param data 
   * @return esp_err_t 
   */
  esp_err_t read(uint8_t cmd, sht4x_data_t &data);

  /**
   * @brief 
   * 
   * @param cmd 
   * @return esp_err_t 
   */
  esp_err_t write(uint8_t cmd);

  /**
   * @brief Reset the SHT4X sensor
   * 
   * @return true if reset is successful
   */
  bool reset();       // Reset the SHT4X

  /**
   * @brief Check if the SHT4X sensor is connected
   * 
   * @return true if device acks at the I2C address
   */
  bool isConnected();

  /**
   * @brief Get the Serial Number of the SHT4X sensor
   * 
   * The serial number is determined during initialization and is stored in the
   * class instance. This function returns the serial number. If initialization
   * is not eanbled, it will return 0.
   * 
   * @return uint32_t 
   */
  uint32_t getSerialNumber();

private:
  i2c_master_bus_handle_t *bus_handle;
  i2c_master_dev_handle_t dev_handle;

  uint32_t serialNumber;

  uint16_t getDelay(uint8_t cmd);                       // return time for measurement to complete
  void delay(uint8_t cmd);                              // wait for the command to complete
  esp_err_t readSerialNumber();                         // Read the serial number
  uint8_t calculate8BitCRC(uint8_t data[], size_t len); // Calculate 8-bit CRC
};

#endif // SHT4X_h
