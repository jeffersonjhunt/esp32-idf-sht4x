/*
  This is an ESP32 IDF library written for the NAU7802 24-bit wheatstone
  bridge and load cell amplifier.

  It is a port by Jefferson J Hunt @ OOE, August 23rd, 2024 from the
  Arduino driver by Nathan Seidle @ SparkFun Electronics, March 3nd, 2019

  The NAU7802 is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  The NAU7802 is a better version of the popular HX711 load cell amplifier.
  It uses a true I2C interface so that it can share the bus with other
  I2C devices while still taking very accurate 24-bit load cell measurements
  up to 320Hz.

  Original Nathan Seidle driver
  https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

  ESP32 IDF port by Jefferson J Hunt
  https://github.com/jeffersonjhunt/esp32-idf-nau7802
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
  SHT4X(i2c_master_bus_handle_t *bus_handle, i2c_device_config_t *dev_config); // Default constructor
  ~SHT4X();                                                                    // Destructor

  bool begin(bool initialize); // Check communication and initialize sensor

  esp_err_t read(uint8_t cmd, sht4x_data_t &data);
  esp_err_t write(uint8_t cmd);
  bool reset();                // Reset the SHT4X
  bool isConnected();          // Returns true if device acks at the I2C address

  esp_err_t measure(sht4x_cmd_t cmd, float *temperature, float *humidity); // Measure temperature and humidity in Celsius and %RH
  uint32_t getSerialNumber();                                              // Return the serial number of the sensor

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
