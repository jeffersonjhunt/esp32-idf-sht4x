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

#include "sht4x.h"

static const char *TAG = "driver-sht4x"; // tag for logging

#define MAX_WAIT 10     // Maximum time to wait for I2C operations to complete
#define DELAY_PADDING 1 // Delay padding for sensor read operations

#define G_POLYNOM 0x31 // CRC-8 polynomial

#define CMD_SOFT_RESET 0x94
#define CMD_READ_SERIAL 0x89

SHT4X::SHT4X(i2c_master_bus_handle_t *bus_handle, i2c_device_config_t *dev_config)
{
  this->bus_handle = bus_handle;
  ESP_ERROR_CHECK(i2c_master_bus_reset(*this->bus_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(*this->bus_handle, dev_config, &dev_handle));
}

SHT4X::~SHT4X()
{
  i2c_master_bus_rm_device(this->dev_handle);
}

bool SHT4X::begin(bool initialize)
{
  // Check if the device ack's over I2C
  if (isConnected() == false)
  {
    // There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (isConnected() == false)
      return (false);
  }

  bool result = true; // Accumulate a result as we do the setup

  if (initialize)
  {
    ESP_LOGI(TAG, "Initializing SHT4X");

    // Reset the SHT4X
    result &= reset();

    // Read the serial number
    result &= readSerialNumber() == ESP_OK;

    // Reset the SHT4X
    result &= reset();
  }

  return (result);
}

esp_err_t SHT4X::readSerialNumber()
{
  uint8_t cmd = CMD_READ_SERIAL;
  sht4x_data_t data;

  ESP_ERROR_CHECK(write(cmd));
  delay(cmd);
  ESP_ERROR_CHECK(read(cmd, data));

  this->serialNumber = (uint32_t)data[0] << 24; // MSB
  this->serialNumber |= (uint32_t)data[1] << 16;
  this->serialNumber |= (uint32_t)data[3] << 8;
  this->serialNumber |= (uint32_t)data[4]; // LSB

  ESP_LOGI(TAG, "Serial Number: %lu", this->serialNumber);
  return ESP_OK;
}

esp_err_t SHT4X::read(uint8_t cmd, sht4x_data_t &data)
{
  ESP_LOGD(TAG, "Reading SHT4X");
  uint8_t _buffer[6];

  isConnected(); // Check if the device is connected
  ESP_ERROR_CHECK(i2c_master_receive(this->dev_handle, _buffer, sizeof(_buffer), MAX_WAIT));

  if (_buffer[2] != calculate8BitCRC(_buffer, 2) || _buffer[5] != calculate8BitCRC(_buffer + 3, 2))
  {
    ESP_LOGE(TAG, "Invalid CRC");
    return ESP_ERR_INVALID_CRC;
  }

  data[0] = _buffer[0];
  data[1] = _buffer[1];
  data[2] = _buffer[3];
  data[3] = _buffer[4];

  return ESP_OK;
}

esp_err_t SHT4X::write(uint8_t cmd)
{
  ESP_LOGD(TAG, "Writing SHT4X");
  isConnected(); // Check if the device is connected
  uint8_t reg[1] = {cmd};
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, reg, sizeof(reg), MAX_WAIT));
  return ESP_OK;
}

// Reset the SHT4X
// Returns true if reset is successful
bool SHT4X::reset()
{
  ESP_LOGD(TAG, "Resetting SHT4X");
  uint8_t reg[1] = {CMD_SOFT_RESET};
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, reg, sizeof(reg), MAX_WAIT));
  delay(CMD_SOFT_RESET);
  return (true);
}

// Returns true if device is present
// Tests for device ack to I2C address
bool SHT4X::isConnected()
{
  ESP_ERROR_CHECK(i2c_master_probe(*this->bus_handle, SHT4X_I2CADDR_DEFAULT, MAX_WAIT)); // probe device
  return (true);                                                                         // All good
}

/**
 * @brief Measure temperature and humidity in Celsius and %RH using the
 * conversions specified in the datasheet.
 *
 * t_degC = -45 + 175 * t_ticks/65535
 * rh_pRH = -6 + 125 * rh_ticks/65535
 */
esp_err_t SHT4X::measure(sht4x_cmd_t cmd, float *temperature, float *humidity)
{
  sht4x_data_t raw_data;

  ESP_ERROR_CHECK(write((uint8_t)cmd));
  delay((uint8_t)cmd);
  ESP_ERROR_CHECK(read((uint8_t)cmd, raw_data));

  *temperature = ((uint16_t)raw_data[0] << 8 | raw_data[1]) * 175.0 / 65535.0 - 45.0;
  *humidity = ((uint16_t)raw_data[2] << 8 | raw_data[3]) * 125.0 / 65535.0 - 6.0;

  return ESP_OK;
}

void SHT4X::delay(uint8_t cmd)
{
  vTaskDelay(getDelay(cmd) + DELAY_PADDING / portTICK_PERIOD_MS);
}

uint16_t SHT4X::getDelay(uint8_t cmd)
{
  switch (cmd)
  {
  case CMD_MEAS_HIGHREP_LONG:
  case CMD_MEAS_MEDREP_LONG:
  case CMD_MEAS_LOWREP_LONG:
    ESP_LOGD(TAG, "Long delay");
    return 1010;
  case CMD_MEAS_HIGHREP_SHORT:
  case CMD_MEAS_MEDREP_SHORT:
  case CMD_MEAS_LOWREP_SHORT:
    ESP_LOGD(TAG, "Short delay");
    return 110;
    break;
  default:
    ESP_LOGD(TAG, "Default delay");
    return 10;
  }
}

// Return the serial number of the sensor
uint32_t SHT4X::getSerialNumber()
{
  return serialNumber;
}

uint8_t SHT4X::calculate8BitCRC(uint8_t data[], size_t len)
{
  uint8_t crc = 0xff;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (size_t i = 0; i < 8; i++)
      crc = crc & 0x80 ? (crc << 1) ^ G_POLYNOM : crc << 1;
  }
  return crc;
}