ESP32 IDF SHT-4x Library
===========================================================

 [![Sensirion SHT4x](https://cdn-shop.adafruit.com/970x728/5665-01.jpg)](https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf) 

 This SHT-4x ESP32 IDF driver allows you to connect the Sensirion SHT4x sensor to easily read temperature and relative humidity.

A breakout board for prototyping is avaialbe from:

- [Adafruit SHT45 - STEMMA QT / Qwiic](https://www.adafruit.com/product/5665)
- [SparkFun SHT4x](https://www.sparkfun.com/) *DISCONTINUED*

Thanks to for code on which this driver is based: 

- [Ruslan V. Uss](https://github.com/UncleRus)
- [Sensirion](https://github.com/Sensirion/embedded-i2c-sht4x)

Repository Contents
-------------------

- **/examples** - Example code for the library.
- **/licenses** - Attributions.

Documentation
--------------

- **[IDF Component Registry](https://components.espressif.com)** - Espressif Component Registry
- **[IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/en/latest/index.html)** - Information on the Espressif IDF Component Manager.
- **[Product documentation](https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf)** - Datasheet including I2C documentation.

Notable Changes & Improvements
------------------------------

- **[Espressif IoT Development Framework](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)** - based on v5 of the ESP-IDF.
- **[Utilizes the new I2C master/slave drivers](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html)**

License Information
-------------------

This library is _**open source**_! 

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.
