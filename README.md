# BusIO library

## Installation
This repo can be use to program sensor that uses I2C or SPI. The BusIO struct is an abstraction struct to communicate with the sensor.
If you want to use this library just add it to your CMakeList in the file like so:

```
idf_component_register(SRCS "I2CController.cpp" "SPIController.cpp"
                    INCLUDE_DIRS ".")
```
