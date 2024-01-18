#ifndef BUSCONTROLLER_H
#define BUSCONTROLLER_H

#include "esp_log.h"
#include "I2CController.h"
#include "SPIController.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum BusType {
    I2C,
    SPI
};

struct BusController {
    i2c_data_t i2c_controller;
    spi_controller_t spi_controller;
    uint8_t address;
    enum BusType bus_type;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t clk_speed;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sclk_pin;
    uint8_t cs_pin;
};

esp_err_t BusControllerInit(struct BusController *bus_controller);
esp_err_t BusControllerWrite(struct BusController *bus_controller, uint8_t cmd, uint8_t *data, size_t data_size);
esp_err_t BusControllerRead(struct BusController *bus_controller, uint8_t cmd, uint8_t *data, size_t data_size);

#endif