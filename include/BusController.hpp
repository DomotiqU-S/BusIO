#ifndef BUSCONTROLLER_H
#define BUSCONTROLLER_H

#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} DevHandle_t;

class BusController {
private:
public:
    virtual esp_err_t begin() = 0;
    
    virtual esp_err_t readByte(DevHandle_t device, uint8_t *rx_buffer) = 0;
    virtual esp_err_t readWord(DevHandle_t device, uint8_t *rx_buffer) = 0;
    virtual esp_err_t read(DevHandle_t device, uint8_t *rx_buffer, uint8_t len) = 0;
    
    virtual esp_err_t writeByte(DevHandle_t device, uint8_t *tx_buffer) = 0;
    virtual esp_err_t writeWord(DevHandle_t device, uint8_t *tx_buffer) = 0;
    virtual esp_err_t write(DevHandle_t device, uint8_t *tx_buffer, uint8_t len) = 0;
};

#endif