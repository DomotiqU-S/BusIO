#include "I2CController.hpp"

I2CController* I2CController::instance = NULL;

I2CController::I2CController(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
}

esp_err_t I2CController::begin() {
    if(is_initialized)
    {
        #if DEBUG_I2C_CONTROLLER
            ESP_LOGI(TAG, "I2C controller already initialized");
        #endif
        return ESP_OK;
    }
    
    i2c_bus_config.sda_io_num = this->sda_pin;
    i2c_bus_config.scl_io_num = this->scl_pin;
    i2c_bus_config.i2c_port = I2C_MASTER_NUM;

    i2c_new_master_bus(&i2c_bus_config, &bus_handle);

    return ESP_OK;
}

I2CController::~I2CController()
{
    // Free the memory allocated for the I2C controller
    i2c_del_master_bus(&bus_handle);
}

esp_err_t I2CController::write(DevHandle_t device, uint8_t *tx_buffer, uint8_t len)
{
    int ret;

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%02x", len, reg);
    #endif
    ret = i2c_master_transmit(device.i2c_dev, tx_buffer, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t I2CController::writeByte(DevHandle_t device, uint8_t *tx_buffer)
{
    return write(device, tx_buffer, 1);
}

esp_err_t I2CController::writeWord(DevHandle_t device, uint8_t *tx_buffer)
{
    return write(device, tx_buffer, 2);
}

esp_err_t I2CController::read(DevHandle_t device, uint8_t *rx_buffer, uint8_t len)
{
    int ret;

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Reading %d bytes from register 0x%02x", len, reg);
    #endif

    

    ret = i2c_master_receive(device.i2c_dev, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t I2CController::readByte(DevHandle_t device, uint8_t *rx_buffer)
{
    return read(device, rx_buffer, 1);
}

esp_err_t I2CController::readWord(DevHandle_t device, uint8_t *rx_buffer)
{
    return read(device, rx_buffer, 2);
}

void I2CController::changePins(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;

    // Update the I2C bus configuration
    this->begin();
}

I2CController* I2CController::getInstance()
{
    if(instance == NULL)
    {
        instance = new I2CController(GPIO_NUM_11, GPIO_NUM_10);
    }
    return instance;
}