#include "I2CController.hpp"

I2CController* I2CController::instance = NULL;

I2CController::I2CController(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
    this->clk_speed = clk_speed;
    this->m_address = address;
}

esp_err_t I2CController::begin() {
    if(is_initialized)
    {
        #if DEBUG_I2C_CONTROLLER
            ESP_LOGI(TAG, "I2C controller already initialized");
        #endif
        return ESP_OK;
    }
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_flags = 0;

    i2c_param_config(i2c_master_port, &conf);

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Initializing I2C controller");
    #endif

    is_initialized = true;

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

I2CController::~I2CController()
{
    i2c_driver_delete(I2C_MASTER_NUM);
}

esp_err_t I2CController::readByte(uint8_t address, uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(address, rx_buffer, reg, 1, restart);
}

esp_err_t I2CController::readWord(uint8_t address, uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(address, rx_buffer, reg, 4, restart);
}

esp_err_t I2CController::read(uint8_t address, uint8_t *rx_buffer, uint8_t reg, uint8_t len, bool restart)
{
    uint8_t ret;
    write_buffer[0] = reg;

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Reading %d bytes from register 0x%02x in address 0x%02x", len, reg, address);
    #endif

    if(!restart)
    {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, address, write_buffer, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret write: %d", ret);
        vTaskDelay(1);
        ret |= i2c_master_read_from_device(I2C_MASTER_NUM, address, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret read: %d", ret);
    }
    else
    {
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, address, write_buffer, 1, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret Write_Read: %d", ret);
    }
    return ret;
}

esp_err_t I2CController::writeByte(uint8_t address, uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(address, tx_buffer, reg, 1);
}

esp_err_t I2CController::writeWord(uint8_t address, uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(address, tx_buffer, reg, 4);
}

esp_err_t I2CController::write(uint8_t address, uint8_t *tx_buffer, uint8_t reg, uint8_t len)
{
    int ret;
    
    // Fill the write buffer with the register address and data
    write_buffer[0] = reg;
    memcpy(write_buffer + 1, tx_buffer, len);

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%02x", len, reg);
    #endif
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, address, write_buffer, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    return ret;
}

void I2CController::setSDAPin(gpio_num_t sda_pin)
{
    if(this->sda_pin != sda_pin) {
        #ifdef DEBUG_I2C_CONTROLLER
            ESP_LOGI(TAG, "Different SDA pin detected, reinitializing I2C controller");
        #endif
        is_initialized = false;
        this->sda_pin = sda_pin;
    }
}

void I2CController::setSCLPin(gpio_num_t scl_pin)
{
    if(this->scl_pin != scl_pin) {
        #ifdef DEBUG_I2C_CONTROLLER
            ESP_LOGI(TAG, "Different SCL pin detected, reinitializing I2C controller");
        #endif
        is_initialized = false;
        this->scl_pin = scl_pin;
    }
}

I2CController* I2CController::getInstance()
{
    if(instance == NULL)
    {
        instance = new I2CController(0x00, CONFIG_I2C_SDA, CONFIG_I2C_SCL, I2C_MASTER_FREQ_HZ);
    }
    return instance;
}
