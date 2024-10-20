#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H

#include "BusController.hpp"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"


#define I2C_MASTER_NUM              (i2c_port_t)0   /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C timeout in milliseconds */
// #define DEBUG_I2C_CONTROLLER        1               /*!< Enable debug logs for the I2C controller */

static const char *TAG = "I2C_CONTROLLER";

class I2CController : public BusController {
private:
    uint8_t m_address;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t clk_speed;

    uint8_t write_buffer[32];
    bool is_initialized = false;

    i2c_master_bus_config_t i2c_bus_config;
    i2c_master_handle_t bus_handle;

    static I2CController* instance;
protected:
    /**
     * @brief Constructor for the I2CController class.
     * 
     * This constructor initializes the I2C controller with the given parameters.
     * 
     * @param address The I2C address of the slave device.
     * @param sda_pin The GPIO pin number for the SDA line.
     * @param scl_pin The GPIO pin number for the SCL line.
     * @param clk_speed The clock speed of the I2C bus.
     * 
     * @return N/A
    */
    I2CController(gpio_num_t sda_pin, gpio_num_t scl_pin);
    
    /**
     * @brief Destructor for the I2CController class.
     * 
     * This destructor frees the memory allocated for the I2C controller.
     * 
     * @return N/A
    */
    ~I2CController();
public:

    I2CController(I2CController const&) = delete;
    /**
     * @brief Gets the instance of the I2C controller.
     * 
     * This function returns the instance of the I2C controller.
     * 
     * @return The instance of the I2C controller.
    */
    static I2CController* getInstance();

    void operator=(I2CController const&) = delete;

    /**
    * @brief Initializes the I2C controller.
    * 
    * This function initializes the I2C controller with the given parameters.
    * 
    * @return ESP_OK if the I2C controller is initialized successfully, ESP_FAIL otherwise.
    */
    esp_err_t begin();

    /**
     * @brief Reads a byte from the slave device.
     * 
     * This function reads a byte from the slave device at the given register address.
     * 
     * @param rx_buffer Pointer to the buffer to store the received data.
     * @param reg The register address to read from.
     * @param restart if sensor support restart command
     * 
     * @return ESP_OK if the byte is read successfully, ESP_FAIL otherwise.
    */
    esp_err_t readByte(DevHandle_t device, uint8_t *rx_buffer);
    
    /**
     * @brief Reads a word from the slave device.
     * 
     * This function reads a word from the slave device at the given register address.
     * 
     * @param rx_buffer Pointer to the buffer to store the received data.
     * @param reg The register address to read from.
     * @param restart if sensor support restart command
     * getInstance
     * @return ESP_OK if the word is read successfully, ESP_FAIL otherwise.
    */
    esp_err_t readWord(DevHandle_t device, uint8_t *rx_buffer);
    
    /**
     * @brief Reads data from the slave device.
     * 
     * This function reads data from the slave device at the given register address.
     * 
     * @param rx_buffer Pointer to the buffer to store the received data.
     * @param reg The register address to read from.
     * @param len The length of the data to be read.
     * @param restart if sensor support restart command
     * 
     * @return ESP_OK if the data is read successfully, ESP_FAIL otherwise.
    */
    esp_err_t read(DevHandle_t device, uint8_t *rx_buffer, uint8_t len);
    
    /**
     * @brief Writes a byte to the slave device.
     * 
     * This function writes a byte to the slave device at the given register address.
     * 
     * @param tx_buffer Pointer to the buffer containing the data to be written.
     * @param reg The register address to write to.
     * 
     * @return ESP_OK if the byte is written successfully, ESP_FAIL otherwise.
    */
    esp_err_t writeByte(DevHandle_t device, uint8_t *tx_buffer);
    
    /**
     * @brief Writes a word to the slave device.
     * 
     * This function writes a word to the slave device at the given register address.
     * 
     * @param tx_buffer Pointer to the buffer containing the data to be written.
     * @param reg The register address to write to.
     * 
     * @return ESP_OK if the word is written successfully, ESP_FAIL otherwise.
    */
    esp_err_t writeWord(DevHandle_t device, uint8_t *tx_buffer);

    /**
     * @brief Writes data to the slave device.
     * 
     * This function writes data to the slave device at the given register address.
     * 
     * @param tx_buffer Pointer to the buffer containing the data to be written.
     * @param reg The register address to write to.
     * @param len The length of the data to be written.
     * 
     * @return ESP_OK if the data is written successfully, ESP_FAIL otherwise.
    */
    esp_err_t write(DevHandle_t device, uint8_t *tx_buffer, uint8_t len);

    /**
     * @brief The function to change the pins of the I2C controller.
     * 
     * @param sda_pin new SDA pin
     * @param scl_pin new SCL pin
     */
    void changePins(gpio_num_t sda_pin, gpio_num_t scl_pin);
};

#endif