/**
 * @brief Main for coffee-spy.
 * 
 * ESP32-Thing: https://learn.sparkfun.com/tutorials/esp32-thing-hookup-guide#hardware-overview
 * 
 * @file main.cpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#include <iostream>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"


#include "BLE.hpp"
#include "Logger.hpp"
#include "MAX31855.hpp"
#include "Utilities.hpp"

static const Logger log("MAIN");

extern "C" {

void app_main() {
    // initialize NVS
    esp_err_t status = nvs_flash_init();
    if (status == ESP_ERR_NVS_NO_FREE_PAGES || status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);

    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    std::cout << "========================================" << std::endl;
    std::cout << "* Welcome to Coffee-Spy" << std::endl;
    std::cout << "*" << std::endl;
    std::cout << "* ESP32 Chip Rev: " << static_cast<int>(chipInfo.revision) << " (" << static_cast<int>(chipInfo.cores) << " cores)" << std::endl;
    std::cout << "* ESP32 IDF Ver:  " << esp_get_idf_version() << std::endl;
    std::cout << "* Flash size:     " << spi_flash_get_chip_size() / 1048576 << " MB" << std::endl;
    std::cout << "========================================" << std::endl;

    // ble_init();
    log.Info("Waiting for BLE connection...");

    MAX31855 probe1(5);
    MAX31855 probe2(2);

    for (int i = 0; i < 10; i++) {
        auto result1 = probe1.ReadTempC();
        auto result2 = probe2.ReadTempC();
        if (result1.getError() != MAX31855::Error::OK) {
            log.Error("Failed to read thermocouple 1: %d", result1.getError());
        } else if (result2.getError() != MAX31855::Error::OK) {
            log.Error("Failed to read thermocouple 2: %d", result1.getError());
        } else {
            std::cout << "Temp read: 1 " << result1.getValue() << " | 2 "<< result2.getValue() << std::endl;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

}