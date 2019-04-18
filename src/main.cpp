/**
 * @brief Main for coffee-spy.
 * 
 * @file main.cpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "Logger.h"

static const Logger log("MAIN");

extern "C" {

void app_main() {
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    std::cout << "========================================" << std::endl;
    std::cout << "* Welcome to Coffee-Spy" << std::endl;
    std::cout << "*" << std::endl;
    std::cout << "* ESP32 Chip Rev: " << static_cast<int>(chipInfo.revision) << " (" << static_cast<int>(chipInfo.cores) << " cores)" << std::endl;
    std::cout << "* ESP32 IDF Ver:  " << esp_get_idf_version() << std::endl;
    std::cout << "* Flash size:     " << spi_flash_get_chip_size() / 1048576 << " MB" << std::endl;
    std::cout << "========================================" << std::endl;

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}

}