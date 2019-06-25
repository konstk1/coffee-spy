/**
 * @brief MAX31855 driver implementation.
 * 
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html
 * https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Mode_numbers
 * 
 * @file MAX31855.cpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#include <string.h>

#include "driver/spi_master.h"

#include "Logger.hpp"
#include "Utilities.hpp"
#include "MAX31855.hpp"

static Logger log("SPI", Logger::Level::VERBOSE);

typedef struct __packed {
    uint8_t openCircuitFault : 1;
    uint8_t shortToGndFault  : 1;
    uint8_t shortToVccFault  : 1;
    uint8_t reserved1        : 1;
    uint16_t internalTemp    : 12;
    bool    fault            : 1;
    uint8_t reserved2        : 1;
    uint16_t tcTemp          : 14;
} max31855_data_t;

int MAX31855::mNumDevices = 0;

MAX31855::MAX31855(int chipSelectPin): mCsPin(chipSelectPin) {
    // only init SPI once
    if (mNumDevices == 0) {
        log.Info("Initializing VSPI (MISO %d CLK %d)", mDataPin, mClkPin);
        constexpr spi_bus_config_t busCfg = {
            .mosi_io_num = -1,
            .miso_io_num = mDataPin,
            .sclk_io_num = mClkPin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
            .flags = 0,
            .intr_flags = 0,
        };

        // initialize the SPI bus
        esp_err_t status = spi_bus_initialize(VSPI_HOST, &busCfg, 1);
        if (status != ESP_OK) {
            throw "Failed SPI BUS init";
        }
    }

    const spi_device_interface_config_t devCfg = {
        .command_bits     = 0,                      // no command
        .address_bits     = 0,                      // no address
        .dummy_bits       = 0,                      // no dummy bits
        .mode             = 0,                      // mode 0: CPOL=0, CPHA=0
        .duty_cycle_pos   = 0,                      // 50% duty cycle
        .cs_ena_pretrans  = 0,                      // no cycles before trans
        .cs_ena_posttrans = 0,                      // no cycles after trans
        .clock_speed_hz   = SPI_MASTER_FREQ_10M,    // 10Mhz clock
        .input_delay_ns   = 0,                      
        .spics_io_num     = mCsPin,                  // CS pin
        .flags            = 0,
        .queue_size       = 1,                      // 1 command at a time
        .pre_cb           = nullptr,
        .post_cb          = nullptr,
    };

    //Attach the LCD to the SPI bus
    esp_err_t status = spi_bus_add_device(VSPI_HOST, &devCfg, &mSpiDevice);
    if (status != ESP_OK) {
        throw "Failed to add SPI device";
    }

    ++mNumDevices;
    log.Verbose("Added SPI Dev (CS %d)", mCsPin);
}

MAX31855::~MAX31855() {
    esp_err_t status = spi_bus_remove_device(mSpiDevice);
    if (status != ESP_OK) {
        log.Error("Failed to remove device (CS %d)", mCsPin);
    }

    log.Verbose("Removed MAX31855 (CS %d)", mCsPin);
    
    --mNumDevices;

    if (mNumDevices == 0) {
        log.Verbose("No more devices, freeing VSPI Bus");
        spi_bus_free(VSPI_HOST);
    }
}

Either<int, MAX31855::Error> MAX31855::ReadTemp() const {
    // create 32-bit rx-only transaction putting data directly into transaction struct
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags    = SPI_TRANS_USE_RXDATA;
    t.length   = 32;
    t.rxlength = 32;

    esp_err_t status = spi_device_polling_transmit(mSpiDevice, &t);
    if (status != ESP_OK) {
        log.Error("Failed SPI transmit: %s", esp_err_to_name(status));
        return Either<int, MAX31855::Error>(0, MAX31855::Error::SPI_ERROR);
    }

    uint32_t rawData = SPI_SWAP_DATA_RX(*(uint32_t*)t.rx_data, 32);  // byte-swap RX data

    if (rawData == 0) {
        log.Error("SPI device not responding");
        return Either<int, MAX31855::Error>(0, MAX31855::Error::SPI_ERROR);
    }

    max31855_data_t *data = reinterpret_cast<max31855_data_t *>(&rawData);
    
    log.Debug("Data (0x%08X): TC %d F %d IC %d SCV %d SCG %d OC %d", rawData, (int)data->tcTemp, (int)data->fault, (int)data->internalTemp,
        (int)data->shortToVccFault, (int)data->shortToGndFault, (int)data->openCircuitFault);
    log.Debug("Data HEX %02X %02X %02X %02X", t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);

    if (data->fault) {
        log.Error("Data fault (SCV %d SCG %d OC %d)", (int)data->shortToVccFault, (int)data->shortToGndFault, (int)data->openCircuitFault);
        MAX31855::Error error = data->shortToGndFault ? MAX31855::Error::TC_SCG : (
                                data->shortToVccFault ? MAX31855::Error::TC_SCV : MAX31855::Error::TC_OPEN);
        return Either<int, MAX31855::Error>(0, error);
    }

    return Either<int, MAX31855::Error>(data->tcTemp, MAX31855::Error::OK) ;
}

Either<int, MAX31855::Error> MAX31855::ReadTempC() const {
    auto result = ReadTemp();

    if (result.getError() != Error::OK) {
        return result;
    }

    int tempC = result.getValue() >> 2; // convert from LSB (0.25C) to C

    return Either<int, Error>(tempC, Error::OK);
}