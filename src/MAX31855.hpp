/**
 * @brief MAX31855 driver header.
 * 
 * @file MAX31855.hpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#ifndef MAX31855_H
#define MAX31855_H

#include "driver/spi_master.h"

#include "Utilities.hpp"

class MAX31855 final {
public:
    enum class Error {
        OK = 0,
        SPI_ERROR,
        TC_OPEN,
        TC_SCG,
        TC_SCV,
    };

    /**
     * @brief Constructs a new MAX31855 object.
     * 
     * @param dataPin The GPIO # of data line (MOSI)
     * @param clkPin The GPIO # of clock line (CLK)
     * @param csPin The GPIO # of chip select line (CS)
     */
    MAX31855(int dataPin = 19, int clkPin = 18, int csPin = 5);

    /**
     * @brief Destroys the MAX31855 object.
     * 
     */
    ~MAX31855();

    // remove copy ctor and assignment
    MAX31855(const MAX31855 &) = delete;
    MAX31855 & operator=(const MAX31855 &) = delete;

    /**
     * @brief Read thermocouple temperature.
     * 
     * @return Maybe<int, MAX31855::Error> Result with temperature in Celsius or error code
     */
    Maybe<int, MAX31855::Error> ReadTempC() const;

private:
    spi_device_handle_t m_spi;
};

#endif