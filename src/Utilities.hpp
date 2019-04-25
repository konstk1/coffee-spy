/**
 * @brief Utilities header.
 * 
 * @file Utilities.hpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#ifndef UTILITIES_H
#define UTILITIES_H

inline uint32_t swap_byte_32(uint32_t x) {
    return (((x & 0x000000ffUL) << 24) |
            ((x & 0x0000ff00UL) << 8) |
            ((x & 0x00ff0000UL) >> 8) |
            ((x & 0xff000000UL) >> 24));
}

template<class T, class E>
class Either {
public:
    Either(T value, E error): m_value(value), m_error(error) {};
    ~Either() = default;

    T getValue() const { return m_value; };
    E getError() const { return m_error; };

private:
    T m_value;
    E m_error;
};

#endif
