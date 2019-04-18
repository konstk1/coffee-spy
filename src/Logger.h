/**
 * @brief Logging utility.
 * 
 * @file Logger.h
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */
#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <string>

#include "esp_log.h"

#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_RESET_COLOR   "\033[0m"

// #define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)
// #define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
// #define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
// #define LOG_COLOR_D
// #define LOG_COLOR_V

class Logger {
public:
    enum class Level {
        ERROR = 0,
        WARN,
        INFO,
        DEBUG,
        VERBOSE,
    };

    Logger(const std::string &tag, Logger::Level level = Logger::Level::VERBOSE) : mTag(tag), mMinLevel(level) {};
    ~Logger() = default;

    // remove copy ctr and assignment
    Logger(const Logger &) = delete;
    Logger & operator=(const Logger &) = delete;

    template<typename... Args>
    void Error(const char *format, Args&& ...args) const {
        Write<Logger::Level::ERROR>(format, std::forward<Args>(args)...);
    };
    template<typename... Args>
    void Warn(const char *format, Args&& ...args) const {
        Write<Logger::Level::WARN>(format, std::forward<Args>(args)...);
    };
    template<typename... Args>
    void Info(const char *format, Args&& ...args) const {
        Write<Logger::Level::INFO>(format, std::forward<Args>(args)...);
    };
    template<typename... Args>
    void Debug(const char *format, Args&& ...args) const {
        Write<Logger::Level::DEBUG>(format, std::forward<Args>(args)...);
    };
    template<typename... Args>
    void Verbose(const char *format, Args&& ...args) const {
        Write<Logger::Level::VERBOSE>(format, std::forward<Args>(args)...);
    };

    void SetMinLevel(Logger::Level level) { mMinLevel = level; };
    Logger::Level GetMinLevel() const { return mMinLevel; };

    static void SetGlobalLevel(Logger::Level level) { mGlobalLevel = level; };
    static Logger::Level GetGlobalLevel() { return mGlobalLevel; };

private:
    const std::string mTag;
    Logger::Level mMinLevel;

    static Logger::Level mGlobalLevel;

    template <Logger::Level level, typename... Args>
    void Write(const char *format, Args &&... args) const {
        // if log level is below local level or global level, don't log
        if (level > mMinLevel || level > mGlobalLevel) {
            return;
        }

        constexpr auto color = LevelColor(level);
        constexpr auto prefix = LevelPrefix(level);

        printf("%s%s (%d) %s: ", color, prefix, esp_log_timestamp(), mTag.c_str());
        printf(format, std::forward<Args>(args)...);
        printf("\n" LOG_RESET_COLOR);

        fflush(stdout);
    };

    constexpr static auto LevelPrefix(Logger::Level level) {
        switch (level) {
            case Logger::Level::ERROR:
                return "E";
            case Logger::Level::WARN:
                return "W";
            case Logger::Level::INFO:
                return "I";
            case Logger::Level::DEBUG:
                return "D";
            case Logger::Level::VERBOSE:
                return "V";
            default:
                return "X";
        }
    };

    constexpr static auto LevelColor(Logger::Level level) {
        switch(level) {
            case Logger::Level::ERROR:
                return LOG_COLOR(LOG_COLOR_RED);
            case Logger::Level::WARN:
                return LOG_COLOR(LOG_COLOR_BROWN);
            case Logger::Level::INFO:
                return LOG_COLOR(LOG_COLOR_GREEN);
            default:
                return "";
        }
    }
};

#endif
