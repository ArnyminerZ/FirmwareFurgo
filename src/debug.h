#ifndef DEBUG__h
#define DEBUG__h

#include <Arduino.h>

#define BOLD_BLACK "\e[1;30m" // debug
#define BOLD_RED "\e[1;31m" // error
#define BOLD_YELLOW "\e[1;33m" // warning
#define BOLD_BLUE "\e[1;34m" // info
#define GREEN "\e[0;32m" // success

#define BLACK "\e[0;30m"
#define RED "\e[0;31m"
#define YELLOW "\e[0;33m"
#define BLUE "\e[0;34m"
#define GREEN "\e[0;32m"

#define RESET "\e[0m"

#define TAIL_SIZE 100

enum LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    SUCCESS = 3,
    ERROR = 4
};

struct LogMessage {
    const char* message;
    LogLevel level;
};

/** A tail of the latest 100 log messages */
static LogMessage messages[TAIL_SIZE] = {};

static void logMessage(LogLevel level, const char* message) {
    // Shift messages if we have 100
    if (messages[TAIL_SIZE - 1].message != nullptr) {
        free((void*)messages[TAIL_SIZE - 1].message);
    }
    for (int i = TAIL_SIZE - 2; i >= 0; i--) {
        messages[i + 1] = messages[i];
    }
    messages[0] = { strdup(message), level };
}

static void logMessageWithFormat(LogLevel level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    char* buffer = nullptr;
    vasprintf(&buffer, format, args);
    va_end(args);
    
    if (buffer) {
        logMessage(level, buffer);
        free(buffer);
    }
}

static void debug(const char* message) {
    Serial.printf(BOLD_BLACK "[DEBU] " BLACK "%s" RESET "\n", message);
    logMessage(DEBUG, message);
}

static void debugf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    Serial.printf(BOLD_BLACK "[DEBU] " BLACK);
    Serial.printf(format, args);
    Serial.println(RESET);
    va_end(args);
    logMessageWithFormat(DEBUG, format, args);
}

static void info(const char* message) {
    Serial.printf(BOLD_BLUE "[INFO] " BLUE "%s" RESET "\n", message);
    logMessage(INFO, message);
}

static void infof(const char* format, ...) {
    va_list args;
    va_start(args, format);
    Serial.printf(BOLD_BLUE "[INFO] " BLUE);
    Serial.printf(format, args);
    Serial.println(RESET);
    va_end(args);
    logMessageWithFormat(INFO, format, args);
}

static void warning(const char* message) {
    Serial.printf(BOLD_YELLOW "[WARN] " YELLOW "%s" RESET "\n", message);
    logMessage(WARNING, message);
}

static void success(const char* message) {
    Serial.printf(GREEN "[SUCC] " GREEN "%s" RESET "\n", message);
    logMessage(SUCCESS, message);
}

static void successf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    Serial.printf(GREEN "[SUCC] " GREEN);
    Serial.printf(format, args);
    Serial.println(RESET);
    va_end(args);
    logMessageWithFormat(SUCCESS, format, args);
}

static void error(const char* message) {
    Serial.printf(BOLD_RED "[ERRO] " RED "%s" RESET "\n", message);
    logMessage(ERROR, message);
}

static void errorf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    Serial.printf(BOLD_RED "[ERRO] " RED);
    Serial.printf(format, args);
    Serial.println(RESET);
    va_end(args);
    logMessageWithFormat(ERROR, format, args);
}

#endif