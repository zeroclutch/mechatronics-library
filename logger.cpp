#include "logger.hpp"

Logger::Logger(bool isDebugMode, bool enableSerial) {
    this->isDebugMode = isDebugMode;
    this->logSerial = enableSerial;
}

Logger::~Logger() {
    dump();
}

void Logger::dump(void) {
    if(isDebugMode) { 
        if(Serial && !logSerial) {
            Serial.print(output);
        }
    }
}

char* Logger::getTime(void) {
    int size = snprintf(nullptr, 0, "[%lu] ", millis() );
    if (size < 0) {
        return nullptr;
    }

    char* s = (char*) malloc(size + 1);
    snprintf(s, size + 1, "[%lu] ", millis() );
    return s;
}