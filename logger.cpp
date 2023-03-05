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
    char* s = (char*) malloc(10);
    sprintf(s, "[%lu] ", millis() );
    return s;
}