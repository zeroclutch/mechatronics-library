#include "logger.hpp"

Logger::Logger(bool isDebugMode, bool enableSerial) {
    this->isDebugMode = isDebugMode;
    this->logSerial = enableSerial;
}

Logger::~Logger() {
    dump();
}

void Logger::dump(void) {
    #if DEBUG_ROBOT == 1
        if(Serial && !logSerial) {
            Serial.print(output);
        }
    #endif
}

char* Logger::getTime(void) {
    char* s = (char*) malloc(10);
    sprintf(s, "[%lu] ", millis() );
    return s;
}