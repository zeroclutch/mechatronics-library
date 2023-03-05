// Logger capable of dumping to a file and/or stdout
#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef DEBUG_ROBOT
#define DEBUG_ROBOT 0
#endif

#ifndef LOGGER_H
#define LOGGER_H

#include <time.h>

class Logger {
    private:
        String output;

        bool logSerial;
        char* getTime(void);
    public:
        Logger(bool enableSerial);
        ~Logger();

        template<typename... Args>
        void log(char * format, Args ...args) {
            #if DEBUG_ROBOT == 1
                // Get buffer size first
                int size = snprintf(nullptr, 0, format, args...);
                if (size < 0) {
                    return;
                }


                char * s = (char *) malloc(size + 1);
                if(!size) {
                    return;
                }

                int sn = snprintf(s, size + 1, format, args... ); 
                char* time = getTime();

                if(Serial && logSerial) {
                        Serial.print(time);
                        Serial.print(s);
                        Serial.println();
                } else {
                        output += time;
                        output += s;
                        output += "\n";
                }

                free(s);
                free(time);
            #endif
        }
        void dump(void);
};

#endif