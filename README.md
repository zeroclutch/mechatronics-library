# Mechatronics

![](https://github.com/zeroclutch/mechatronics-library/blob/main/assets/moles.gif?raw=true)

*Whacking moles*

![](https://github.com/zeroclutch/mechatronics-library/blob/main/assets/coins.gif?raw=true)

*Paying coins to play*

The software for a Whac-a-mole robot project created for a university mechatronics course. The goal of the project was to create a robot that can knock four coins into a basket and smack some moles by driving to them and hitting them. My group was able to achieve a rate of a 1 mole every 4.2 seconds, and won the class competition.

# Components

The robot is powered by an Arduino Mega 2560 and uses several sensors and actuators in order to perform the tasks required.

* Pololu QTRX reflectance sensors
* DC Motors with H-bridge
* Adafruit TCS34725 color sensor
* Sharp GP2Y0A21YK0F distance sensor
* Limit switches

Each module within this software interfaces with the sensors.