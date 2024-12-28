#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <Arduino.h>
#include <SPI.h>

class SPISlave {
public:
    SPISlave(int sensorPin, int misoPin, int mosiPin, int sckPin, int ssPin);
    void begin();
    void update();
    
private:
    // Pin configurations
    const int _sensorPin;
    const int _misoPin;
    const int _mosiPin;
    const int _sckPin;
    const int _ssPin;
    
    // Data buffer
    uint8_t _spiData;
    
    // Methods
    void initializePins();
    void initializeSPI();
    bool readSensor();
};

#endif