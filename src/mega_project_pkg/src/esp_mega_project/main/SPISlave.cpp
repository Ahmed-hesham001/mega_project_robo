#include "SPISlave.h"

SPISlave::SPISlave(int sensorPin, int misoPin, int mosiPin, int sckPin, int ssPin)
    : _sensorPin(sensorPin)
    , _misoPin(misoPin)
    , _mosiPin(mosiPin)
    , _sckPin(sckPin)
    , _ssPin(ssPin)
    , _spiData(0)
{
}

void SPISlave::begin() {
    Serial.begin(115200);
    initializePins();
    initializeSPI();
    Serial.println("SPI Slave initialized");
}

void SPISlave::initializePins() {
    pinMode(_sensorPin, INPUT); // Digital sensor pin
    pinMode(_misoPin, OUTPUT);
    pinMode(_mosiPin, INPUT);
    pinMode(_sckPin, INPUT);
    pinMode(_ssPin, INPUT);
}

void SPISlave::initializeSPI() {
    SPI.begin(_sckPin, _misoPin, _mosiPin, _ssPin);
    // Set up SPI in slave mode
    SPI.setClockDivider(SPI_CLOCK_DIV16); // Ignored in slave mode but kept for completeness
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
}

bool SPISlave::readSensor() {
    return digitalRead(_sensorPin); // Read digital value (HIGH or LOW)
}

void SPISlave::update() {
    if (digitalRead(_ssPin) == LOW) {
        // Read sensor state
        bool sensorState = readSensor();

        // Convert boolean state to byte (0 or 1)
        _spiData = sensorState ? 1 : 0;

        // Perform SPI transfer
        uint8_t receivedData = SPI.transfer(_spiData);

        // Log sent and received data
        Serial.print("Sent sensor state: ");
        Serial.print(_spiData);
        Serial.print(", Received data: ");
        Serial.println(receivedData);
    }
}
