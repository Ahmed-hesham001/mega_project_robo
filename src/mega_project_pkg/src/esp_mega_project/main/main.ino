// main.ino
#include "SPISlave.h"

// Define pins
const int SENSOR_PIN = 4;     // sensor input pin
const int SPI_MISO = 19;      // Master In Slave Out
const int SPI_MOSI = 23;      // Master Out Slave In
const int SPI_SCK = 18;       // Serial Clock
const int SPI_SS = 5;         // Slave Select

// Create SPISlave instance
SPISlave spiSlave(SENSOR_PIN, SPI_MISO, SPI_MOSI, SPI_SCK, SPI_SS);

void setup() {
    spiSlave.begin();
}

void loop() {
    spiSlave.update();
    delay(100);  // Small delay to prevent overwhelming the bus
}