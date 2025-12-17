#include <Arduino.h>
#include <pins.h>

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
}

void loop() {
    Serial.println("Hello from ESP32-P4!");
    delay(100);
}