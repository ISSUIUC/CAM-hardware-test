#include <Arduino.h>
#include <pins.h>
#include <Si446x/Si446x.h>

const int LED_PIN = 51;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);

    Si446x_init();
    Si446x_setTxPower(SI446X_MAX_TX_POWER);
}

void loop()
{
    Serial.println("HIII");
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}