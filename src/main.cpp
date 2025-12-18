#include <Arduino.h>
#include <pins.h>
#include <Si446x.h>

const int LED_PIN = 51;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);

    // Si446x_init();
    // Si446x_setTxPower(SI446X_MAX_TX_POWER);
}

typedef struct {
    uint8_t a;
    uint8_t b;
} packet_t;

void loop()
{
    packet_t a;

    a.a = 10;
    a.b = 20;

    // Si446x_TX(&a, sizeof(a), 20, SI446X_STATE_RX);
    Serial.println("HIII");
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}