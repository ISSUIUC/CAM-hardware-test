#include <Arduino.h>
#include <pins.h>
#include <SPI.h>
#include <RH_RF24.h>
#include <RHSoftwareSPI.h>
#include <RHHardwareSPI.h>

// RHSoftwareSPI _rspi;
RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);

#ifdef IS_CAM
const int LED_PIN = 51;
#endif

#ifdef IS_EAGLE
const int LED_PIN = 22;
#endif

void setup()
{

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);

    while (!Serial)
    {
    };
    delay(50);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    while (!radio.init())
    {
        Serial.println("Radio failed to init");
        delay(500);
    }

    // if(!radio.setFrequency(430)) {
    //     Serial.println("Failed to set radio freq");
    //     while(1) {};
    // }

    Serial.println("Radio set up!");
    // radio.setTxPower(0x20);

    // radio.printRegisters();

    // while(1) {};
}

typedef struct
{
    uint8_t a;
    uint8_t b;
} packet_t;

bool led_state = false;
long t1 = 0;
long t2 = 0;
bool alternate = true;

void loop()
{
#ifdef IS_EAGLE
    // Serial.println(radio.available());

    if (radio.available())
    {
        uint8_t buf[8];
        uint8_t len = sizeof(packet_t);
        if (radio.recv(buf, &len))
        {
            packet_t in_pkt;
            memcpy(&in_pkt, buf, sizeof(packet_t));
            Serial.print("Received!  ");
            Serial.print(in_pkt.a);
            Serial.print("  ");
            Serial.print(in_pkt.b);
            Serial.println(";");
        }

    }

    // Serial.println("loop");
    // SPI.transfer(0xAF);
    delay(1);
#endif

#ifdef IS_CAM

    packet_t a;

    a.a = 10;
    a.b = 20;
    uint8_t buf[8];
    memcpy(buf, &a, sizeof(packet_t));

    // Serial.println("Attemping to queue msg..");
    if (radio.send(buf, sizeof(packet_t)))
    {
        // Serial.println("MSG queued correctly");
    }
    if (radio.waitPacketSent(200))
    {
        // Serial.println("Packet sent!");
        Serial.print('.');
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
    }
    else
    {
        // digitalWrite(LED_PIN, HIGH);
        // delay(20);
        // digitalWrite(LED_PIN, LOW);
        // delay(20);
        Serial.println("Packet send fail");
    }

    delay(1);
#endif
}