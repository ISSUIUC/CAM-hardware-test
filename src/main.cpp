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

    while(!Serial) {};
    delay(50);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    if (!radio.init()) {
        Serial.println("Radio failed to init");
        while(1) {};

    }

    if(!radio.setFrequency(430)) {
        Serial.println("Failed to set radio freq");
        while(1) {};
    }


    Serial.println("Radio set up!");
    radio.setTxPower(0x7F);

}

typedef struct {
    uint8_t a;
    uint8_t b;
} packet_t;

void loop()
{
    #ifdef IS_EAGLE

    if(radio.available()) {
        uint8_t buf[8];
        uint8_t len = sizeof(packet_t);
        if(radio.recv(buf, &len)) {
            packet_t in_pkt;
            memcpy(&in_pkt, buf, sizeof(packet_t));
            Serial.println("Received!");
            Serial.print(in_pkt.a);
            Serial.print("  ");
            Serial.print(in_pkt.b);
            Serial.println(";");
        }



        Serial.println("rdy");
    //     uint8_t buf[10];
    //     memcpy(buf, &a, sizeof(packet_t));
    //     radio.send(buf, sizeof(packet_t));

    //     Serial.print("Sending...");
    //     if(radio.waitPacketSent(200)) {
    //         Serial.println("Sent successfully!");
    //         digitalWrite(LED_PIN, HIGH);
    //         delay(50);
    //         digitalWrite(LED_PIN, LOW);
    //         delay(50);
    //     }

    }

    // Serial.println("loop");
    // SPI.transfer(0xAF);
    delay(2);
    #endif





    #ifdef IS_CAM

    packet_t a;

    a.a = 10;
    a.b = 20;
    uint8_t buf[8];
    memcpy(buf, &a, sizeof(packet_t));

    radio.send(buf, sizeof(packet_t));
    if(radio.waitPacketSent(200)) {
        Serial.println("Packet sent!");
        digitalWrite(LED_PIN, HIGH);
        delay(20);
        digitalWrite(LED_PIN, LOW);
        delay(20);

    } else {
        Serial.println("Packet send fail");
    }

    delay(2);
    #endif
}