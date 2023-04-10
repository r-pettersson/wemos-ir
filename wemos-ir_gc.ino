/* IRremoteESP8266: IRsendDemo - demonstrates sending IR codes with IRsend.
 *
 * Version 1.1 January, 2019
 * Based on Ken Shirriff's IrsendDemo Version 0.1 July, 2009,
 * Copyright 2009 Ken Shirriff, http://arcfn.com
 *
 * An IR LED circuit *MUST* be connected to the ESP8266 on a pin
 * as specified by kIrLed below.
 *
 * TL;DR: The IR LED needs to be driven by a transistor for a good result.
 *
 * Suggested circuit:
 *     https://github.com/crankyoldgit/IRremoteESP8266/wiki#ir-sending
 *
 * Common mistakes & tips:
 *   * Don't just connect the IR LED directly to the pin, it won't
 *     have enough current to drive the IR LED effectively.
 *   * Make sure you have the IR LED polarity correct.
 *     See: https://learn.sparkfun.com/tutorials/polarity/diode-and-led-polarity
 *   * Typical digital camera/phones can be used to see if the IR LED is flashed.
 *     Replace the IR LED with a normal LED if you don't have a digital camera
 *     when debugging.
 *   * Avoid using the following pins unless you really know what you are doing:
 *     * Pin 0/D3: Can interfere with the boot/program mode & support circuits.
 *     * Pin 1/TX/TXD0: Any serial transmissions from the ESP8266 will interfere.
 *     * Pin 3/RX/RXD0: Any serial transmissions to the ESP8266 will interfere.
 *   * ESP-01 modules are tricky. We suggest you use a module with more GPIOs
 *     for your first time. e.g. ESP-12 etc.
 */

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "wifi.h"

// Hostname of ESP
#define WIFI_HOSTNAME "ESP-IR"

const char* MYNAME = "WEMOS_IR";

char publishString[40];

const int TICKTIME = 5000; // send message every TICKTIME milliseconds

uint16_t *code_array;
const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).

IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.

const char* SUB_TOPIC = "irWemos";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);
  
 if (strcmp(topic, "irWemos") == 0) {
   if (message.equals("SPL1200ON")){
      client.publish("irWemosStatus","SPL1200R_On");
    String ir_code_str = "38000,1,1,341,171,21,21,22,21,22,21,21,21,22,21,22,21,21,21,22,21,22,63,22,64,21,64,22,63,22,64,21,64,22,63,22,64,21,21,22,21,22,21,21,64,22,21,21,21,22,21,22,21,21,64,22,63,22,64,21,21,22,64,21,64,22,63,22,64,21,1516,341,85,22,3651,341,85,22,51040";
  //sendGCString(ir_code_str);
  irsend.sendNEC(0xFF807FUL);
   } else if((message.equals("alarmOff"))) {
    
    //  client.publish("alarmStatus1","Alarm Off");
   } 
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      
      // Once connected, publish an announcement...
      //client.publish(PUB_DISCOVERY, DISCOVERY_JSON);
      
      // ... and resubscribe
      client.subscribe(SUB_TOPIC);
      //client.subscribe(SUB_TOPIC2);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  irsend.begin();
#if ESP8266
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
#else  // ESP8266
  Serial.begin(115200, SERIAL_8N1);
#endif  // ESP8266
Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  client.subscribe(SUB_TOPIC);

  // Setup OTA
  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.begin();
}

void sendGCString(String str) {
  int16_t index;
  uint16_t count;

  // Find out how many items there are in the string.
  index = -1;
  count = 1;
  do {
    index = str.indexOf(',', index + 1);
    count++;
  } while (index != -1);

  // Now we know how many there are, allocate the memory to store them all.
  code_array = reinterpret_cast<uint16_t*>(malloc(count * sizeof(uint16_t)));
  // Check we malloc'ed successfully.
  if (code_array == NULL) {  // malloc failed, so give up.
    Serial.printf("\nCan't allocate %d bytes. (%d bytes free)\n",
                  count * sizeof(uint16_t), ESP.getFreeHeap());
    Serial.println("Giving up & forcing a reboot.");
    ESP.restart();  // Reboot.
    delay(500);  // Wait for the restart to happen.
    return;  // Should never get here, but just in case.
  }

  // Now convert the strings to integers and place them in code_array.
  count = 0;
  uint16_t start_from = 0;
  do {
    index = str.indexOf(',', start_from);
    code_array[count] = str.substring(start_from, index).toInt();
    start_from = index + 1;
    count++;
  } while (index != -1);

//#if SEND_GLOBALCACHE
  irsend.sendGC(code_array, count);  // All done. Send it.
//#endif  // SEND_GLOBALCACHE
  free(code_array);  // Free up the memory allocated.
}

void loop() {
  //Serial.println("NEC");
  //irsend.sendNEC(0xFF807FUL);
  //delay(2000);
  //Serial.println("Sony");
  //irsend.sendSony(0xa90, 12, 2);  // 12 bits & 2 repeats
  //delay(2000);
  //Serial.println("a rawData capture from IRrecvDumpV2");
  //irsend.sendRaw(rawData2, 61, 38);  // Send a raw data capture at 38kHz.
  //delay(2000);
  //Serial.println("a Samsung A/C state from IRrecvDumpV2");
  //irsend.sendSamsungAC(samsungState);
  //delay(2000);
  //Serial.println("gc");
  //String ir_code_str = "38000,1,1,341,171,21,21,22,21,22,21,21,21,22,21,22,21,21,21,22,21,22,63,22,64,21,64,22,63,22,64,21,64,22,63,22,64,21,21,22,21,22,21,21,64,22,21,21,21,22,21,22,21,21,64,22,63,22,64,21,21,22,64,21,64,22,63,22,64,21,1516,341,85,22,3651,341,85,22,51040";
  //sendGCString(ir_code_str);
  //delay(2000);
    if (!client.connected()) {
    reconnect();
  }
  client.loop();

  ArduinoOTA.handle();

  unsigned long now = millis();
  if (now - lastMsg > TICKTIME) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, MYNAME);
    sprintf(publishString,"%u",now);
    //client.publish(PUB_TICK, publishString);
    //client.publish(PUB_DISCOVERY, DISCOVERY_JSON);
    //sprintf(publishString,"%u",alarmState);
    //client.publish("alarmState", publishString);
  }
}
