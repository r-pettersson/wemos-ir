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
 *
 * Author: Roger
 * Date: 
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

const int TICKTIME = 5000;  // send message every TICKTIME milliseconds

uint16_t* code_array;
const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).

IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.

const char* SUB_TOPIC = "irWemos";

typedef struct {
  const char* remote_cmd;
  const int code_cmd;
  //char* code_value;
} ir_cmds_s;

int ir_num_codes = 16;

ir_cmds_s ir_cmd[16] = {
	{"SPL1200ULTRA_POWER",		0xFF807F}, 
	{"SPL1200ULTRA_POWER_ON",	0xFF20DF}, 
	{"SPL1200ULTRA_POWER_OFF",	0xFFE01F},
	{"SPL1200ULTRA_VOL_UP",		0xFFD02F},
	{"SPL1200ULTRA_VOL_DOWN",   0xFFF00F},
	{"SPL1200ULTRA_LIGHT_ON",   0xFF28D7},
	{"SPL1200ULTRA_LIGHT_OFF",	0xFFA857},
	{"SPL1200ULTRA_MUTE_ON",	0xFFB04F}, 
	{"SPL1200ULTRA_MUTE_OFF",	0xFF708F}, 
	{"SPL1200ULTRA_NIGHT_ON",	0xff6897},
	{"SPL1200ULTRA_NIGHT_OFF",	0xffe817},
	{"SPL1200ULTRA_PHASE_0",	0xffc03f},
	{"SPL1200ULTRA_PHASE_90",	0xff609f},
	{"SPL1200ULTRA_PHASE_180",	0xff50af},
	{"SPL1200ULTRA_PHASE_270",	0xffa05f}, 
	{"SPL1200ULTRA_EQ",	0xff00ff},
};

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
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


  if (message.startsWith("SPL1200"))  //kolla ifall det är SPL1200*
  {

    for (int i = 0; i < ir_num_codes; i++) {
      if (message.equals(ir_cmd[i].remote_cmd)) {
        Serial1.println(message);
        //irsend.sendNEC(0xFF807F,32);
        irsend.sendNEC(ir_cmd[i].code_cmd, 32);
        client.publish("subwooferstate", "SPL1200ULTRA_ON");
      }
    }
    //irsend.sendNEC(SPL1200ULTRA_POWER_ON,32);
    //Serial.println(SPL1200ULTRA_POWER_ON);

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
#else   // ESP8266
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

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  ArduinoOTA.handle();

  unsigned long now = millis();
  if (now - lastMsg > TICKTIME) {
    lastMsg = now;
    ++value;
    snprintf(msg, MSG_BUFFER_SIZE, MYNAME);
    sprintf(publishString, "%u", now);
    //client.publish(PUB_TICK, publishString);
    //client.publish(PUB_DISCOVERY, DISCOVERY_JSON);
    //sprintf(publishString,"%u",alarmState);
    //client.publish("alarmState", publishString);
  }
}
