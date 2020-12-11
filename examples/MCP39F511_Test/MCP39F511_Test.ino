#include "MCP39F511N.h"

// TODO: This section only supports ESP32 and anything else with a software serial
// library. This example can be made more fully fledged with a stream initialization
// helper library. Given the sheer breadth of the modern Arduino ecosystem it should
// be left to the actual sketch writer to modify to suit their async serial stream
// implementation
#if defined(ESP32)
HardwareSerial Serial1(2);
#else
#import <SoftwareSerial.h>
#define RX_PIN 10
#define TX_PIN 11
SoftwareSerial Serial1(RX_PIN, TX_PIN, false, MCP_BUFFER_LEN);
#endif

MCP39F511N mcp_emon = MCP39F511N(&Serial1);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello MCP39F511");
  mcp_emon.readConfig();
}
