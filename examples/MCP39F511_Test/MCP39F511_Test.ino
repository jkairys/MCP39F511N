#include "MCP39F511N.h"

#if defined(ESP32)
HardwareSerial Serial1(2);
#endif

MCP39F511N mcp_emon = MCP39F511N(&Serial1);

#define TEST_COMMAND {0xA5, 0x08, 0x41, 0x00, 0x02,  0x4E, 0x20, 0x5E}

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello MCP39F511");
  mcp_emon.readConfig();
}
