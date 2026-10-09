// Bounded work per loop; discard the ENTIRE malformed line, including suffixes.
char commandBuffer[32];
byte commandLength = 0;
bool discardCommand = false;
void serviceSerialCommands() {
  byte budget = 32;
  while (budget-- && Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!discardCommand && commandLength) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
      }
      commandLength = 0;
      discardCommand = false;
    } else if (!discardCommand) {
      if (c < 32 || c > 126 || commandLength >= sizeof(commandBuffer) - 1) {
        discardCommand = true;
        commandLength = 0;
        Serial.println(F("ERROR,BAD_LINE"));
      } else commandBuffer[commandLength++] = c;
    }
  }
}
