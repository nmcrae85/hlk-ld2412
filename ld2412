#pragma once

#include "esphome.h"

// Define lowByte and highByte for ESPHome/ESP-IDF environment
#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)(((w) >> 8) & 0xFF))

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

class LD2412 : public PollingComponent, public UARTDevice {
public:
  LD2412(UARTComponent *parent) : UARTDevice(parent) {}

  BinarySensor *hasTarget = new BinarySensor();
  BinarySensor *hasMovingTarget = new BinarySensor();
  BinarySensor *hasStillTarget = new BinarySensor();
  Sensor *movingTargetDistance = new Sensor();
  Sensor *movingTargetEnergy = new Sensor();
  Sensor *stillTargetDistance = new Sensor();
  Sensor *stillTargetEnergy = new Sensor();

  int movingSensitivities[14] = {0};
  int stillSensitivities[14] = {0};

  long lastPeriodicMillis = millis();

  void sendCommand(char *commandStr, char *commandValue, int commandValueLen) {
    // Frame start bytes
    write_byte(0xFD);
    write_byte(0xFC);
    write_byte(0xFB);
    write_byte(0xFA);
    
    // Length bytes
    int len = 2;
    if (commandValue != nullptr)
      len += commandValueLen;
    write_byte(lowByte(len));
    write_byte(highByte(len));
    
    // Command string bytes
    write_byte(commandStr[0]);
    write_byte(commandStr[1]);
    
    // Command value bytes
    if (commandValue != nullptr) {
      for (int i = 0; i < commandValueLen; i++) {
        write_byte(commandValue[i]);
      }
    }
    
    // Frame end bytes
    write_byte(0x04);
    write_byte(0x03);
    write_byte(0x02);
    write_byte(0x01);
    delay(50);
  }

  int twoByteToInt(char firstByte, char secondByte) {
    return (int16_t)(secondByte << 8) + firstByte;
  }

  void handlePeriodicData(char *buffer, int len) {
    if (len < 12)
      return; // 4 frame start bytes + 2 length bytes + 1 data end byte + 1 crc byte + 4 frame end bytes
    if (buffer[0] != 0xF4 || buffer[1] != 0xF3 || buffer[2] != 0xF2 || buffer[3] != 0xF1)
      return; // Check 4 frame start bytes
    if (buffer[7] != 0xAA) 
      return; // Data head = 0xAA
    if (buffer[len - 6] != 0x55)
      return; // Data end = 0x55

    long currentMillis = millis();
    if (currentMillis - lastPeriodicMillis < 1000)
      return;
    lastPeriodicMillis = currentMillis;

    char stateByte = buffer[8];
    hasTarget->publish_state(stateByte != 0x00);

    hasMovingTarget->publish_state(CHECK_BIT(stateByte, 0));
    hasStillTarget->publish_state(CHECK_BIT(stateByte, 1));
    
    int newMovingTargetDistance = twoByteToInt(buffer[9], buffer[10]);
    if (movingTargetDistance->get_state() != newMovingTargetDistance)
      movingTargetDistance->publish_state(newMovingTargetDistance);

    int newMovingTargetEnergy = buffer[11];
    if (movingTargetEnergy->get_state() != newMovingTargetEnergy)
      movingTargetEnergy->publish_state(newMovingTargetEnergy);
    
    int newStillTargetDistance = twoByteToInt(buffer[12], buffer[13]);
    if (stillTargetDistance->get_state() != newStillTargetDistance)
      stillTargetDistance->publish_state(newStillTargetDistance);
    
    int newStillTargetEnergy = buffer[14];
    if (stillTargetEnergy->get_state() != newStillTargetEnergy)
      stillTargetEnergy->publish_state(newStillTargetEnergy);
  }

  void handleACKData(char *buffer, int len) {
    if (len < 10)
      return;
    if (buffer[0] != 0xFD || buffer[1] != 0xFC || buffer[2] != 0xFB || buffer[3] != 0xFA)
      return; // Check 4 frame start bytes
    if (buffer[7] != 0x01)
      return;
    if (twoByteToInt(buffer[8], buffer[9]) != 0x00)
      return;
  }

  void readline(int readch, char *buffer, int len) {
    static int pos = 0;

    if (readch >= 0) {
      if (pos < len - 1) {
        buffer[pos++] = readch;
        buffer[pos] = 0;
      } else {
        pos = 0;
      }
      if (pos >= 4) {
        if (buffer[pos - 4] == 0xF8 && buffer[pos - 3] == 0xF7 && buffer[pos - 2] == 0xF6 && buffer[pos - 1] == 0xF5) {
          handlePeriodicData(buffer, pos);
          pos = 0;
        } else if (buffer[pos - 4] == 0x04 && buffer[pos - 3] == 0x03 && buffer[pos - 2] == 0x02 && buffer[pos - 1] == 0x01) {
          handleACKData(buffer, pos);
          pos = 0;
        }
      }
    }
    return;
  }

  void setConfigMode(bool enable) {
    char cmd[2] = {static_cast<char>(enable ? 0xFF : 0xFE), 0x00};
    char value[2] = {0x01, 0x00};
    sendCommand(cmd, enable ? value : nullptr, 2);
  }

  void queryParameters() {
    char cmd_query[2] = {0x61, 0x00};
    sendCommand(cmd_query, nullptr, 0);
  }

  void setup() override {
    set_update_interval(15000);
  }

  void loop() override {
    const int max_line_length = 80;
    static char buffer[max_line_length];
    while (available()) {
      readline(read(), buffer, max_line_length);
    }
  }

  void setEngineeringMode(bool enable) {
    char cmd[2] = {static_cast<char>(enable ? 0x62 : 0x63), 0x00};
    sendCommand(cmd, nullptr, 0);
  }

  void factoryReset() {
    char cmd[2] = {0xA2, 0x00};
    sendCommand(cmd, nullptr, 0);
  }

  void reboot() {
    char cmd[2] = {0xA3, 0x00};
    sendCommand(cmd, nullptr, 0);
  }

  void setBaudrate(int index) {
    char cmd[2] = {0xA1, 0x00};
    char value[2] = {static_cast<char>(index), 0x00};
    sendCommand(cmd, value, 2);
  }

  void update() {
    // Your custom update logic here, if needed
  }
};
