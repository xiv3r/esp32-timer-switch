# Requirements
- 1 pcs esp32 32 pins
- 2 8 channel relay
- 1 ds3231 rtc module + cr2038 battery
- 20 F-F Dupont Wires

# Installation
> Download and Flash the firmware to offset `0x0`
- https://github.com/xiv3r/esp32-timer-switch/raw/refs/heads/main/esp32-dump-0x0.bin

# Wifi Key
- SSID: `ESP32_Timer_Switch_XXXX`
- PASSWORD: `12345678`

## 16 CHANNEL RELAY GPIO Connection 
```
2x8CH  |  ESP32 38P
VCC _____ 5V
IN8 _____ GPIO23  Relay 1
IN7 _____ GPIO32  Relay 2
IN6 _____ GPIO33  Relay 3
IN5 _____ GPIO25  Relay 4
IN4 _____ GPIO26  Relay 5
IN3 _____ GPIO27  Relay 6
IN2 _____ GPIO14  Relay 7
IN1 _____ GPIO13  Relay 8
GND _____ GND

VCC _____ 5V
IN1 _____ GPIO1   Relay 9  (TX)
IN2 _____ GPIO3   Relay 10 (RX)
IN3 _____ GPIO19  Relay 11
IN4 _____ GPIO18  Relay 12
IN5 _____ GPIO5   Relay 13
IN6 _____ GPIO4   Relay 14
IN7 _____ GPIO2   Relay 15
IN8 _____ GPIO15  Relay 16
GND _____ GND
```

## DS3231 GPIO Connection 
```
DS3231 |  ESP32 38P
VCC _____ 3.3V
SDA _____ 21
SCL _____ 22
GND _____ GND
```
