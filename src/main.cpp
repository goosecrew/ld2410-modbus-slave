#include <Arduino.h>
#include "./wi-fi.h"
#include "./ota.h"
#include "./modbus-slave.h"
#include "./ld-2410.h"

void setup() {
  Serial.begin(9600); //для логов в консоль
  setupWiFi(); //подключаемся к вафле
  setupOTA(); //настраиваем OTA чтобы обновляться через вафлю
  eeprom_setup(); // настраиваем EEPROM
  read_modbus_settings(); //читаем настройки modbus из eeprod
  modbus_setup();  // настраиваем UART для Modbus
  ld2410_setup(); //настраиваем UART для Serial
}

void loop() {
  pollLD2410(); //читаем показания датчика
  mbTask(); //работаем с modbus
}
