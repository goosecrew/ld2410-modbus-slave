#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// Настройка Modbus
#define SLAVE_ID 1 // адрес нашего сервера
#define PIN_FLOW 25 // пин контроля направления приёма/передачи,
#define RX_PIN 33
#define TX_PIN 32

#define LD2410_TX 27
#define LD2410_RX 26
#define LD2410_SPEED 256000

bool SENSOR_PRESENCE;
bool SENSOR_STATIONARY;
bool SENSOR_MOVING;                
uint16_t SENSOR_STATIONARY_TARGET_DISTANCE;
uint16_t SENSOR_STATIONARY_TARGET_ENERGY;
uint16_t SENSOR_MOVING_TARGET_DISTANCE;
uint16_t SENSOR_MOVING_TARGET_ENERGY;
uint16_t SENSOR_MAX_MOVING_GATE;
uint16_t SENSOR_MAX_STATIONARY_GATE;
uint16_t SENSOR_IDLE_TIME;

#define REG_MB_ADDRESS                          100 
#define REG_MB_STOP_BITS                        101 
#define REG_MB_BAUDRATE                         102
#define REG_SENSOR_PRESENCE                     0 
#define REG_SENSOR_STATIONARY                   1 
#define REG_SENSOR_MOVING                       2 
#define REG_SENSOR_STATIONARY_TARGET_DISTANCE   3 
#define REG_SENSOR_STATIONARY_TARGET_ENERGY     4 
#define REG_SENSOR_MOVING_TARGET_DISTANCE       5 
#define REG_SENSOR_MOVING_TARGET_ENERGY         6 
#define REG_SENSOR_MAX_MOVING_GATE              103
#define REG_SENSOR_MAX_STATIONARY_GATE          104
#define REG_SENSOR_IDLE_TIME                    105

#define EEPROM_SIZE           6 // мы займём 6 ячеек памяти: 3*2=6
#define EEPROM_MB_ADDRESS     0 // номер ячейки с адресом устройства
#define EEPROM_MB_STOP_BITS   2 // номер ячейки со стоп-битами
#define EEPROM_MB_BAUDRATE    4 // номер ячейки со скоростью

#define DEFAULT_MB_ADDRESS    1  // адрес нашего сервера
#define DEFAULT_MB_STOP_BITS  2  // количество стоповых битов
#define DEFAULT_MB_BAUDRATE   96 // скорость подключения/100

#define DEFAULT_SENSOR_MAX_MOVING_GATE 8
#define DEFAULT_SENSOR_MAX_STATIONARY_GATE 8
#define DEFAULT_SENSOR_IDLE_TIME 8

uint16_t mbAddress = DEFAULT_MB_ADDRESS; // modbus адрес устройства
uint16_t mbStopBits = DEFAULT_MB_STOP_BITS; // количество стоповых битов
uint16_t mbBaudrate = DEFAULT_MB_BAUDRATE; // скорость подключения modbus

#include <ld2410.h>

ld2410 radar;
bool engineeringMode = false;
String command;

// Номера Modbus регистров
#define REG_TEST 0       // тестовый регистр с номером 0

ModbusRTU mb;

void modbus_setup();
void eeprom_setup();
uint32_t convert_baudrate(uint16_t baudrateValue);
void  read_modbus_settings();
void ld2410_setup();
void setupWiFi();
void setupOTA();

void setup() {
  Serial.begin(9600);

  setupWiFi();
  setupOTA();

  eeprom_setup(); // настраиваем EEPROM
  read_modbus_settings();
  modbus_setup();  // настраиваем Modbus

  EEPROM.get(EEPROM_MB_ADDRESS, mbAddress);
  Serial.printf("mbAddress: %d\n", mbAddress);

  EEPROM.get(EEPROM_MB_STOP_BITS, mbStopBits);
  Serial.printf("mbStopBits: %d\n", mbStopBits);
  
  EEPROM.get(EEPROM_MB_BAUDRATE, mbBaudrate);
  Serial.printf("mbBaudrate: %d\n", mbBaudrate);

  ld2410_setup();

}

void ld2410_setup() {
  Serial2.begin(LD2410_SPEED, SERIAL_8N1, LD2410_RX, LD2410_TX);
  if(radar.begin(Serial2))
  {
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("not connected"));
  }

  radar.read();
  if(!radar.isConnected()){
    return;
  }
  if(!radar.requestCurrentConfiguration()) {
    return;
  }
  SENSOR_MAX_MOVING_GATE = radar.max_moving_gate;
  SENSOR_MAX_STATIONARY_GATE = radar.max_stationary_gate;
  SENSOR_IDLE_TIME = radar.sensor_idle_time;
  Serial.printf("%d; %d; %d;\n", SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
}

SerialConfig convert_stop_bits_to_config(uint16_t stopBits);  
uint16_t callback_set_mb_reg(TRegister* reg, uint16_t val);
uint16_t callback_get_sensor_reg(TRegister* reg, uint16_t val);

void modbus_setup() {
  Serial1.begin(convert_baudrate(mbBaudrate), convert_stop_bits_to_config(mbStopBits), RX_PIN, TX_PIN); // задаём парамеры связи
  mb.begin(&Serial1);
  mb.begin(&Serial1, PIN_FLOW); // включаем контроль направления приёма/передачи
  mb.slave(mbAddress); // указываем адрес нашего сервера

  // описываем три holding регистра
  mb.addHreg(REG_MB_ADDRESS);   // адрес устройства на шине
  mb.addHreg(REG_MB_STOP_BITS); // стоповые биты
  mb.addHreg(REG_MB_BAUDRATE);  // скорость подключения
  mb.addHreg(REG_SENSOR_MAX_MOVING_GATE);  // скорость подключения
  mb.addHreg(REG_SENSOR_MAX_STATIONARY_GATE);  // скорость подключения
  mb.addHreg(REG_SENSOR_IDLE_TIME);  // скорость подключения
  mb.addIreg(REG_SENSOR_PRESENCE);  // датчик присутствия
  mb.addIreg(REG_SENSOR_STATIONARY);  // датчик присутствия
  mb.addIreg(REG_SENSOR_STATIONARY_TARGET_DISTANCE);  // датчик присутствия
  mb.addIreg(REG_SENSOR_STATIONARY_TARGET_ENERGY);  // датчик присутствия
  mb.addIreg(REG_SENSOR_MOVING);  // датчик присутствия
  mb.addIreg(REG_SENSOR_MOVING_TARGET_DISTANCE);  // датчик присутствия
  mb.addIreg(REG_SENSOR_MOVING_TARGET_ENERGY);  // датчик присутствия

  // записываем в регистры значения адреса, стоповых битов и скорости
  mb.Hreg(REG_MB_ADDRESS, mbAddress);
  mb.Hreg(REG_MB_STOP_BITS, mbStopBits);
  mb.Hreg(REG_MB_BAUDRATE, mbBaudrate);
  mb.Hreg(REG_SENSOR_MAX_MOVING_GATE, DEFAULT_SENSOR_MAX_MOVING_GATE);
  mb.Hreg(REG_SENSOR_MAX_STATIONARY_GATE, DEFAULT_SENSOR_MAX_STATIONARY_GATE);
  mb.Hreg(REG_SENSOR_IDLE_TIME, DEFAULT_SENSOR_IDLE_TIME);
  mb.Ireg(REG_SENSOR_PRESENCE, 0); 
  mb.Ireg(REG_SENSOR_STATIONARY, 0); 
  mb.Ireg(REG_SENSOR_STATIONARY_TARGET_DISTANCE, 0); 
  mb.Ireg(REG_SENSOR_STATIONARY_TARGET_ENERGY, 0); 
  mb.Ireg(REG_SENSOR_MOVING, 0); 
  mb.Ireg(REG_SENSOR_MOVING_TARGET_DISTANCE, 0); 
  mb.Ireg(REG_SENSOR_MOVING_TARGET_ENERGY, 0); 

  // описываем колбек функцию, которая будет вызвана при записи регистров
  // параметров подключения
  mb.onSetHreg(REG_MB_ADDRESS, callback_set_mb_reg);
  mb.onSetHreg(REG_MB_STOP_BITS, callback_set_mb_reg);
  mb.onSetHreg(REG_MB_BAUDRATE, callback_set_mb_reg); 
  
  mb.onSetHreg(REG_SENSOR_MAX_MOVING_GATE, callback_set_mb_reg); 
  mb.onSetHreg(REG_SENSOR_MAX_STATIONARY_GATE, callback_set_mb_reg); 
  mb.onSetHreg(REG_SENSOR_IDLE_TIME, callback_set_mb_reg);
  
  mb.onGetHreg(REG_SENSOR_MAX_MOVING_GATE, callback_get_sensor_reg); 
  mb.onGetHreg(REG_SENSOR_MAX_STATIONARY_GATE, callback_get_sensor_reg); 
  mb.onGetHreg(REG_SENSOR_IDLE_TIME, callback_get_sensor_reg); 
  
  mb.onGetIreg(REG_SENSOR_PRESENCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY_TARGET_DISTANCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY_TARGET_ENERGY, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING_TARGET_DISTANCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING_TARGET_ENERGY, callback_get_sensor_reg);
  
}

// Конвертер стоповых битов в настройки типа SerialConfig.
// Я не стал реализовывать все возможные варианты
// и давать настраивать чётность и количество битов данных, так как
// почти никто эти параметры при работе по протоколу Modbus не меняет
SerialConfig convert_stop_bits_to_config(uint16_t stopBits) {
  return (stopBits == 2) ? SERIAL_8N2 : SERIAL_8N1;
}

// Конвертер значения скорости. Для экономии места во флеше и регистрах, мы храним
// значение скорости, делённое на 100. То есть вместо 9600, мы храним 96.
// Эта функция умножает значение на 100.
uint32_t convert_baudrate(uint16_t baudrateValue) {
  return baudrateValue * 100;
}

// Функция, которая находит вхождение числа в массив
bool contains(uint16_t a, uint16_t arr[], uint8_t arr_size) {
  for (uint8_t i = 0; i < arr_size; i++) if (a == arr[i]) return true;
  return false;
}

// запись значения в EEPROM
void write_eeprom(uint8_t eepromm_address, uint16_t val) {
  EEPROM.put(eepromm_address, val);
  EEPROM.commit();
}

uint16_t callback_get_sensor_reg(TRegister* reg, uint16_t val) {
  switch(reg->address.address) {
    case REG_SENSOR_PRESENCE:
      return SENSOR_PRESENCE;
    case REG_SENSOR_MOVING:
      return SENSOR_MOVING;
    case REG_SENSOR_STATIONARY:
      return SENSOR_STATIONARY;
    case REG_SENSOR_MOVING_TARGET_DISTANCE:
      return SENSOR_MOVING_TARGET_DISTANCE;
    case REG_SENSOR_MOVING_TARGET_ENERGY:
      return SENSOR_MOVING_TARGET_ENERGY;
    case REG_SENSOR_STATIONARY_TARGET_DISTANCE:
      return SENSOR_STATIONARY_TARGET_DISTANCE;
    case REG_SENSOR_STATIONARY_TARGET_ENERGY:
      return SENSOR_STATIONARY_TARGET_ENERGY;      
    case REG_SENSOR_MAX_MOVING_GATE:
      return SENSOR_MAX_MOVING_GATE; 
    case REG_SENSOR_MAX_STATIONARY_GATE:
      return SENSOR_MAX_STATIONARY_GATE;
    case REG_SENSOR_IDLE_TIME:
      return SENSOR_IDLE_TIME;           
  }
  return 0;
}

/* Вспомогательные функции */
// Колбек функция в которой мы записываем полученные по Modbus регистры в EEPROMM.
// Не забываем проверять записываемые значения на корректность, иначе мы можем потерять
// связь с устройством



uint16_t callback_set_mb_reg(TRegister* reg, uint16_t val) {
  uint16_t correctGates[] = {2,3,4,5,6,7,8};
  uint16_t correctBaudRates[] = {12, 24, 48, 96, 192, 384, 576, 1152};
  switch (reg->address.address) {
    case REG_MB_ADDRESS: // если записываем регистр с адресом
      if (val > 0 && val < 247) { // проверяем, что записываемое число корректно
        write_eeprom(EEPROM_MB_ADDRESS, val); // записываем значение в EEPROM
      } else {
        val = reg->value; // этот трюк сгенерирует ошибку записи, что нам и нужно, так как значение неверное
      }
      break;
    case REG_MB_STOP_BITS: // если регистр со стоповыми битами
      if (val == 1 || val == 2) {
        write_eeprom(EEPROM_MB_STOP_BITS, val);
      } else {
        val = reg->value;
      }
      break;
    case REG_MB_BAUDRATE: // если регистр со скоростью
      if (contains(val, correctBaudRates, 8)) {
        write_eeprom(EEPROM_MB_BAUDRATE, val);
      } else {
        val = reg->value;
      }
      break;
    case REG_SENSOR_MAX_MOVING_GATE:
      if (contains(val, correctGates, 7)) {
        SENSOR_MAX_MOVING_GATE = val;
        radar.setMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
      } else {
        val = reg->value;
      }
      break;
    case REG_SENSOR_MAX_STATIONARY_GATE:
      if (contains(val, correctGates, 7)) {
        SENSOR_MAX_STATIONARY_GATE = val;
        radar.setMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
      } else {
        val = reg->value;
      }
      break;
    case REG_SENSOR_IDLE_TIME:
      SENSOR_IDLE_TIME = val;
      radar.setMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
      break;
  }
  return val;
}


// Настройка параметров EEPROM
void eeprom_setup() {
  EEPROM.begin(EEPROM_SIZE);
}

// Чтение настроек Modbus: сперва читаем из EEPROM,
// а если там пусто, то берём значения по умолчанию
void read_modbus_settings() {
  EEPROM.get(EEPROM_MB_ADDRESS, mbAddress);
  if (mbAddress == 0xffff) {
    mbAddress = DEFAULT_MB_ADDRESS;
  }

  EEPROM.get(EEPROM_MB_STOP_BITS, mbStopBits);
  if (mbStopBits == 0xffff) {
    mbStopBits = DEFAULT_MB_STOP_BITS;
  }

  EEPROM.get(EEPROM_MB_BAUDRATE, mbBaudrate);
  if (mbBaudrate == 0xffff) {
    mbBaudrate = DEFAULT_MB_BAUDRATE;
  };
}

void pollLD2410() {

  radar.read();

  if(radar.isConnected()){
    SENSOR_PRESENCE = radar.presenceDetected();
    SENSOR_MOVING = radar.movingTargetDetected();
    SENSOR_STATIONARY = radar.stationaryTargetDetected();
    SENSOR_STATIONARY_TARGET_DISTANCE = radar.stationaryTargetDistance();
    SENSOR_STATIONARY_TARGET_ENERGY = radar.stationaryTargetEnergy();
    SENSOR_MOVING_TARGET_DISTANCE = radar.movingTargetDistance();
    SENSOR_MOVING_TARGET_ENERGY = radar.movingTargetEnergy(); 
  }

  if(Serial.available())
  {
    char typedCharacter = Serial.read();
    if(typedCharacter == '\r' || typedCharacter == '\n')
    {
      command.trim();
      if(command.equals("read"))
      {
        command = "";
        Serial.print(F("Reading from sensor: "));
        if(radar.isConnected())
        {
          Serial.println(F("OK"));
          if(radar.presenceDetected())
          {
            if(radar.stationaryTargetDetected())
            {
              Serial.print(F("Stationary target: "));
              Serial.print(radar.stationaryTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.stationaryTargetEnergy());
            }
            if(radar.movingTargetDetected())
            {
              Serial.print(F("Moving target: "));
              Serial.print(radar.movingTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.movingTargetEnergy());
            }
          }
          else
          {
            Serial.println(F("nothing detected"));
          }
        }
        else
        {
          Serial.println(F("failed to read"));
        }
      }
      else if(command.equals("readconfig"))
      {
        command = "";
        Serial.print(F("Reading configuration from sensor: "));
        if(radar.requestCurrentConfiguration())
        {
          Serial.println(F("OK"));
          Serial.print(F("Maximum gate ID: "));
          Serial.println(radar.max_gate);
          Serial.print(F("Maximum gate for moving targets: "));
          Serial.println(radar.max_moving_gate);
          Serial.print(F("Maximum gate for stationary targets: "));
          Serial.println(radar.max_stationary_gate);
          Serial.print(F("Idle time for targets: "));
          Serial.println(radar.sensor_idle_time);
          Serial.println(F("Gate sensitivity"));
          for(uint8_t gate = 0; gate <= radar.max_gate; gate++)
          {
            Serial.print(F("Gate "));
            Serial.print(gate);
            Serial.print(F(" moving targets: "));
            Serial.print(radar.motion_sensitivity[gate]);
            Serial.print(F(" stationary targets: "));
            Serial.println(radar.stationary_sensitivity[gate]);
          }
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.startsWith("setmaxvalues"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t newMovingMaxDistance = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t newStationaryMaxDistance = (command.substring(secondSpace,thirdSpace)).toInt();
        uint16_t inactivityTimer = (command.substring(thirdSpace,command.length())).toInt();
        if(newMovingMaxDistance > 0 && newStationaryMaxDistance > 0 && newMovingMaxDistance <= 8 && newStationaryMaxDistance <= 8)
        {
          Serial.print(F("Setting max values to gate "));
          Serial.print(newMovingMaxDistance);
          Serial.print(F(" moving targets, gate "));
          Serial.print(newStationaryMaxDistance);
          Serial.print(F(" stationary targets, "));
          Serial.print(inactivityTimer);
          Serial.print(F("s inactivity timer: "));
          command = "";
          if(radar.setMaxValues(newMovingMaxDistance, newStationaryMaxDistance, inactivityTimer))
          {
            Serial.println(F("OK, now restart to apply settings"));
          }
          else
          {
            Serial.println(F("failed"));
          }
        }
        else
        {
          Serial.print(F("Can't set distances to "));
          Serial.print(newMovingMaxDistance);
          Serial.print(F(" moving "));
          Serial.print(newStationaryMaxDistance);
          Serial.println(F(" stationary, try again"));
          command = "";
        }
      }
      else if(command.startsWith("setsensitivity"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t gate = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t motionSensitivity = (command.substring(secondSpace,thirdSpace)).toInt();
        uint8_t stationarySensitivity = (command.substring(thirdSpace,command.length())).toInt();
        if(motionSensitivity >= 0 && stationarySensitivity >= 0 && motionSensitivity <= 100 && stationarySensitivity <= 100)
        {
          Serial.print(F("Setting gate "));
          Serial.print(gate);
          Serial.print(F(" motion sensitivity to "));
          Serial.print(motionSensitivity);
          Serial.print(F(" & stationary sensitivity to "));
          Serial.print(stationarySensitivity);
          Serial.println(F(": "));
          command = "";
          if(radar.setGateSensitivityThreshold(gate, motionSensitivity, stationarySensitivity))
          {
            Serial.println(F("OK, now restart to apply settings"));
          }
          else
          {
            Serial.println(F("failed"));
          }
        }
        else
        {
          Serial.print(F("Can't set gate "));
          Serial.print(gate);
          Serial.print(F(" motion sensitivity to "));
          Serial.print(motionSensitivity);
          Serial.print(F(" & stationary sensitivity to "));
          Serial.print(stationarySensitivity);
          Serial.println(F(", try again"));
          command = "";
        }
      }
      else if(command.equals("enableengineeringmode"))
      {
        command = "";
        Serial.print(F("Enabling engineering mode: "));
        if(radar.requestStartEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("disableengineeringmode"))
      {
        command = "";
        Serial.print(F("Disabling engineering mode: "));
        if(radar.requestEndEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("restart"))
      {
        command = "";
        Serial.print(F("Restarting sensor: "));
        if(radar.requestRestart())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("readversion"))
      {
        command = "";
        Serial.print(F("Requesting firmware version: "));
        if(radar.requestFirmwareVersion())
        {
          Serial.print('v');
          Serial.print(radar.firmware_major_version);
          Serial.print('.');
          Serial.print(radar.firmware_minor_version);
          Serial.print('.');
          Serial.println(radar.firmware_bugfix_version,HEX);
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.equals("factoryreset"))
      {
        command = "";
        Serial.print(F("Factory resetting sensor: "));
        if(radar.requestFactoryReset())
        {
          Serial.println(F("OK, now restart sensor to take effect"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else
      {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        command = "";
      }
    }
    else
    {
      command += typedCharacter;
    }
  }


}

void loop() {
  mb.task();
  pollLD2410();
}



WiFiClient wifiClient;

char ssid[6] = "q.O.p";
char password[13] = "ILoveYouBaby";

void setupWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  char emptyMsg[1] = "";
  char pointMsg[2] = ".";
  Serial.println(emptyMsg);

    // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(pointMsg);
  }
  Serial.println(emptyMsg);
  char connectedToMsg[14] = "Connected to ";
  Serial.println(connectedToMsg);
  Serial.println(ssid);
  char ipAddrMsg[30];
  sprintf(ipAddrMsg, "IP address: %s", WiFi.localIP().toString().c_str());
  Serial.println(ipAddrMsg);

}

unsigned long ota_progress_millis = 0;
AsyncWebServer server(80);


void setupOTA(void) {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is a sample response.");
  });

  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}
