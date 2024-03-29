#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>
#include "./ld-2410.h"
#include "./modbus-slave.h"

// Настройка Modbus
#define SLAVE_ID  1 // modbus slave ID по умолчанию
#define PIN_FLOW  25 // пин контроля направления приёма/передачи,
#define RX_PIN    33 //пин uart для приёма
#define TX_PIN    32 //пин uart для отправки

#define EEPROM_SIZE           6 // мы займём 6 ячеек памяти: 3*2=6
#define EEPROM_MB_ADDRESS     0 // номер ячейки с адресом устройства
#define EEPROM_MB_STOP_BITS   2 // номер ячейки со стоп-битами
#define EEPROM_MB_BAUDRATE    4 // номер ячейки со скоростью

#define DEFAULT_MB_ADDRESS    1  // адрес нашего сервера
#define DEFAULT_MB_STOP_BITS  2  // количество стоповых битов
#define DEFAULT_MB_BAUDRATE   96 // скорость подключения/100

//адрес регистров
#define REG_MB_ADDRESS                          100 //modbus serial slave id
#define REG_MB_STOP_BITS                        101 //modbus serial stop bits
#define REG_MB_BAUDRATE                         102 //modbus serial baudrate
#define REG_SENSOR_PRESENCE                     0 //присутствие
#define REG_SENSOR_STATIONARY                   1 //неподвижность
#define REG_SENSOR_MOVING                       2 //движение
#define REG_SENSOR_STATIONARY_TARGET_DISTANCE   3 //расстояние до недвижимого объекта
#define REG_SENSOR_STATIONARY_TARGET_ENERGY     4 //энергия недвижимого объекта
#define REG_SENSOR_MOVING_TARGET_DISTANCE       5 //расстояние до движущегося объекта
#define REG_SENSOR_MOVING_TARGET_ENERGY         6 //энергия недвижимого объекта
#define REG_SENSOR_MAX_MOVING_GATE              103 //максимальное расстояние до движущегося объекта
#define REG_SENSOR_MAX_STATIONARY_GATE          104 //максимальное расстояние до неподвижного объекта
#define REG_SENSOR_IDLE_TIME                    105 //idle time

//runtime переменные
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

//настройки по умолчанию для датчика
#define DEFAULT_SENSOR_MAX_MOVING_GATE      4
#define DEFAULT_SENSOR_MAX_STATIONARY_GATE  4
#define DEFAULT_SENSOR_IDLE_TIME            4

uint16_t mbAddress = DEFAULT_MB_ADDRESS; // modbus адрес устройства
uint16_t mbStopBits = DEFAULT_MB_STOP_BITS; // количество стоповых битов
uint16_t mbBaudrate = DEFAULT_MB_BAUDRATE; // скорость подключения modbus


//инстанс для работы с modbus
ModbusRTU mb;

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
        radarSetMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME); 
      } else {
        val = reg->value;
      }
      break;
    case REG_SENSOR_MAX_STATIONARY_GATE:
      if (contains(val, correctGates, 7)) {
        SENSOR_MAX_STATIONARY_GATE = val;
        radarSetMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
      } else {
        val = reg->value;
      }
      break;
    case REG_SENSOR_IDLE_TIME:
      SENSOR_IDLE_TIME = val;
      radarSetMaxValues(SENSOR_MAX_MOVING_GATE, SENSOR_MAX_STATIONARY_GATE, SENSOR_IDLE_TIME);
      break;
  }
  return val;
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


void modbus_setup() {
  Serial1.begin(convert_baudrate(mbBaudrate), convert_stop_bits_to_config(mbStopBits), RX_PIN, TX_PIN); // задаём парамеры связи
  mb.begin(&Serial1);
  mb.begin(&Serial1, PIN_FLOW); // включаем контроль направления приёма/передачи
  mb.slave(mbAddress); // указываем адрес нашего сервера

  // holding регистры
  mb.addHreg(REG_MB_ADDRESS);   // адрес устройства на шине
  mb.addHreg(REG_MB_STOP_BITS); // стоповые биты
  mb.addHreg(REG_MB_BAUDRATE);  // скорость подключения
  mb.addHreg(REG_SENSOR_MAX_MOVING_GATE);  // настройка максимального расстояния до движущегося объекта
  mb.addHreg(REG_SENSOR_MAX_STATIONARY_GATE);  // настройка максимального расстояния до неподвижного объекта
  mb.addHreg(REG_SENSOR_IDLE_TIME);  // настройка Idle time

  // input регистры (для чтения показаний датчика)
  mb.addIreg(REG_SENSOR_PRESENCE); 
  mb.addIreg(REG_SENSOR_STATIONARY);  
  mb.addIreg(REG_SENSOR_STATIONARY_TARGET_DISTANCE); 
  mb.addIreg(REG_SENSOR_STATIONARY_TARGET_ENERGY);  
  mb.addIreg(REG_SENSOR_MOVING); 
  mb.addIreg(REG_SENSOR_MOVING_TARGET_DISTANCE); 
  mb.addIreg(REG_SENSOR_MOVING_TARGET_ENERGY); 

  // записываем в регистры значения адреса, стоповых битов и скорости
  mb.Hreg(REG_MB_ADDRESS, mbAddress);
  mb.Hreg(REG_MB_STOP_BITS, mbStopBits);
  mb.Hreg(REG_MB_BAUDRATE, mbBaudrate);
  mb.Hreg(REG_SENSOR_MAX_MOVING_GATE, DEFAULT_SENSOR_MAX_MOVING_GATE);
  mb.Hreg(REG_SENSOR_MAX_STATIONARY_GATE, DEFAULT_SENSOR_MAX_STATIONARY_GATE);
  mb.Hreg(REG_SENSOR_IDLE_TIME, DEFAULT_SENSOR_IDLE_TIME);

  // инициализируем input-регистры
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
  
  //колбеки для изменения настроек датчика
  mb.onSetHreg(REG_SENSOR_MAX_MOVING_GATE, callback_set_mb_reg); 
  mb.onSetHreg(REG_SENSOR_MAX_STATIONARY_GATE, callback_set_mb_reg); 
  mb.onSetHreg(REG_SENSOR_IDLE_TIME, callback_set_mb_reg);

  //колбеки для чтения настроек датчика
  mb.onGetHreg(REG_SENSOR_MAX_MOVING_GATE, callback_get_sensor_reg); 
  mb.onGetHreg(REG_SENSOR_MAX_STATIONARY_GATE, callback_get_sensor_reg); 
  mb.onGetHreg(REG_SENSOR_IDLE_TIME, callback_get_sensor_reg); 
  
  //колбеки для чтения показаний датчика
  mb.onGetIreg(REG_SENSOR_PRESENCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY_TARGET_DISTANCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_STATIONARY_TARGET_ENERGY, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING_TARGET_DISTANCE, callback_get_sensor_reg);
  mb.onGetIreg(REG_SENSOR_MOVING_TARGET_ENERGY, callback_get_sensor_reg);
  
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

  EEPROM.get(EEPROM_MB_ADDRESS, mbAddress);
  Serial.printf("mbAddress: %d\n", mbAddress);

  EEPROM.get(EEPROM_MB_STOP_BITS, mbStopBits);
  Serial.printf("mbStopBits: %d\n", mbStopBits);
  
  EEPROM.get(EEPROM_MB_BAUDRATE, mbBaudrate);
  Serial.printf("mbBaudrate: %d\n", mbBaudrate);

}

void mbTask() {
    mb.task();
}

void setSENSOR_PRESENCE(bool v) {
    SENSOR_PRESENCE = v;
}

void setSENSOR_MOVING(bool v) {
    SENSOR_MOVING = v;
}
void setSENSOR_STATIONARY(bool v) {
    SENSOR_STATIONARY = v;
}
void setSENSOR_STATIONARY_TARGET_DISTANCE(uint16_t v) {
    SENSOR_STATIONARY_TARGET_DISTANCE = v;
}
void setSENSOR_STATIONARY_TARGET_ENERGY(uint16_t v) {
    SENSOR_STATIONARY_TARGET_ENERGY = v;
}
void setSENSOR_MOVING_TARGET_DISTANCE(uint16_t v) {
    SENSOR_MOVING_TARGET_DISTANCE = v;
}
void setSENSOR_MOVING_TARGET_ENERGY(uint16_t v) {
    SENSOR_MOVING_TARGET_ENERGY = v;
}


void setSENSOR_MAX_MOVING_GATE(uint8_t v) {
    SENSOR_MAX_MOVING_GATE = v;
}
void setSENSOR_MAX_STATIONARY_GATE(uint8_t v) {
    SENSOR_MAX_STATIONARY_GATE = v;
}
void setSENSOR_IDLE_TIME(uint8_t v) {
    SENSOR_IDLE_TIME = v;
}
