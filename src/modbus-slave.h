void modbus_setup();
void mbTask();
void eeprom_setup();
void read_modbus_settings();
void setSENSOR_PRESENCE(bool v);
void setSENSOR_MOVING(bool v);
void setSENSOR_STATIONARY(bool v);
void setSENSOR_STATIONARY_TARGET_DISTANCE(uint16_t v);
void setSENSOR_STATIONARY_TARGET_ENERGY(uint16_t v);
void setSENSOR_MOVING_TARGET_DISTANCE(uint16_t v);
void setSENSOR_MOVING_TARGET_ENERGY(uint16_t v);
void setSENSOR_MAX_MOVING_GATE(uint8_t v);
void setSENSOR_MAX_STATIONARY_GATE(uint8_t v);
void setSENSOR_IDLE_TIME(uint8_t v);
